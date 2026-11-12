"""原手根与六驱动坐标上的局部碰撞约束；完整几何检查仍决定接受。"""
import numpy as np
import torch
from scipy.optimize import minimize


def body_pose_jacobians(body_poses,q,needed=None):
    """每body共用12个变换分量的反传，batch各行彼此独立。"""
    result={}
    for body in sorted(body_poses if needed is None else needed):
        pose=body_poses[body][:,:3,:].sum(dim=0).reshape(-1)
        jac=torch.autograd.grad(pose,q,grad_outputs=torch.eye(12,device=q.device,dtype=q.dtype),
            is_grads_batched=True,retain_graph=True)[0]
        result[body]=jac.detach().cpu().numpy().transpose(1,0,2)
    return result


def contact_geometry_jacobians(hand,body_jacobians):
    """由同一body导数得到实际指肚采样点、单位法线对原q的导数。"""
    points=[];normals=[]
    weights=hand._sample_contact_points.detach().cpu().numpy()
    for key,basis in hand.contact_point_basis.items():
        body=key.split('+')[0]
        jac=body_jacobians[body].reshape(hand.batch_size,3,4,-1)
        local=np.einsum('wk,bkc->bwc',weights,basis.detach().cpu().numpy())
        points.append(np.einsum('bcij,bwi->bwcj',jac,local)*hand.scale)
        n=hand.contact_normals[key].detach().cpu().numpy().reshape(hand.batch_size,3)
        nj=np.einsum('bcij,bi->bcj',jac[:,:,:3],n)
        pose=hand.current_status[body].get_matrix()[:,:3,:3]
        R=(hand.global_rotation@pose).detach().cpu().numpy()
        world=np.einsum('bij,bj->bi',R,n);length=np.linalg.norm(world,axis=1)
        unit=world/length[:,None]
        nj=(nj-unit[:,:,None]*np.einsum('bi,bij->bj',unit,nj)[:,None,:])/length[:,None,None]
        normals.append(np.repeat(nj[:,None],len(weights),axis=1))
    return np.concatenate(points,axis=1),np.concatenate(normals,axis=1)


def scene_clearance_rows(avoidance,body_poses,q,*,active_band_m=.003,witnesses_per_geom=16,pose_jacobians=None):
    """返回每候选米制g及对原q列的Jacobian，约束为g+JΔq≥0。

    每个body只反传12个变换分量，密集见证点通过解析链式法则合成。
    FK的batch互相独立，先对batch求和反传即可同时得到各候选的导数。
    """
    rows=avoidance.near_scene_rows(body_poses,active_band_m=active_band_m,witnesses_per_geom=witnesses_per_geom)
    output=[]
    needed={body for row in rows for body in row['bodies']}
    body_jacobians=body_pose_jacobians(body_poses,q,needed) if pose_jacobians is None else pose_jacobians
    for b,row in enumerate(rows):
        g=row['gaps'];derivatives=np.zeros((len(g),q.shape[1]))
        for body in set(row['bodies']):
            use=np.asarray(row['bodies'])==body
            derivatives[use]=row['pose_partials'][use].reshape(-1,12)@body_jacobians[body][b]
        output.append(dict(gaps=g.detach().cpu().numpy().astype(float),
            jacobian=derivatives,keys=row['keys']))
    return output


def sufficient_contact_decrease(before,after,predicted_reduction):
    """实际误差须按预测取得明确下降，零步及非有限值均不能冒充进展。"""
    return bool(np.isfinite([before,after,predicted_reduction]).all() and predicted_reduction>1e-12
                and before-after>=max(1e-12,1e-4*predicted_reduction))


def solve_joint_residual_step(residual,jacobian,gaps,clearance_jacobian,lower,upper,scales,*,
                              damping=.01,interior_margin_m=1e-6):
    """直接最小化线性接触残差及缩放步长；无Adam动量方向。"""
    r,J,g,A,lo,hi,s=[np.asarray(v,dtype=float) for v in
        (residual,jacobian,gaps,clearance_jacobian,lower,upper,scales)]
    n=len(s)
    if (r.ndim!=1 or J.shape!=(len(r),n) or g.ndim!=1 or A.shape!=(len(g),n)
            or lo.shape!=(n,) or hi.shape!=(n,) or np.any(lo>hi) or np.any(s<=0)
            or not all(np.isfinite(v).all() for v in [r,J,g,A,lo,hi,s])
            or damping<=0 or not np.isfinite(damping)):
        raise ValueError('几何联合步输入、限位或尺度无效')
    K=J*s[None,:];matrix=A*s[None,:]/.001;gap=(g-interior_margin_m)/.001
    H=K.T@K+damping*np.eye(n);gradient=K.T@r
    constraints=[] if not len(g) else [dict(type='ineq',fun=lambda x:gap+matrix@x,jac=lambda x:matrix)]
    result=minimize(lambda x:float(.5*x@H@x+gradient@x),np.clip(np.zeros(n),lo/s,hi/s),
        jac=lambda x:H@x+gradient,bounds=list(zip(lo/s,hi/s)),constraints=constraints,
        method='SLSQP',options=dict(maxiter=100,ftol=1e-11))
    step=result.x*s
    feasible=(np.isfinite(step).all() and np.all(step>=lo-1e-10) and np.all(step<=hi+1e-10)
              and np.all(g-interior_margin_m+A@step>=-1e-9))
    if not feasible and interior_margin_m>0:
        return solve_joint_residual_step(r,J,g,A,lo,hi,s,damping=damping,interior_margin_m=0.)
    predicted=float(.5*(r@r-(r+J@step)@(r+J@step))) if feasible else 0.
    return (step if feasible else np.zeros(n)),dict(feasible=bool(feasible),solver_success=bool(result.success),
        predicted_reduction=predicted,interior_margin_m=interior_margin_m,message=str(result.message))


def project_joint_step(proposal,gaps,jacobian,lower,upper,scales,*,interior_margin_m=1e-6):
    """最小化缩放坐标中与Adam提议的距离，保持间隙、信赖范围及关节限位。

    3mm/旋转表示/关节的不同单位通过scales归一化。局部留1µm计算余量，
    避免恰好切向的解被转动二阶项阻挡；最终容差仍由完整检查决定。
    无内部余量可行解时退回原边界QP，不能把计算余量当新增物理限制。
    """
    p,g,A,lo,hi,s=[np.asarray(v,dtype=float) for v in (proposal,gaps,jacobian,lower,upper,scales)]
    n=len(p)
    if (p.ndim!=1 or g.ndim!=1 or A.shape!=(len(g),n) or any(v.shape!=p.shape for v in [lo,hi,s])
            or np.any(lo>hi) or np.any(s<=0) or not np.isfinite(interior_margin_m) or interior_margin_m<0
            or not all(np.isfinite(v).all() for v in [p,g,A,lo,hi,s])):
        raise ValueError('联合步的维度、限位、尺度或数值无效')
    target=p/s;matrix=A*s[None,:]/.001;gap=(g-interior_margin_m)/.001
    constraints=[] if not len(g) else [dict(type='ineq',fun=lambda x:gap+matrix@x,jac=lambda x:matrix)]
    result=minimize(lambda x:.5*np.sum((x-target)**2),np.clip(target,lo/s,hi/s),jac=lambda x:x-target,
        bounds=list(zip(lo/s,hi/s)),constraints=constraints,method='SLSQP',options=dict(maxiter=100,ftol=1e-10))
    value=result.x*s
    feasible=(np.isfinite(value).all() and np.all(value>=lo-1e-10) and np.all(value<=hi+1e-10)
              and np.all(g-interior_margin_m+A@value>=-1e-9))
    if not feasible and interior_margin_m>0:
        return project_joint_step(p,g,A,lo,hi,s,interior_margin_m=0.)
    return (value if feasible else np.zeros(n)),dict(feasible=bool(feasible),solver_success=bool(result.success),
        message=str(result.message),minimum_linear_gap_m=float((g+A@value).min()) if len(g) else None,
        interior_margin_m=interior_margin_m)
