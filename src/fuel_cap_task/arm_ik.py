"""当前模型的端点逆运动学；不执行轨迹，也不作为抓稳验收。"""
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from .fixed_grasp import load_fixed_grasp
from .interfaces import TaskParameters
from .grasp_search import quaternion
from .grasp_targets import inverse, transform
from .grasp_preparation import prepare_grasp
from .mujoco_scene import ARM_JOINTS, MujocoCapScene


@dataclass(frozen=True)
class IKSettings:
    seeds: int = 24
    iterations: int = 250
    random_seed: int = 0
    position_tolerance_m: float = .0001
    orientation_tolerance_rad: float = np.deg2rad(.1)
    rotation_weight_m: float = .2
    damping: float = .01
    max_step_rad: float = .2
    penetration_diagnostic_m: float = .0001


def body_transform(data, name):
    b = data.body(name)
    return transform(b.xpos.copy(), b.xmat.reshape(3, 3).copy())


def rotation_error(target, actual):
    q = np.asarray(quaternion(target @ actual.T))
    if q[3] < 0:
        q = -q
    length = np.linalg.norm(q[:3])
    return 2*q[:3] if length < 1e-12 else q[:3] * (2*np.arctan2(length, q[3])/length)


class ArmIKPreview:
    def __init__(self, candidate_path, settings=None, *, scene=None, approach_distance_m=None):
        self.settings = settings or IKSettings()
        self.scene = scene if scene is not None else MujocoCapScene()
        self.model, self.data = self.scene.model, self.scene.data
        self.candidate_path = Path(candidate_path).expanduser().resolve()
        self.fixed_grasp = load_fixed_grasp(self.candidate_path, source_scene_path=self.scene_path,
                                            expected_mujoco_version=mujoco.__version__)
        self.scene_hash = self.fixed_grasp.provenance.source_scene_sha256
        # 原 JSON 仅保留闭合实验等显示记录；摘要须与公共读取器真正检查的字节一致。
        raw = self.candidate_path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != self.fixed_grasp.provenance.candidate_sha256:
            raise ValueError('Candidate changed while loading')
        self.candidate = json.loads(raw)
        if self.fixed_grasp.grasp_channels != TaskParameters().grasp_channels:
            raise ValueError('Candidate final target does not match current task parameters')
        self.scene.hand.apply(self.data, self.fixed_grasp.pregrasp_channels)
        ids = [self.model.joint(name).id for name in ARM_JOINTS]
        self.addresses = self.model.jnt_qposadr[ids]
        self.dofs = self.model.jnt_dofadr[ids]
        self.limits = self.model.jnt_range[ids]
        self.initial = self.data.qpos[self.addresses].copy()
        self.flange = self.model.body('wrist3_Link').id
        # 固定安装链给出法兰到手根的变换；base 是手根，base_link 才是臂基座。
        self.flange_from_hand = inverse(body_transform(self.data, 'wrist3_Link')) @ body_transform(self.data, 'base')
        # 通过估计接口获取盖体位姿；当前场景的来源仍为仿真真值。
        self.estimate = self.scene.estimator.read()
        self.preparation = prepare_grasp(
            self.fixed_grasp, self.estimate,
            flange_from_hand=self.flange_from_hand,
            world_from_base=body_transform(self.data, 'base_link'),
            approach_distance_m=approach_distance_m,
        )
        targets = self.preparation.targets
        self.world_from_cap = targets.world_from_cap
        self.world_from_hand = targets.world_from_hand
        self.world_from_flange = targets.world_from_flange
        self.base_from_hand = targets.base_from_hand
        self.base_from_flange = targets.base_from_flange
        self.robot_bodies = set()
        base_id = self.model.body('base_link').id
        for b in range(self.model.nbody):
            parent = b
            while parent and parent != base_id:
                parent = int(self.model.body_parentid[parent])
            if parent == base_id:
                self.robot_bodies.add(b)
        self.initial_contacts = self.contacts()

    @property
    def scene_path(self):
        from .mujoco_scene import DEFAULT_SCENE
        return DEFAULT_SCENE

    def forward(self, q):
        self.data.qpos[self.addresses] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def error(self):
        actual = body_transform(self.data, 'wrist3_Link')
        return np.r_[self.world_from_flange[:3, 3]-actual[:3, 3],
                     rotation_error(self.world_from_flange[:3, :3], actual[:3, :3])]

    def contacts(self):
        mujoco.mj_forward(self.model, self.data)
        result = []
        for contact in self.data.contact:
            a, b = map(int, contact.geom)
            if not any(int(self.model.geom_bodyid[g]) in self.robot_bodies for g in (a, b)):
                continue
            if contact.dist >= 0:
                continue
            result.append({'geoms': [self.model.geom(g).name or self.model.body(self.model.geom_bodyid[g]).name for g in (a,b)],
                           'penetration_m': float(-contact.dist)})
        return result

    def show(self, solution, closed=False):
        self.scene.hand.apply(self.data, self.fixed_grasp.pregrasp_channels)
        if closed:
            trial = self.candidate.get('contact_trial') or {}
            q = trial.get('final_qpos')
            # 现有固定夹具模型仅包含手部关节；通过该模型的关节名匹配记录值，
            # 不使用数组偏移猜测对应关系。
            if q is None:
                raise ValueError('Candidate has no recorded actual closure')
            from .grasp_contact import ContactExplorer
            local = ContactExplorer().scene.model
            if len(q) != local.nq:
                raise ValueError('Recorded closure does not match local hand model')
            for j in self.scene.hand.joint_ids:
                local_joint = local.joint(self.model.joint(j).name)
                self.data.qpos[self.model.jnt_qposadr[j]] = q[int(local_joint.qposadr[0])]
        self.data.qpos[self.addresses] = solution['arm_qpos_rad']
        mujoco.mj_forward(self.model, self.data)

    def solve(self, progress=None):
        cfg = self.settings
        rng = np.random.default_rng(cfg.random_seed)
        seeds = [self.initial, np.zeros(6)] + [rng.uniform(np.maximum(self.limits[:,0],-np.pi),
                 np.minimum(self.limits[:,1],np.pi)) for _ in range(cfg.seeds-2)]
        weight = np.array([1.,1.,1.,cfg.rotation_weight_m,cfg.rotation_weight_m,cfg.rotation_weight_m])
        jp, jr = np.zeros((3,self.model.nv)), np.zeros((3,self.model.nv))
        solutions, best = [], None
        for seed_index, seed in enumerate(seeds):
            q = seed.copy()
            for iteration in range(cfg.iterations):
                self.forward(q)
                error = self.error()
                score = float(np.linalg.norm(error*weight))
                if best is None or score < best['score']:
                    best = {'score': score, 'arm_qpos_rad': q.tolist(),
                            'position_error_m': float(np.linalg.norm(error[:3])),
                            'orientation_error_rad': float(np.linalg.norm(error[3:]))}
                if (np.linalg.norm(error[:3]) <= cfg.position_tolerance_m
                        and np.linalg.norm(error[3:]) <= cfg.orientation_tolerance_rad):
                    break
                mujoco.mj_jacBody(self.model,self.data,jp,jr,self.flange)
                j = np.vstack((jp[:,self.dofs],jr[:,self.dofs]))*weight[:,None]
                dq = j.T @ np.linalg.solve(j@j.T + cfg.damping**2*np.eye(6),error*weight)
                dq *= min(1.,cfg.max_step_rad/max(np.linalg.norm(dq),1e-12))
                improved = False
                for scale in (1.,.5,.25,.125,.0625):
                    trial = np.clip(q+scale*dq,self.limits[:,0],self.limits[:,1])
                    self.forward(trial)
                    if np.linalg.norm(self.error()*weight) < score:
                        q, improved = trial, True
                        break
                if not improved:
                    break
            self.forward(q)
            error = self.error()
            if (np.linalg.norm(error[:3]) <= cfg.position_tolerance_m
                    and np.linalg.norm(error[3:]) <= cfg.orientation_tolerance_rad):
                # 在限位内选择最接近初始预览姿态的等效转角。
                for axis in range(6):
                    variants = q[axis] + 2*np.pi*np.arange(-3, 4)
                    allowed = variants[(variants >= self.limits[axis,0]) & (variants <= self.limits[axis,1])]
                    q[axis] = allowed[np.argmin(np.abs(allowed-self.initial[axis]))]
                self.forward(q)
                if not any(np.linalg.norm(q-np.array(s['arm_qpos_rad'])) < .01 for s in solutions):
                    contacts = self.contacts()
                    hand = body_transform(self.data,'base')
                    solutions.append({'arm_qpos_rad': q.tolist(), 'seed_index': seed_index,
                        'hand_position_error_m': float(np.linalg.norm(hand[:3,3]-self.world_from_hand[:3,3])),
                        'hand_orientation_error_rad': float(np.linalg.norm(rotation_error(self.world_from_hand[:3,:3],hand[:3,:3]))),
                        'contacts': contacts, 'interference_count':sum(c['penetration_m']>cfg.penetration_diagnostic_m for c in contacts)})
            if progress:
                progress(f'IK seed {seed_index+1}/{len(seeds)}: {len(solutions)} distinct solutions')
        solutions.sort(key=lambda s:(s['interference_count'],np.linalg.norm(np.array(s['arm_qpos_rad'])-self.initial)))
        if solutions:
            self.show(solutions[0])
        else:
            self.forward(self.initial)
            mujoco.mj_forward(self.model,self.data)
        return {'mode':'current_model_endpoint_ik_no_execution','source_scene_sha256':self.scene_hash,
                'candidate_path':str(self.candidate_path), 'candidate_sha256':self.fixed_grasp.provenance.candidate_sha256,
                'candidate_provenance':asdict(self.fixed_grasp.provenance),
                'pregrasp_channels':list(self.fixed_grasp.pregrasp_channels),
                'grasp_channels':list(self.fixed_grasp.grasp_channels),
                **({'approach_targets':self.preparation.approach_record()} if self.preparation.approach is not None else {}),
                'settings':asdict(cfg),'joint_names':list(ARM_JOINTS),'joint_limits_rad':self.limits.tolist(),
                'cap_estimate':asdict(self.estimate),'flange_from_hand':self.flange_from_hand.tolist(),
                'world_from_hand_target':self.world_from_hand.tolist(),'base_from_hand_target':self.base_from_hand.tolist(),
                'world_from_flange_target':self.world_from_flange.tolist(),'base_from_flange_target':self.base_from_flange.tolist(),
                'initial_preview_contacts':self.initial_contacts,
                'solutions':solutions,'best_residual':best,'ik_found':bool(solutions),
                'enabled_geometry_interference_free_solution_found':any(s['interference_count']==0 for s in solutions),
                'physical_grasp_verified':False,'path_verified':False,
                'unchecked':['sensor_adapter_collision','ground_collision_disabled','approach_path','continuous_twist','loads','hardware_calibration']}
