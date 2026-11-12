"""固定接触点的原生手部FK；变量为手根6维切空间和六个模型驱动角。"""
import json
import hashlib
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .observed_hand_geometry import ObservedHandGeometry
from .rh56_drive_map import RH56DriveMap

FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')


def skew(v):
    """叉乘矩阵，用于左乘旋转增量的点/法向Jacobian。"""
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0.]])


def retract(pose, drivers, delta):
    """平移用m、左乘旋转及驱动用rad；围绕手根旋转，不绕场景原点。"""
    d = np.asarray(delta, dtype=float)
    if d.shape != (12,) or not np.isfinite(d).all():
        raise ValueError('运动学增量须为有限12维向量')
    result = np.array(pose, dtype=float, copy=True)
    result[:3, 3] += d[:3]
    result[:3, :3] = Rotation.from_rotvec(d[3:6]).as_matrix() @ result[:3, :3]
    return result, np.asarray(drivers)+d[6:]


def enumerate_assignments(features):
    """枚举拇指/任一对指的不同侧高分点；不修改分数或合成新目标。"""
    if (len({f['id'] for f in features}) != len(features)
            or any(f['side'] not in (-1, 1) for f in features)
            or {f['side'] for f in features} != {-1, 1}):
        raise ValueError('固定对应需要唯一点号和两侧接触目标')
    return [dict(thumb_feature=a['id'], partner_feature=b['id'], partner=finger)
            for a in features for b in features if a['side'] != b['side']
            for finger in FINGERS[1:]]


class NativeHandKinematics:
    """同源MuJoCo静态FK和Jacobian；从动关节始终由六驱动展开。"""
    def __init__(self, model_dir, scene_path):
        self.model_dir = Path(model_dir)
        self.metadata = json.loads((self.model_dir/'model.json').read_text())
        if self.metadata.get('unit') != 'm/rad':
            raise ValueError('手模型须采用米制')
        self.drives = RH56DriveMap.from_dict(self.metadata['mapping'])
        workspace = Path(__file__).resolve().parents[2]
        def verify_source(path):
            """运行模型必须绑定导出指面所用资产；副本也须保持原内容。"""
            path = Path(path).resolve()
            try:
                key = str(path.relative_to(workspace))
            except ValueError:
                key = str(path)
            expected = self.metadata['inputs'].get(key)
            if expected is None:
                matching = [sha for name, sha in self.metadata['inputs'].items() if Path(name).name == path.name]
                if len(matching) == 1:
                    expected = matching[0]
            if expected is None or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError('运行手模型与导出指面来源不符：'+str(path))
        verify_source(scene_path)
        self.hand = ObservedHandGeometry(scene_path)
        for path in self.hand.mesh_paths:
            verify_source(path)
        if self.metadata['source_mujoco_version'] != mujoco.__version__:
            raise ValueError('手模型与当前MuJoCo版本不一致')
        self.drive_dofs = np.zeros((self.hand.model.nv, 6))
        for i, name in enumerate(self.drives.joint_names):
            joint = self.hand.model.joint(name)
            if (not self.hand.model.jnt_limited[joint.id]
                    or not np.allclose(joint.range, [self.drives.full_lower[i], self.drives.full_upper[i]],
                                       atol=1e-8, rtol=0)):
                raise ValueError('运行手模型与驱动映射的关节限位不一致：'+name)
            self.drive_dofs[joint.dofadr[0]] = self.drives.matrix[i]
        self.pose = np.eye(4)
        self.set_state(self.pose, self.drives.lower)

    def set_state(self, pose, drivers):
        """先验证再设置，拒绝非刚体变换和超限值，不静默裁剪。"""
        p, d = np.asarray(pose, dtype=float), np.asarray(drivers, dtype=float)
        if (p.shape != (4, 4) or not np.isfinite(p).all()
                or not np.allclose(p[3], [0, 0, 0, 1], atol=1e-10)
                or not np.allclose(p[:3, :3].T@p[:3, :3], np.eye(3), atol=2e-6)
                or abs(np.linalg.det(p[:3, :3])-1) > 2e-6
                or d.shape != (6,) or not self.drives.validate(d, tolerance=1e-9)):
            raise ValueError('手根位姿或六驱动角无效')
        self.pose, self.drivers = p.copy(), d.copy()
        self.hand.place(self.drives.joint_names, self.drives.expand(d))

    def attached_point(self, body, point_hand):
        """当前hand_base系中、视为固连到body的点及12维位置Jacobian。"""
        h = self.hand
        body_id = h.model.body(body).id if isinstance(body, str) else int(body)
        jp, jr = np.zeros((3, h.model.nv)), np.zeros((3, h.model.nv))
        mujoco.mj_jac(h.model, h.data, jp, jr, np.asarray(point_hand), body_id)
        R, t = self.pose[:3, :3], self.pose[:3, 3]
        point = R@point_hand+t
        jacobian = np.column_stack([np.eye(3), -skew(point-t), R@jp@self.drive_dofs])
        angular = np.column_stack([np.zeros((3, 3)), np.eye(3), R@jr@self.drive_dofs])
        return point, jacobian, angular

    def point(self, link, point_link, normal_link):
        """真实模型局部点/单位法向 → 场景坐标和对12个增量的Jacobian。"""
        p, n = np.asarray(point_link, dtype=float), np.asarray(normal_link, dtype=float)
        if p.shape != (3,) or n.shape != (3,) or not np.isfinite([p, n]).all() or np.linalg.norm(n) < 1e-10:
            raise ValueError('接触点和法向须为有限三维量')
        body = self.hand.data.body(link)
        rh = body.xmat.reshape(3, 3)
        hand_point = body.xpos+rh@p
        point, jp, angular = self.attached_point(link, hand_point)
        normal = self.pose[:3, :3]@rh@(n/np.linalg.norm(n))
        return point, normal, jp, -skew(normal)@angular


def sample_pad_candidates(model_dir, samples_per_finger=32, seed=20261111):
    """五指闭合侧面固定采样；保留三角面和重心坐标，语义区域仍需查看核对。"""
    import trimesh
    if type(samples_per_finger) is not int or samples_per_finger < 1:
        raise ValueError('指面采样数须为正整数')
    model_dir = Path(model_dir)
    meta = json.loads((model_dir/'model.json').read_text())
    rng = np.random.RandomState(seed)
    output = {}
    for finger in FINGERS:
        region = next(r for r in meta['contact_regions'] if r['finger'] == finger)
        mesh = trimesh.load(model_dir/'meshes'/(region['link']+'.obj'), process=False, force='mesh')
        cosine = mesh.face_normals@np.asarray(region['closing_direction_link'])
        distance = np.linalg.norm(mesh.triangles_center-np.asarray(region['tip_site_link_m']), axis=1)
        valid = np.flatnonzero((cosine >= .5) & (distance <= .015) & (mesh.area_faces > 1e-10))
        if not len(valid):
            raise ValueError('闭合侧无有效指面：'+finger)
        faces = rng.choice(valid, samples_per_finger, p=mesh.area_faces[valid]/mesh.area_faces[valid].sum())
        uv = rng.uniform(size=(samples_per_finger, 2))
        root = np.sqrt(uv[:, 0])
        weights = np.column_stack([1-root, root*(1-uv[:, 1]), root*uv[:, 1]])
        points = np.einsum('ni,nij->nj', weights, mesh.triangles[faces])
        output[finger] = [dict(link=region['link'], point_link_m=p.tolist(), normal_link=n.tolist(),
                               face_id=int(face), barycentric=w.tolist(), sample_id=i)
                          for i, (p, n, face, w) in enumerate(zip(points, mesh.face_normals[faces], faces, weights))]
    return output
