"""围绕当前候选生成有边界场景变化；端点检查不代表路径或执行通过。"""
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .arm_ik import ArmIKPreview, body_transform
from .grasp_search import quaternion, rotation
from .mujoco_scene import DEFAULT_SCENE, MujocoCapScene


@dataclass(frozen=True)
class SamplingSettings:
    per_group: int = 4
    random_seed: int = 0
    x_min_m: float = 0.
    x_max_m: float = .2
    y_min_m: float = -1.
    y_max_m: float = -.8
    height_m: float | None = None
    max_horizontal_distance_m: float = 1.
    tank_yaw_limit_deg: float = 30.
    cap_yaw_limit_deg: float = 30.

    def __post_init__(self):
        if type(self.per_group) is not int or self.per_group < 1:
            raise ValueError('per_group must be a positive integer')
        if type(self.random_seed) is not int or self.random_seed < 0:
            raise ValueError('random_seed must be a nonnegative integer')
        vals=[v for k,v in asdict(self).items() if k not in ('per_group','random_seed') and v is not None]
        if not np.isfinite(vals).all(): raise ValueError('Nonfinite sampling bound')
        if not (0 <= self.x_min_m < self.x_max_m and self.y_min_m < self.y_max_m):
            raise ValueError('Invalid position bounds')
        if self.max_horizontal_distance_m <= 0 or min(self.tank_yaw_limit_deg,self.cap_yaw_limit_deg)<0:
            raise ValueError('Invalid radius or angle bound')
        nearest_y=np.clip(0.,self.y_min_m,self.y_max_m)
        if np.hypot(self.x_min_m,nearest_y)>=self.max_horizontal_distance_m:
            raise ValueError('Sampling rectangle has no area inside radial bound')


class SceneSampler:
    def __init__(self, candidate_path, settings=None):
        self.settings=settings or SamplingSettings()
        self.reference=ArmIKPreview(candidate_path)
        self.candidate_path=Path(candidate_path)
        d=self.reference.data
        self.world_from_base=body_transform(d,'base_link')
        self.mount=body_transform(d,'inspection_tank_mount')
        self.opening=d.body('tank_opening_center').xpos.copy()
        # 提高后的开孔位置不再位于车体高度中点。
        self.center=d.body('tank_body_center').xpos.copy()
        self.base_opening=self.world_from_base[:3,:3].T@(self.opening-self.world_from_base[:3,3])
        self.cap_initial=float(d.joint('cap_twist').qpos[0])

    def samples(self):
        cfg=self.settings;rng=np.random.default_rng(cfg.random_seed)
        height=self.base_opening[2] if cfg.height_m is None else cfg.height_m
        rows=[{'id':0,'group':'baseline','opening_base_m':self.base_opening.tolist(),
               'tank_yaw_deg':0.,'cap_yaw_deg':0.}]
        for group in ('position','tank_yaw','cap_yaw','mixed'):
            for _ in range(cfg.per_group):
                p=self.base_opening.copy()
                if group in ('position','mixed'):
                    for attempt in range(10000):
                        p=np.array([rng.uniform(cfg.x_min_m,cfg.x_max_m),rng.uniform(cfg.y_min_m,cfg.y_max_m),height])
                        if np.linalg.norm(p[:2])<=cfg.max_horizontal_distance_m:break
                    else:raise ValueError('Unable to sample within configured bounds')
                rows.append({'id':len(rows),'group':group,'opening_base_m':p.tolist(),
                             'tank_yaw_deg':float(rng.uniform(-cfg.tank_yaw_limit_deg,cfg.tank_yaw_limit_deg)) if group in ('tank_yaw','mixed') else 0.,
                             'cap_yaw_deg':float(rng.uniform(-cfg.cap_yaw_limit_deg,cfg.cap_yaw_limit_deg)) if group in ('cap_yaw','mixed') else 0.})
        return rows

    def single_sample(self):
        """生成一个独立混合样本，在可行性检查之前固定采样结果。"""
        cfg = self.settings
        # 相同种子复现同一个样本；这里的位置在机械臂 base_link 系中，单位 m。
        rng = np.random.default_rng(cfg.random_seed)
        height = self.base_opening[2] if cfg.height_m is None else cfg.height_m
        # 拒绝的只是超出水平距离边界的位置，不依据 IK 或碰撞结果挑样。
        for _ in range(10000):
            p = [rng.uniform(cfg.x_min_m, cfg.x_max_m),
                 rng.uniform(cfg.y_min_m, cfg.y_max_m), height]
            if np.linalg.norm(p[:2]) <= cfg.max_horizontal_distance_m:
                break
        else:
            raise ValueError('Unable to sample within configured bounds')
        # 两个角度分别是箱体绕竖直轴偏航、盖体绕开口轴转动，单位 deg。
        return {'id': 0, 'group': 'mixed', 'opening_base_m': list(map(float, p)),
                'tank_yaw_deg': float(rng.uniform(-cfg.tank_yaw_limit_deg, cfg.tank_yaw_limit_deg)),
                'cap_yaw_deg': float(rng.uniform(-cfg.cap_yaw_limit_deg, cfg.cap_yaw_limit_deg))}

    def preview(self, sample, ik_settings=None):
        root=ET.parse(DEFAULT_SCENE).getroot()
        root.set('model', f"sample_{sample.get('id', 'custom')}_{sample.get('group', 'custom')}")
        for mesh in root.findall('./asset/mesh'):
            mesh.set('file',str((DEFAULT_SCENE.parent/mesh.get('file')).resolve()))
        # br 将基座系方向转到世界系；delta 是在世界系表达的箱体偏航旋转。
        br=self.world_from_base[:3,:3]
        delta=br@rotation(2,np.deg2rad(sample['tank_yaw_deg']))@br.T
        # 先绕车体几何中心旋转，求旋转后的开孔位置，再算采样期望的世界位置。
        rotated_opening=self.center+delta@(self.opening-self.center)
        desired=self.world_from_base[:3,3]+br@np.array(sample['opening_base_m'])
        # 将安装根也绕车体中心转动，再平移 desired - rotated_opening。
        # 箱体、门和盖共用这棵装配树，因而保持相对装配关系。
        pos=self.center+delta@(self.mount[:3,3]-self.center)+desired-rotated_opening
        r=delta@self.mount[:3,:3]
        body=root.find("./worldbody/body[@name='inspection_tank_mount']")
        body.set('pos',' '.join(map(str,pos)))
        for attr in ('euler','axisangle','xyaxes','zaxis'):body.attrib.pop(attr,None)
        # 工具输出 XYZW，MuJoCo XML 使用 WXYZ；只调整存储顺序。
        q=np.array(quaternion(r))[[3,0,1,2]]
        body.set('quat',' '.join(map(str,q)))
        xml=ET.tostring(root,encoding='unicode')
        # 重新编译派生模型，避免静态几何移动后沿用旧碰撞缓存；不改源场景。
        model=mujoco.MjModel.from_xml_string(xml)
        scene=MujocoCapScene(model=model)
        # 盖体额外绕开口轴转动；这个简化关节只改变姿态，不模拟螺纹运动。
        scene.data.joint('cap_twist').qpos[0]=self.cap_initial+np.deg2rad(sample['cap_yaw_deg'])
        mujoco.mj_forward(model,scene.data)
        p=ArmIKPreview(self.candidate_path,ik_settings,scene=scene)
        p.variation_source_sha256=hashlib.sha256(xml.encode()).hexdigest()
        return p


def classify(report):
    if not report['ik_found']:return 'ik_not_found'
    return 'endpoint_clear' if report['enabled_geometry_interference_free_solution_found'] else 'endpoint_interference'


def save_report(path, report):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    tmp.replace(path)
