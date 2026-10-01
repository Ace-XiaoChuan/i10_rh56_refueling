"""Bounded scene variations around the current candidate; endpoint checks only."""
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
        # The raised opening is no longer the body-height midpoint.
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

    def preview(self, sample, ik_settings=None):
        root=ET.parse(DEFAULT_SCENE).getroot()
        root.set('model', f"sample_{sample.get('id', 'custom')}_{sample.get('group', 'custom')}")
        for mesh in root.findall('./asset/mesh'):
            mesh.set('file',str((DEFAULT_SCENE.parent/mesh.get('file')).resolve()))
        br=self.world_from_base[:3,:3]
        delta=br@rotation(2,np.deg2rad(sample['tank_yaw_deg']))@br.T
        rotated_opening=self.center+delta@(self.opening-self.center)
        desired=self.world_from_base[:3,3]+br@np.array(sample['opening_base_m'])
        pos=self.center+delta@(self.mount[:3,3]-self.center)+desired-rotated_opening
        r=delta@self.mount[:3,:3]
        body=root.find("./worldbody/body[@name='inspection_tank_mount']")
        body.set('pos',' '.join(map(str,pos)))
        for attr in ('euler','axisangle','xyaxes','zaxis'):body.attrib.pop(attr,None)
        q=np.array(quaternion(r))[[3,0,1,2]]
        body.set('quat',' '.join(map(str,q)))
        xml=ET.tostring(root,encoding='unicode')
        model=mujoco.MjModel.from_xml_string(xml)
        scene=MujocoCapScene(model=model)
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
