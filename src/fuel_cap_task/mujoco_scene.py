"""当前 MJCF 场景、手部预览映射与模型位姿读取。

本模块设置预览姿态、调用 mj_forward 更新派生量，不推进时间或执行轨迹。
坐标及映射约定见 doc/assembly_contract.md。
"""

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from .interfaces import CapEstimate, Pose, TaskParameters
from .validation import validate_estimate

DEFAULT_SCENE = Path(__file__).resolve().parents[2] / "assets/scenes/i10_rh56_fuel_cap_preview.xml"
ACTUATORS = ("pinky", "ring", "middle", "index", "thumb_proximal", "thumb_yaw")
ARM_JOINTS = ("shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint")


@dataclass(frozen=True)
class ChannelMapping:
    actuator: str
    joint: str
    actuator_id: int
    joint_id: int
    q_open: float
    q_closed: float


@dataclass(frozen=True)
class Coupling:
    follower_id: int
    driver_id: int
    offset: float
    slope: float


class HandPreviewMapping:
    """受当前模型关节限位约束的线性预览映射，尚未经实物标定。"""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        actuator_ids = [model.actuator(name).id for name in ACTUATORS]
        drivers = [int(model.actuator_trnid[a, 0]) for a in actuator_ids]
        self.couplings: list[Coupling] = []
        for eq in range(model.neq):
            if model.eq_type[eq] != mujoco.mjtEq.mjEQ_JOINT or model.eq_obj2id[eq] not in drivers:
                continue
            follower, driver = int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])
            coefficients = model.eq_data[eq, :5]
            if not model.eq_active0[eq] or np.any(coefficients[2:] != 0) or coefficients[1] == 0:
                raise ValueError("hand preview requires active linear joint couplings")
            # y - y0 = a0 + a1 * (x - x0).
            offset = (model.qpos0[model.jnt_qposadr[follower]] + coefficients[0]
                      - coefficients[1] * model.qpos0[model.jnt_qposadr[driver]])
            self.couplings.append(Coupling(follower, driver, float(offset), float(coefficients[1])))

        self.channels: list[ChannelMapping] = []
        for name, actuator, joint in zip(ACTUATORS, actuator_ids, drivers):
            lower = max(model.actuator_ctrlrange[actuator, 0], model.jnt_range[joint, 0])
            upper = min(model.actuator_ctrlrange[actuator, 1], model.jnt_range[joint, 1])
            for coupling in self.couplings:
                if coupling.driver_id == joint:
                    limits = sorted((model.jnt_range[coupling.follower_id] - coupling.offset) / coupling.slope)
                    lower, upper = max(lower, limits[0]), min(upper, limits[1])
            if lower >= upper:
                raise ValueError(f"{name}: no usable range after hand coupling limits")
            self.channels.append(ChannelMapping(name, model.joint(joint).name, actuator, joint,
                                                float(lower), float(upper)))
        self.joint_ids = drivers + [c.follower_id for c in self.couplings]

    def targets(self, channels: Sequence[int]) -> np.ndarray:
        values = np.asarray(channels, dtype=float)
        if values.shape != (6,) or not np.all(np.isfinite(values) & (values >= 0) & (values <= 1000)):
            raise ValueError("preview requires six channel values in 0..1000; -1 hold is not supported")
        opening = np.array([c.q_open for c in self.channels])
        closing = np.array([c.q_closed for c in self.channels])
        return opening + (1 - values / 1000) * (closing - opening)

    def apply(self, data: mujoco.MjData, channels: Sequence[int]) -> None:
        """Set driver and follower poses without altering arm, cap or door joints."""
        for mapping, target in zip(self.channels, self.targets(channels)):
            data.qpos[self.model.jnt_qposadr[mapping.joint_id]] = target
            data.ctrl[mapping.actuator_id] = target
        for coupling in self.couplings:
            driver = data.qpos[self.model.jnt_qposadr[coupling.driver_id]]
            data.qpos[self.model.jnt_qposadr[coupling.follower_id]] = coupling.offset + coupling.slope * driver
        data.qvel[self.model.jnt_dofadr[self.joint_ids]] = 0
        mujoco.mj_forward(self.model, data)


class MujocoCapEstimator:
    """从同一份模型与状态读取盖体，输出世界系中的仿真真值快照。"""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        """绑定场景与握梁中心 site；不创建第二份仿真状态。"""
        self.model, self.data = model, data
        self.grip_site = model.site("cap_grip_center").id

    def read(self) -> CapEstimate:
        """同步读取当前位姿，更新运动学后复制数据，不推进时间或发送控制。

        与显示或执行线程共享 data 时，调用方须在整个读取过程持有同一把锁。
        状态、时间或派生位姿无效时返回 valid=False，未知位姿用 NaN 标记，
        消费方须先校验有效性；不会用旧位姿或单位姿态代替失效观测。
        """
        # 1. 采样时间来自同一份 data；先拒绝异常状态，避免向 MuJoCo 传入 NaN。
        timestamp = float(self.data.time)
        state_valid = bool(np.isfinite(self.data.qpos).all()
                           and np.isfinite(timestamp) and timestamp >= 0)
        position = np.full(3, np.nan)
        rotation = np.full((3, 3), np.nan)
        if state_valid:
            # 2. 更新 qpos 对应的派生坐标；mj_forward 不推进仿真时间。
            mujoco.mj_forward(self.model, self.data)
            position = self.data.site_xpos[self.grip_site].copy()
            # 模型的 cap_grip_center 已统一为 X 沿握梁、Z 朝外，直接读取世界朝向。
            rotation = self.data.site_xmat[self.grip_site].reshape(3, 3).copy()

        # 3. 只有有效旋转才转换四元数，防止转换函数掩盖非法矩阵。
        valid = bool(state_valid and np.isfinite(position).all() and np.isfinite(rotation).all()
                     and np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-6)
                     and abs(np.linalg.det(rotation) - 1.0) <= 1e-6)
        xyzw = np.full(4, np.nan)
        if valid:
            wxyz = np.empty(4)
            mujoco.mju_mat2Quat(wxyz, rotation.ravel())
            xyzw = wxyz[[1, 2, 3, 0]]  # MuJoCo 使用 WXYZ，项目接口使用 XYZW。
            valid = bool(np.isfinite(xyzw).all()
                         and abs(np.linalg.norm(xyzw) - 1.0) <= 1e-6)

        # 4. 复制为不可变元组，后续模型运动不会改变已经返回的观测。
        return CapEstimate(
            pose=Pose("world", tuple(position.tolist()), tuple(xyzw.tolist())),
            timestamp_s=timestamp, source="simulation_ground_truth",
            valid=valid, timestamp_clock="simulation",
        )

    async def estimate(self) -> CapEstimate:
        """供异步任务层使用；与同步入口共用同一读取逻辑。"""
        return self.read()


class MujocoCapScene:
    def __init__(self, path: Path = DEFAULT_SCENE, *, model=None):
        self.model = model if model is not None else mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.hand = HandPreviewMapping(self.model)
        self.estimator = MujocoCapEstimator(self.model, self.data)
        # Existing view_mjcf.py display posture; not an accepted task start pose.
        self.data.joint("upperArm_joint").qpos[0] = -0.5
        self.data.joint("foreArm_joint").qpos[0] = 1.2
        self.hand.apply(self.data, TaskParameters().grasp_channels)

    async def report(self, channels: Sequence[int]) -> dict:
        estimate = await self.estimator.estimate()
        validate_estimate(estimate)
        quat = np.array(estimate.pose.orientation_xyzw)[[3, 0, 1, 2]]
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, quat)
        rotation = rotation.reshape(3, 3)
        equality = self.data.efc_pos[self.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
        thumb = self.data.site("right_thumb_tip").xpos
        index = self.data.site("right_index_tip").xpos
        joints = {self.model.joint(j).name: float(self.data.qpos[self.model.jnt_qposadr[j]])
                  for j in self.hand.joint_ids}
        return {
            "mode": "kinematic_preview_no_physics", "mapping_source": "model_ranges_uncalibrated",
            "model": {"joints": self.model.njnt, "actuators": self.model.nu,
                      "arm_joints": list(ARM_JOINTS), "cap_joint": "cap_twist", "door_joint": "fuel_door_hinge"},
            "requested_channels": list(channels), "channel_mapping": [asdict(c) for c in self.hand.channels],
            "joint_positions_rad": joints, "cap_estimate": asdict(estimate),
            "cap_axis_world": rotation[:, 2].tolist(), "grip_axis_world": rotation[:, 0].tolist(),
            "coupling_residual_max_rad": float(np.max(np.abs(equality))) if equality.size else 0.0,
            "thumb_index_tip_site_distance_m": float(np.linalg.norm(thumb - index)),
            "tip_distance_note": "site-center distance only; not pad clearance or grasp validation",
        }
