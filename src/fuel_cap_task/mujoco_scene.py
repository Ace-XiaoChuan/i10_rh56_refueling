"""Current MJCF scene, provisional hand mapping and ground-truth observation.

This module only sets kinematic poses and calls mj_forward. It is not an Executor.
Mapping conventions and validity are recorded in doc/assembly_contract.md.
"""

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from .interfaces import CapEstimate, Pose, TaskParameters

DEFAULT_SCENE = Path(__file__).resolve().parents[2] / "assets/scenes/i10_rh56_fuel_cap_preview.xml"
ACTUATORS = ("pinky", "ring", "middle", "index", "thumb_proximal", "thumb_yaw")
ARM_JOINTS = ("shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint")
# Cap model's +Y is along the beam. Columns are task axes in the model frame.
CAP_MODEL_FROM_TASK = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


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
    """Linear, uncalibrated mapping constrained by this model's joint limits."""

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
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model, self.data = model, data
        self.grip_site = model.site("cap_grip_center").id

    async def estimate(self) -> CapEstimate:
        """Read the current pose in world, using simulation seconds as timestamp.

        Caller holds the viewer lock if the same data is shared with a viewer.
        No physical state is advanced and no control is sent.
        """
        mujoco.mj_forward(self.model, self.data)
        rotation = self.data.site_xmat[self.grip_site].reshape(3, 3) @ CAP_MODEL_FROM_TASK
        wxyz = np.empty(4)
        mujoco.mju_mat2Quat(wxyz, rotation.ravel())
        return CapEstimate(
            pose=Pose("world", tuple(self.data.site_xpos[self.grip_site].tolist()),
                      tuple(wxyz[[1, 2, 3, 0]].tolist())),
            timestamp_s=float(self.data.time), source="simulation_ground_truth",
            valid=True, timestamp_clock="simulation",
        )


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
