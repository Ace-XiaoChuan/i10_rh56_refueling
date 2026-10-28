"""RH56 可微手适配与六驱动优化；依赖独立 SpaHybGen 环境。"""

import json
from pathlib import Path
import tempfile
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import torch
import spahybgen.optimization as upstream_optimization
from spahybgen.handmodel import HandModel
from spahybgen.pipeline.grasp_optimization import GraspOptimization
from spahybgen.utils.utils_trans_torch import robust_R6d2R9d

from fuel_cap_task.rh56_drive_map import RH56DriveMap


class RH56HandModel(HandModel):
    """沿用上游网格/接触采样，把独立变量改为六个模型驱动角。"""

    def __init__(self, model_dir, batch_size=1, device="cpu"):
        model_dir = Path(model_dir)
        self.metadata = json.loads((model_dir / "model.json").read_text())
        self.drives = RH56DriveMap.from_dict(self.metadata["mapping"])
        # 旧 URDF 解析器把文件读为 Unicode，不能接受 XML 编码声明。
        # 只规范化临时副本的序列化形式，源文件及运动学语义保持不变。
        with tempfile.TemporaryDirectory(prefix="rh56-urdf-") as temporary:
            urdf = Path(temporary) / "hand.urdf"
            urdf.write_text(ET.tostring(ET.parse(model_dir / "hand.urdf").getroot(), encoding="unicode"))
            super().__init__(
                robot_name="rh56dfx_right", urdf_path=str(urdf),
                mesh_path=str(model_dir), contact_path=str(model_dir / "contacts.json"),
                actuate_dofs=6, batch_size=batch_size, device=torch.device(device),
                orientation_type="R6d", hand_scale=1.0,
                hand_approaching_matrix=self.metadata["hand_approaching_matrix"],
                hand_position_offset=self.metadata["hand_position_offset_m"],
            )
        # 从动角不能成为自由变量；显式对齐运动学库实际返回的关节顺序。
        chain_names = self.robot.get_joint_parameter_names()
        if set(chain_names) != set(self.drives.joint_names):
            raise ValueError("kinematic chain joint names differ from drive mapping")
        self.chain_rows = [self.drives.joint_names.index(name) for name in chain_names]
        self.drive_matrix = torch.tensor(self.drives.matrix, dtype=torch.float32, device=self.device)
        self.drive_offset = torch.tensor(self.drives.offset, dtype=torch.float32, device=self.device)
        by_name = {joint.name: joint for joint in self.robot_full.joints}
        self.joints = [by_name[name] for name in self.drives.driver_names]
        self._joints_q_lower, self._joints_q_upper = self.drives.lower, self.drives.upper
        self.joints_q_lower = torch.tensor(self.drives.lower, dtype=torch.float32,
                                           device=self.device).repeat(batch_size, 1)
        self.joints_q_upper = torch.tensor(self.drives.upper, dtype=torch.float32,
                                           device=self.device).repeat(batch_size, 1)
        self.joints_q_mid = ((self.drives.lower + self.drives.upper) / 2).tolist()
        self.joints_q_var = (((self.drives.upper - self.drives.lower) / 2) ** 2).tolist()
        self.initial_joints = self.joints_q_lower.clone()
        # 两套 trimesh 版本必须读取到同一顶点顺序，不能仅核对 OBJ 文件哈希。
        for region in self.metadata["contact_regions"]:
            vertices = self.mesh_verts[region["link"]][region["vertex_indices"]]
            if not np.allclose(vertices, region["vertices_link_m"], atol=1e-8, rtol=0):
                raise ValueError("contact vertex ordering changed across environments")

    def expand_drivers(self, drivers):
        """保持计算图：(...,6) rad → (...,12) rad，顺序按元数据。"""
        return drivers @ self.drive_matrix.T + self.drive_offset

    def update_kinematics(self, q):
        """输入 B×15：场景中的 xyz、R6d、六驱动；输出手根到各 link。"""
        if q.shape != (self.batch_size, 15):
            raise ValueError("RH56 pose must have shape (batch,15)")
        self.global_translation = q[:, :3]
        self.global_rotation = robust_R6d2R9d(q[:, 3:9])
        full = self.expand_drivers(q[:, 9:])
        self.current_status = self.robot.forward_kinematics(full[:, self.chain_rows])
        self.current_approaching_matrices = self.global_rotation @ self.approaching_matrices_tc.inverse()
        return self.current_status


class RH56Optimization(GraspOptimization):
    """复用上游空间损失和迭代流程，加入驱动硬投影及末态重算。"""

    def __init__(self, model_dir, batch=16, device="cpu"):
        hand = RH56HandModel(model_dir, batch_size=batch, device=device)

        class Loader:
            """在本离线构造作用域内替换手加载器，不修改上游 checkout。"""

            @staticmethod
            def load_hand_from_json(name, count, target_device, hand_scale):
                if name != hand.robot_name or count != batch or hand_scale != 1:
                    raise ValueError("unexpected hand factory arguments")
                return hand

        # 上游没有注入手实例的参数；构造结束立即恢复模块绑定。
        with patch.object(upstream_optimization, "HandModel", Loader):
            super().__init__(robot_name=hand.robot_name, batch=batch, device=device,
                             penetration_mode="contact_penetration")

    def project_drivers(self):
        """每次评价前及 Adam 更新后投影；不裁剪 xyz 或旋转参数。"""
        with torch.no_grad():
            self.q_current[:, 9:].copy_(torch.maximum(
                self.q_joint_lower, torch.minimum(self.q_current[:, 9:], self.q_joint_upper)))

    def reset_scene(self, scene_infer_map, cam_appr_matrix):
        """沿用原初始化及特征筛选，并明确保证驱动可行。"""
        super().reset_scene(scene_infer_map, cam_appr_matrix)
        self.project_drivers()

    def loss_custom_kinematics(self):
        """耦合已经硬编码在展开中，不能再用独立从动角的软惩罚。"""
        return torch.zeros(self.batch, device=self.device)

    @staticmethod
    def grouped_losses(losses):
        """按上游顺序合并七组损失，保持其归一化权重。"""
        return [losses["RH"], losses["WH"], losses["PN"],
                losses["FQH"] + losses["QH"], losses["JR"] + losses["CK"],
                losses["FC"] + losses["LD"], losses["AB"]]

    def step(self, penetration_check):
        """基于上游 step 的扰动与 Adam；区别是六驱动硬可行域。"""
        if torch.rand(1) > torch.tensor(1 + self.global_step / 20).sigmoid():
            with torch.no_grad():
                noise = torch.normal(0.0, self.learning_rate, size=self.q_current.shape).to(self.device)
                self.q_current.add_(noise)
        self.project_drivers()
        self.optimizer.zero_grad()
        self.handmodel.update_kinematics(self.q_current)
        self.losses = self.compute_spacial_loss(penetration_check)
        groups = self.grouped_losses(self.losses)
        self.losses["loss_all"] = sum(groups)
        scaled = self.loss_normalizer.update_and_scale(groups, [5, 5, 10, 1, 1, 1, 1])
        loss = torch.stack(scaled).sum(dim=0)
        if not torch.isfinite(loss).all():
            raise ValueError("non-finite optimization objective")
        loss.mean().backward()
        if not torch.isfinite(self.q_current.grad).all():
            raise ValueError("non-finite optimization gradient")
        self.optimizer.step()
        self.project_drivers()
        self.losses["loss_opti"] = loss
        self.global_step += 1

    def evaluate_final(self):
        """末态重新计算，冻结运行均值；不再更新 Adam 或归一化器。"""
        previous_mode = self.penetration_mode
        self.penetration_mode = "surface_penetration"
        try:
            with torch.no_grad():
                self.handmodel.update_kinematics(self.q_current)
                losses = self.compute_spacial_loss(True)
                groups = self.grouped_losses(losses)
                losses["loss_all"] = sum(groups)
                scaled = [x / (mean + self.loss_normalizer.eps) * weight
                          for x, mean, weight in zip(groups, self.loss_normalizer.mean,
                                                    [5, 5, 10, 1, 1, 1, 1])]
                losses["loss_opti"] = torch.stack(scaled).sum(dim=0)
                return {key: value.cpu().numpy() for key, value in losses.items()}
        finally:
            self.penetration_mode = previous_mode


def audit_hand_model(model_dir):
    """独立 FK、网格法向及有限差分核查；不是接触动力学验收。"""
    reference = np.load(Path(model_dir) / "fk_reference.npz")
    drivers = reference["drivers"]
    hand = RH56HandModel(model_dir, batch_size=len(drivers))
    q = torch.zeros(len(drivers), 15)
    q[:, 3], q[:, 7] = 1, 1
    q[:, 9:] = torch.tensor(drivers, dtype=torch.float32)
    hand.update_kinematics(q)
    actual = np.stack([hand.current_status[name].get_matrix().detach().numpy()
                       for name in hand.metadata["link_names"]], axis=1)
    expected = reference["base_from_links"]
    position_error = float(np.max(np.linalg.norm(actual[..., :3, 3] - expected[..., :3, 3], axis=-1)))
    rotation_error = float(np.max(np.abs(actual[..., :3, :3] - expected[..., :3, :3])))
    if position_error > 2e-6 or rotation_error > 2e-6:
        raise ValueError("differentiable FK disagrees with MuJoCo reference")

    # 根据保存的真实三角形独立构造采样点/法向，避免只检查数量与长度。
    weights = hand._sample_contact_points.detach().numpy()
    point_errors, normal_errors = [], []
    points, normals = hand.sample_contact_points_and_normal()
    records = {name: [] for name in hand.contact_point_dict}
    for region in hand.metadata["contact_regions"]:
        records[region["link"]].append(region)
    for index, key in enumerate(hand.contact_point_basis):
        link, number = key.split("+")
        region = records[link][int(number)]
        basis = np.array(region["vertices_link_m"])
        local = weights @ basis
        transform = expected[:, hand.metadata["link_names"].index(link)]
        correct_points = np.einsum("bij,pj->bpi", transform[:, :3, :3], local) + transform[:, None, :3, 3]
        correct_normal = np.einsum("bij,j->bi", transform[:, :3, :3], region["normal_link"])
        selection = slice(index * len(weights), (index + 1) * len(weights))
        point_errors.append(np.max(np.linalg.norm(points[:, selection].detach().numpy() - correct_points, axis=-1)))
        normal_errors.append(np.max(np.abs(normals[:, selection].detach().numpy() - correct_normal[:, None])))
    if max(point_errors) > 2e-6 or max(normal_errors) > 2e-6:
        raise ValueError("sampled contacts disagree with saved surface triangles")

    # 六驱动均必须沿耦合 FK 影响接触点；用数值差分检查反传而非仅看 requires_grad。
    weights_xyz = torch.linspace(.2, 1.0, points.numel()).reshape_as(points)
    q.requires_grad_(True)
    def objective(value):
        p, _ = hand.sample_contact_points_and_normal(value)
        return (p * weights_xyz).sum()
    analytic = torch.autograd.grad(objective(q), q)[0][:, 9:].detach().numpy()
    numeric = np.empty_like(analytic)
    delta = 1e-3
    for column in range(6):
        plus, minus = q.detach().clone(), q.detach().clone()
        plus[:, 9 + column] += delta
        minus[:, 9 + column] -= delta
        # 逐 batch 差分，避免 float32 总和抵消小的指尖变化。
        pp, _ = hand.sample_contact_points_and_normal(plus)
        pm, _ = hand.sample_contact_points_and_normal(minus)
        numeric[:, column] = (((pp - pm) * weights_xyz).sum(dim=(1, 2)) / (2 * delta)).detach().numpy()
    gradient_error = float(np.max(np.abs(analytic - numeric)))
    if gradient_error > 2e-4 or not np.all(np.max(np.abs(analytic), axis=0) > 1e-5):
        raise ValueError("drive/contact gradient failed finite-difference check")
    return {"reference_poses": len(drivers), "links": len(hand.metadata["link_names"]),
            "position_max_error_m": position_error, "rotation_max_element_error": rotation_error,
            "contact_point_max_error_m": float(max(point_errors)),
            "contact_normal_max_element_error": float(max(normal_errors)),
            "gradient_max_error": gradient_error, "gradient_all_six_drivers_nonzero": True,
            "contact_regions": len(hand.contact_point_basis), "sampled_contact_points": hand.contact_points_num}
