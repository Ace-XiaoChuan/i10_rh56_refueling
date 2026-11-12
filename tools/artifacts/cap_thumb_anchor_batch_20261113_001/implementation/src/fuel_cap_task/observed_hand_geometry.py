"""独立 RH56 碰撞与实测表面检查，不读取盖体或车体真值。"""

import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial import ConvexHull


def primitive_inside_depth(kind, size, points, planes=None):
    """凸碰撞体内侧深度，单位米；只有正值表示进入实体内部。"""
    points = np.asarray(points, dtype=float)
    size = np.asarray(size, dtype=float)
    if kind == int(mujoco.mjtGeom.mjGEOM_BOX):
        return np.min(size[:3] - np.abs(points), axis=1)
    if kind == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return size[0] - np.linalg.norm(points, axis=1)
    if kind == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        return np.minimum(size[0] - np.linalg.norm(points[:, :2], axis=1), size[1] - np.abs(points[:, 2]))
    if kind == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        nearest = points.copy()
        nearest[:, 2] -= np.clip(points[:, 2], -size[1], size[1])
        return size[0] - np.linalg.norm(nearest, axis=1)
    if kind == int(mujoco.mjtGeom.mjGEOM_MESH) and planes is not None:
        return -np.max(points @ planes[:, :3].T + planes[:, 3], axis=1)
    raise ValueError("未支持的手部碰撞形状：" + str(kind))


class ObservedHandGeometry:
    """只提取现有 MJCF 的 base 手子树，保留原碰撞原语和手内排除对。"""

    def __init__(self, scene_path):
        self.scene_path = Path(scene_path).resolve()
        source = ET.parse(self.scene_path).getroot()
        root = ET.Element("mujoco", model="observed_rh56_hand_only")
        compiler = copy.deepcopy(source.find("compiler"))
        meshdir = compiler.attrib.pop("meshdir", "")
        root.append(compiler)
        for default in source.findall("default"):
            default = copy.deepcopy(default)
            # 独立手由关节角直接放置，没有驱动器；省去无用的驱动器默认值，
            # 避免新MuJoCo的dampratio等属性阻止旧优化环境读取同一碰撞几何。
            actuator_tags = {"general", "motor", "position", "velocity", "intvelocity",
                             "damper", "cylinder", "muscle", "adhesion"}
            for parent in default.iter():
                for child in list(parent):
                    if child.tag in actuator_tags:
                        parent.remove(child)
            root.append(default)
        hand = copy.deepcopy(source.find(".//body[@name='base']"))
        for key in ("pos", "quat", "euler", "axisangle", "xyaxes", "zaxis"):
            hand.attrib.pop(key, None)
        names = {body.get("name") for body in hand.iter("body")}
        for body in hand.iter("body"):
            for geom in list(body.findall("geom")):
                if geom.get("class") == "visual":
                    body.remove(geom)
        used_meshes = {geom.get("mesh") for geom in hand.iter("geom") if geom.get("mesh")}
        asset = ET.SubElement(root, "asset")
        self.mesh_paths = []
        for mesh in source.findall("./asset/mesh"):
            if mesh.get("name") in used_meshes:
                mesh = copy.deepcopy(mesh)
                if mesh.get("file"):
                    path = (self.scene_path.parent / meshdir / mesh.get("file")).resolve()
                    self.mesh_paths.append(path)
                    mesh.set("file", str(path))
                asset.append(mesh)
        ET.SubElement(root, "worldbody").append(hand)
        contact = ET.SubElement(root, "contact")
        self.excluded_body_pairs = []
        for pair in source.findall("./contact/exclude"):
            if pair.get("body1") in names and pair.get("body2") in names:
                contact.append(copy.deepcopy(pair))
                self.excluded_body_pairs.append([pair.get("body1"), pair.get("body2")])
        self.xml_string = ET.tostring(root, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_string(self.xml_string)
        self.data = mujoco.MjData(self.model)
        self.geoms, self.shapes = [], {}
        for geom in range(self.model.ngeom):
            if not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]):
                continue
            self.geoms.append(geom)
            kind, planes = int(self.model.geom_type[geom]), None
            if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
                mesh = int(self.model.geom_dataid[geom])
                start, count = int(self.model.mesh_vertadr[mesh]), int(self.model.mesh_vertnum[mesh])
                vertices = self.model.mesh_vert[start:start + count].astype(float)
                planes = ConvexHull(vertices).equations
                low, high = vertices.min(axis=0), vertices.max(axis=0)
            else:
                size = self.model.geom_size[geom]
                if kind == int(mujoco.mjtGeom.mjGEOM_BOX):
                    extent = size.copy()
                elif kind == int(mujoco.mjtGeom.mjGEOM_SPHERE):
                    extent = np.repeat(size[0], 3)
                elif kind in (int(mujoco.mjtGeom.mjGEOM_CYLINDER), int(mujoco.mjtGeom.mjGEOM_CAPSULE)):
                    extent = np.array([size[0], size[0], size[1] + (size[0] if kind == int(mujoco.mjtGeom.mjGEOM_CAPSULE) else 0)])
                else:
                    raise ValueError("未支持的手部碰撞形状：" + str(kind))
                low, high = -extent, extent
            self.shapes[geom] = (kind, planes, low, high)

    def place(self, joint_names, angles):
        """设置十二关节并更新几何，不积分物理，不连接硬件。"""
        for name, angle in zip(joint_names, angles):
            self.data.joint(name).qpos[0] = angle
        mujoco.mj_forward(self.model, self.data)

    def self_penetrations(self, tolerance):
        """沿用原碰撞排除对，报告超过既定容差的非允许自碰。"""
        return [dict(geoms=[int(v) for v in contact.geom],
                     bodies=[self.model.body(int(self.model.geom_bodyid[int(v)])).name for v in contact.geom],
                     distance_m=float(contact.dist))
                for contact in self.data.contact if contact.dist < -tolerance]

    def surface_penetrations(self, points_hand_m, target_mask, source_indices, tolerance):
        """逐个手部实体测试原始实测点，分别保留目标与周边表面的侵入证据。"""
        points = np.asarray(points_hand_m)
        occupied, rows = np.zeros(len(points), dtype=bool), []
        for geom in self.geoms:
            rotation = self.data.geom_xmat[geom].reshape(3, 3)
            local = (points - self.data.geom_xpos[geom]) @ rotation
            kind, planes, low, high = self.shapes[geom]
            possible = np.flatnonzero(np.all((local >= low) & (local <= high), axis=1))
            if not len(possible):
                continue
            depth = primitive_inside_depth(kind, self.model.geom_size[geom], local[possible], planes)
            deep = depth > tolerance
            inside = possible[deep]
            if not len(inside):
                continue
            occupied[inside] = True
            order = np.argsort(depth[deep])[::-1][:12]
            rows.append(dict(geom_id=geom, body=self.model.body(int(self.model.geom_bodyid[geom])).name,
                target_points=int(np.asarray(target_mask)[inside].sum()),
                environment_points=int((~np.asarray(target_mask)[inside]).sum()),
                maximum_inside_depth_m=float(depth.max()),
                evidence_source_indices=np.asarray(source_indices)[inside[order]].tolist(),
                evidence_inside_depth_m=depth[deep][order].tolist()))
        return rows, occupied
