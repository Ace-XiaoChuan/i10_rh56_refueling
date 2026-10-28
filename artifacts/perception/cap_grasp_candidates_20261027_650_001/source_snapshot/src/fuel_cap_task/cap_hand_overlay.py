"""将固定手盖关系放到观测盖体上，提取相机系手部网格；仅用于静态几何预览。"""

from dataclasses import dataclass
from copy import deepcopy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .fixed_grasp import FixedGrasp
from .grasp_search import pose_arrays
from .grasp_targets import _rigid_transform, transform
from .mujoco_scene import HandPreviewMapping


@dataclass(frozen=True)
class HandOverlay:
    """坐标与网格顶点均为米制相机系，面索引引用合并后的顶点数组。"""

    camera_from_cap: np.ndarray
    camera_from_hand: np.ndarray
    vertices_camera_m: np.ndarray
    faces: np.ndarray
    geometries: list[dict]
    fingertip_sites_camera_m: dict
    joint_positions_rad: dict
    coupling_residual_max_rad: float


class HandOnlyScene:
    """从已核对的模型提取 RH56，编译时不包含占位环境或盖体真值。

    只继承手部的网格、关节、耦合和驱动映射。目标位置完全由调用方传入的
    实测 camera_from_cap 决定；载体用于静态放置手根，不推进动力学。
    """

    def __init__(self, source_scene_path):
        self.path = Path(source_scene_path).resolve()
        raw = self.path.read_bytes()
        self.source_sha256 = hashlib.sha256(raw).hexdigest()
        source = ET.fromstring(raw)
        hand = source.find(".//body[@name='base']")
        if hand is None:
            raise ValueError("源模型缺少 RH56 手根 base")
        hand = deepcopy(hand)
        # 手根的安装位置由观测抓姿提供，不继承源场景的装配位置或朝向。
        for key in ('pos', 'quat', 'euler', 'axisangle', 'xyaxes', 'zaxis'):
            hand.attrib.pop(key, None)
        bodies = {item.get('name') for item in hand.iter('body')}
        joints = {item.get('name') for item in hand.iter('joint')}
        meshes = {item.get('mesh') for item in hand.iter('geom') if item.get('mesh')}
        root = ET.Element('mujoco', model='rh56_on_measured_point_cloud')
        compiler = deepcopy(source.find('compiler'))
        meshdir = compiler.attrib.pop('meshdir', '')
        root.append(compiler)
        ET.SubElement(root, 'option', gravity='0 0 0')
        for item in source.findall('default') + source.findall('visual'):
            root.append(deepcopy(item))
        assets = ET.SubElement(root, 'asset')
        for item in source.find('asset'):
            if item.tag == 'mesh' and item.get('name') not in meshes:
                continue
            if item.tag not in ('mesh', 'material', 'texture'):
                continue
            item = deepcopy(item)
            if item.get('file'):
                directory = meshdir if item.tag == 'mesh' else ''
                item.set('file', str((self.path.parent / directory / item.get('file')).resolve()))
            assets.append(item)
        world = ET.SubElement(root, 'worldbody')
        carrier = ET.SubElement(world, 'body', name='observed_hand_carrier', mocap='true')
        carrier.append(hand)
        for tag, keys, names in (
                ('actuator', ('joint',), joints),
                ('equality', ('joint1', 'joint2'), joints),
                ('contact', ('body1', 'body2'), bodies)):
            section = ET.SubElement(root, tag)
            original = source.find(tag)
            if original is not None:
                for item in original:
                    if all(item.get(key) in names for key in keys):
                        section.append(deepcopy(item))
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.hand = HandPreviewMapping(self.model)
        self.hand_bodies = {self.model.body(name).id for name in bodies}
        mujoco.mj_forward(self.model, self.data)
        carrier_data = self.data.body('observed_hand_carrier')
        rotation = carrier_data.xmat.reshape(3, 3)
        self.carrier_from_hand_p = rotation.T @ (self.data.body('base').xpos - carrier_data.xpos)
        self.carrier_from_hand_r = rotation.T @ self.data.body('base').xmat.reshape(3, 3)
        self.mocap = int(self.model.body('observed_hand_carrier').mocapid[0])


def extract_visual_meshes(model, data, body_ids):
    """提取指定手部 body 的显示网格，不混入碰撞凸包、传感器或环境几何。

    mesh_vert 是 MuJoCo 编译后的网格坐标；geom_xpos/geom_xmat 已包含编译时
    网格居中与旋转的补偿，二者相乘即可。再次应用 mesh_pos/mesh_quat 会导致
    重复偏移。调用前由调用方执行 mj_forward 更新派生量。
    """
    vertices, faces, records = [], [], []
    offset = 0
    for geom in range(model.ngeom):
        if (int(model.geom_bodyid[geom]) not in body_ids
                or model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError("此显示网格导出器要求手部 visual 使用 mesh，未实现的几何不能静默忽略")
        mesh = int(model.geom_dataid[geom])
        va, vn = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        fa, fn = int(model.mesh_faceadr[mesh]), int(model.mesh_facenum[mesh])
        local = model.mesh_vert[va:va+vn].astype(float)
        rotation = data.geom_xmat[geom].reshape(3, 3)
        camera = local @ rotation.T + data.geom_xpos[geom]
        local_faces = model.mesh_face[fa:fa+fn].astype(np.int32)
        if (not len(camera) or not len(local_faces) or not np.isfinite(camera).all()
                or local_faces.min() < 0 or local_faces.max() >= len(camera)):
            raise ValueError("显示网格顶点或面索引无效")
        vertices.append(camera)
        faces.append(local_faces + offset)
        source = None
        if model.mesh_pathadr[mesh] >= 0:
            source = model.paths[int(model.mesh_pathadr[mesh]):].split(b'\0')[0].decode()
        records.append({"body": model.body(int(model.geom_bodyid[geom])).name,
                        "geom_id": geom, "compiled_mesh": model.mesh(mesh).name,
                        "source_mesh": source, "vertex_offset": offset,
                        "vertex_count": vn, "face_count": fn})
        offset += vn
    if not vertices:
        raise ValueError("未找到手部显示网格")
    return np.vstack(vertices), np.vstack(faces), records


def place_fixed_hand(camera_from_cap, grasp: FixedGrasp, *, scene=None):
    """复用固定抓姿与手部预型，在相机系放置 RH56，返回网格和核对数据。

    camera_from_cap 将盖体系点变换到相机系。手根使用固定抓取位置，手指使用
    grasp.pregrasp_channels；这里的手指预型不改变手根为另一个预抓取位置。
    仅写入预览模型的 mocap/qpos 并执行 mj_forward；不推进动力学或生成轨迹。
    可传入同一模型 scene 复用；输入错误时不改动 scene。
    """
    camera_from_cap = _rigid_transform(camera_from_cap, "camera_from_cap")
    if grasp.cap_from_hand.frame_id != "cap_grip_center":
        raise ValueError("固定抓姿必须在 cap_grip_center 中表达")
    cap_from_hand = transform(*pose_arrays({"position_m": grasp.cap_from_hand.position_m,
                                           "orientation_xyzw": grasp.cap_from_hand.orientation_xyzw}))
    camera_from_hand = camera_from_cap @ cap_from_hand
    scene = scene if scene is not None else HandOnlyScene(grasp.provenance.source_scene_path)
    model, data = scene.model, scene.data
    scene.hand.targets(grasp.pregrasp_channels)

    # 目标描述的是手根 base；扣除显示载体到手根的关系，兼容已有预览载体。
    carrier_rotation = camera_from_hand[:3, :3] @ scene.carrier_from_hand_r.T
    carrier_position = camera_from_hand[:3, 3] - carrier_rotation @ scene.carrier_from_hand_p
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.ascontiguousarray(carrier_rotation).ravel())
    data.mocap_pos[scene.mocap] = carrier_position
    data.mocap_quat[scene.mocap] = quaternion
    scene.hand.apply(data, grasp.pregrasp_channels)
    actual = transform(data.body("base").xpos, data.body("base").xmat.reshape(3, 3))
    if not np.allclose(actual, camera_from_hand, atol=1e-10, rtol=0):
        raise ValueError("手根模型位置与固定抓姿变换不一致")

    vertices, faces, records = extract_visual_meshes(model, data, scene.hand_bodies)
    fingertips = {name: data.site(name).xpos.copy() for name in
                  ("right_thumb_tip", "right_index_tip", "right_middle_tip", "right_ring_tip", "right_pinky_tip")}
    positions = {model.joint(j).name: float(data.qpos[model.jnt_qposadr[j]]) for j in scene.hand.joint_ids}
    for joint in scene.hand.joint_ids:
        value = data.qpos[model.jnt_qposadr[joint]]
        if not model.jnt_range[joint, 0] - 1e-9 <= value <= model.jnt_range[joint, 1] + 1e-9:
            raise ValueError("手部预型超出模型关节范围")
    residuals = [abs(data.qpos[model.jnt_qposadr[c.follower_id]] - c.offset
                     - c.slope * data.qpos[model.jnt_qposadr[c.driver_id]]) for c in scene.hand.couplings]
    return HandOverlay(camera_from_cap, camera_from_hand, vertices, faces, records,
                       fingertips, positions, float(max(residuals, default=0)))


def write_hand_ply(path, vertices_camera_m, faces, *, length_unit="mm"):
    """输出带颜色的二进制 PLY，用于 CloudCompare；双精度坐标避免额外舍入。"""
    if length_unit not in ("m", "mm"):
        raise ValueError("PLY 长度单位必须显式为 m 或 mm")
    vertices = np.asarray(vertices_camera_m, dtype=float)
    triangles = np.asarray(faces)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices)
            or not np.isfinite(vertices).all() or triangles.ndim != 2 or triangles.shape[1] != 3
            or not len(triangles) or not np.issubdtype(triangles.dtype, np.integer)
            or triangles.min() < 0 or triangles.max() >= len(vertices)):
        raise ValueError("PLY 顶点或面索引无效")
    header = ("ply\nformat binary_little_endian 1.0\ncomment coordinate_frame camera\n"
              f"comment length_unit {length_unit}\nelement vertex {len(vertices)}\n"
              "property double x\nproperty double y\nproperty double z\n"
              "property float nx\nproperty float ny\nproperty float nz\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              f"element face {len(triangles)}\nproperty list uchar int vertex_indices\nend_header\n")
    # 按面面积加权计算顶点法向，便于 CloudCompare 直接显示手部表面。
    triangle_points = vertices[triangles]
    area_normals = np.cross(triangle_points[:,1]-triangle_points[:,0],
                            triangle_points[:,2]-triangle_points[:,0])
    normals = np.zeros_like(vertices)
    for corner in range(3):
        np.add.at(normals, triangles[:,corner], area_normals)
    norms = np.linalg.norm(normals, axis=1)
    np.divide(normals, norms[:,None], out=normals, where=norms[:,None]>1e-20)
    points = np.empty(len(vertices), dtype=[('xyz', '<f8', (3,)), ('normal', '<f4', (3,)),
                                           ('rgb', 'u1', (3,))])
    points['xyz'] = vertices * (1000. if length_unit == "mm" else 1.)
    points['normal'] = normals
    points['rgb'] = [175, 184, 197]
    cells = np.empty(len(triangles), dtype=[('count', 'u1'), ('indices', '<i4', (3,))])
    cells['count'], cells['indices'] = 3, triangles
    with open(path, 'xb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.tobytes())
        stream.write(cells.tobytes())
