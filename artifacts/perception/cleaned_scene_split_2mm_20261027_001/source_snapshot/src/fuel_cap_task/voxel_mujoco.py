"""将占据体素逐格写成 MuJoCo box，保留凹槽及未观测的格子。"""

import xml.etree.ElementTree as ET

import numpy as np


CUBE_CORNERS = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,0],
                         [0,0,1], [1,0,1], [1,1,1], [0,1,1]], dtype=np.int64)
CUBE_QUADS = np.array([[0,3,2,1], [4,5,6,7], [0,1,5,4],
                       [1,2,6,5], [2,3,7,6], [3,0,4,7]])
FACE_NORMALS = np.array([[0,0,-1], [0,0,1], [0,-1,0],
                         [1,0,0], [0,1,0], [-1,0,0]], dtype=np.int64)


def exposed_voxel_faces(indices_xyz):
    """返回占据格与非占据格之间的完整四边形，内部公共面不参与显示。

    三个返回数组逐面对应：所属体素行号、方向编号、四个顶点的整数网格坐标。
    不合并不同格子的外露面，不平滑、不补洞；坐标由调用方按原点和边长换算。
    """
    indices = np.asarray(indices_xyz, dtype=np.int64)
    if indices.ndim != 2 or indices.shape[1] != 3 or not len(indices):
        raise ValueError("体素格号须为非空 N×3 数组")
    dtype = np.dtype([("x", "i8"), ("y", "i8"), ("z", "i8")])
    keys = np.ascontiguousarray(indices).view(dtype).reshape(-1)
    rows, directions, corners = [], [], []
    for direction, normal in enumerate(FACE_NORMALS):
        neighbors = np.ascontiguousarray(indices+normal).view(dtype).reshape(-1)
        selected = np.flatnonzero(~np.isin(neighbors, keys))
        rows.append(selected)
        directions.append(np.full(len(selected), direction, dtype=np.int8))
        corners.append(indices[selected,None,:]+CUBE_CORNERS[CUBE_QUADS[direction]])
    return np.concatenate(rows), np.concatenate(directions), np.concatenate(corners)


def canonical_triangles(triangles):
    """按顶点坐标和三角形排序，独立于 OBJ 或 MuJoCo 编译后的顶点编号。"""
    triangles = np.asarray(triangles, dtype=np.int64)
    order = np.lexsort((triangles[:,:,2], triangles[:,:,1], triangles[:,:,0]), axis=1)
    keys = np.take_along_axis(triangles, order[:,:,None], axis=1).reshape(-1,9)
    return keys[np.lexsort(keys.T[::-1])]


def numbers(values):
    """用足够的小数位保存米制几何，避免 XML 舍入改变 2 mm 格子边界。"""
    return " ".join(format(float(value), ".15g") for value in values)


def camera_views(centers_camera_m, focus_camera_m):
    """返回完整场景、盖体区域及格子细节取景；SDK 的图像上方为相机 -Y。"""
    centers = np.asarray(centers_camera_m, dtype=float)
    focus = np.asarray(focus_camera_m, dtype=float)
    center = (centers.min(0) + centers.max(0)) / 2
    # 用最大空间跨度确定完整取景，不裁剪保存的体素。
    distance = max(float(np.ptp(centers, axis=0).max()) * 1.6, .5)
    return {
        "full_scene": {"target_m": center.tolist(), "distance_m": distance},
        "cap_region": {"target_m": focus.tolist(), "distance_m": .5},
        "voxel_detail": {"target_m": focus.tolist(), "distance_m": .11},
    }


def voxel_scene_xml(centers_camera_m, size_m, cap_voxel_mask, focus_camera_m):
    """生成静态实测场景，模型 world 与 SDK 相机系重合，所有长度为米。

    每个中心对应一个真实 box geom；MuJoCo size 使用半边长。所有格子保留
    碰撞掩码，不合并成凸包、不缩小显示方块、不填充格子之间的未知空间。
    cap_voxel_mask 只控制颜色和显示分组，不放行盖体碰撞。
    """
    centers = np.asarray(centers_camera_m, dtype=float)
    mask = np.asarray(cap_voxel_mask)
    focus = np.asarray(focus_camera_m, dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 3 or not len(centers) or not np.isfinite(centers).all():
        raise ValueError("体素中心须为非空且有限的 N×3 米制数组")
    if not np.isfinite(size_m) or size_m <= 0:
        raise ValueError("体素边长须为有限正数，单位 m")
    if mask.shape != (len(centers),) or mask.dtype != bool:
        raise ValueError("盖体标记须为与体素逐行对应的 bool 数组")
    if focus.shape != (3,) or not np.isfinite(focus).all():
        raise ValueError("观察中心须为相机系三个有限米制坐标")

    root = ET.Element("mujoco", model="measured_surface_voxels_camera_frame")
    ET.SubElement(root, "compiler", angle="radian")
    ET.SubElement(root, "option", gravity="0 0 0")
    center = (centers.min(0) + centers.max(0)) / 2
    extent = max(float(np.linalg.norm(np.ptp(centers, axis=0))), .1)
    ET.SubElement(root, "statistic", center=numbers(center), extent=str(extent))
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="900")
    ET.SubElement(visual, "map", znear="0.0001", zfar="8")
    ET.SubElement(visual, "headlight", ambient=".5 .5 .5", diffuse=".5 .5 .5", specular=".03 .03 .03")
    ET.SubElement(visual, "rgba", haze=".96 .97 .98 1")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "geom", type="box", size=numbers([size_m/2]*3),
                  contype="1", conaffinity="1", margin="0", gap="0", group="0")

    # 颜色只表示深度和已知盖体归属，不作为纹理、类别或材质估计。
    anchors = np.array([[.27,.03,.33], [.23,.32,.55], [.13,.57,.55], [.37,.79,.38], [.99,.91,.14]])
    palette = np.column_stack([np.interp(np.linspace(0, 4, 32), np.arange(5), anchors[:, axis]) for axis in range(3)])
    assets = ET.SubElement(root, "asset")
    for index, color in enumerate(palette):
        ET.SubElement(assets, "material", name=f"depth_{index}", rgba=numbers([*color, 1]), shininess=".05")
    cap_color = np.array([.88,.23,.18])
    ET.SubElement(assets, "material", name="cap_observed", rgba=numbers([*cap_color, 1]), shininess=".05")
    depth_span = max(float(np.ptp(centers[:, 2])), size_m)
    bins = np.clip(((centers[:, 2]-centers[:, 2].min())/depth_span*31).astype(int), 0, 31)
    display_rgb = np.rint(palette[bins]*255).astype(np.uint8)
    display_rgb[mask] = np.rint(cap_color*255).astype(np.uint8)
    world = ET.SubElement(root, "worldbody")
    for index, position in enumerate(centers):
        ET.SubElement(world, "geom", name=f"voxel_{index}", pos=numbers(position),
                      material="cap_observed" if mask[index] else f"depth_{bins[index]}",
                      group="1" if mask[index] else "0")
    views = camera_views(centers, focus)
    for name, view in views.items():
        eye = np.asarray(view["target_m"])+[0, 0, -view["distance_m"]]
        # MuJoCo 相机朝本地 -Z 看；这里令图像右方为 +X、上方为 -Y。
        ET.SubElement(world, "camera", name=name, pos=numbers(eye),
                      xyaxes="1 0 0 0 -1 0", fovy="45")
    ET.indent(root)
    return ET.tostring(root, encoding="unicode"), display_rgb, views


def check_compiled_voxels(model, data, centers_camera_m, size_m):
    """反读编译模型，逐格检查位置、完整边长和碰撞掩码，防止半尺寸或凸包误用。"""
    import mujoco

    centers = np.asarray(centers_camera_m)
    if model.ngeom != len(centers) or model.nq != 0 or model.nbody != 1:
        raise ValueError("体素查看场景应仅包含静态观测格子")
    np.testing.assert_array_equal(model.geom_type, np.full(len(centers), mujoco.mjtGeom.mjGEOM_BOX))
    np.testing.assert_allclose(data.geom_xpos, centers, rtol=0, atol=1e-13)
    np.testing.assert_allclose(model.geom_size, np.full((len(centers), 3), size_m/2), rtol=0, atol=1e-15)
    np.testing.assert_array_equal(model.geom_contype, np.ones(len(centers)))
    np.testing.assert_array_equal(model.geom_conaffinity, np.ones(len(centers)))
    np.testing.assert_array_equal(model.geom_margin, np.zeros(len(centers)))
    return {"mujoco_box_count": int(model.ngeom), "all_box_centers_match_voxels": True,
            "all_full_box_edges_m": float(size_m), "all_boxes_collision_enabled": True,
            "convex_meshes_used": False, "placeholder_environment_loaded": False,
            "physics_steps": 0, "simulation_time_s": float(data.time)}
