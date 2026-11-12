"""握梁坐标中的少量拇指锚点，以及可由接触反馈驱动的闭合顺序。

长度为米、时间为秒。这里只决定几何目标和阶段，不发送设备指令。
"""
from dataclasses import dataclass

import numpy as np


def transform(points, matrix):
    """按行保存的米制三维点，经 A_from_B 变换到 A 系。"""
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def rigid(matrix):
    """拒绝无效坐标，不将任意矩阵默认为已标定的刚体位姿。"""
    value = np.asarray(matrix, dtype=float)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=2e-6, rtol=0)
            or not np.isclose(np.linalg.det(value[:3, :3]), 1., atol=2e-6, rtol=0)):
        raise ValueError('需要有限、右手的米制刚体变换')
    return value.copy()


@dataclass(frozen=True)
class GripShape:
    """已知梯形握梁尺寸；位姿必须来自本次观测，尺寸本身不是观测。"""
    half_length_m: float = .030
    top_half_width_m: float = .0085
    bottom_half_width_m: float = .0095
    height_m: float = .017

    def __post_init__(self):
        values = [self.half_length_m, self.top_half_width_m,
                  self.bottom_half_width_m, self.height_m]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError('握梁尺寸须为有限正数')

    @property
    def slope(self):
        """侧面半宽随向下深度增加的斜率。"""
        return (self.bottom_half_width_m - self.top_half_width_m) / self.height_m

    def project(self, points, side):
        """只投影到选定侧面，保留长轴和高度，不捏造网络分数。"""
        result = np.array(points, dtype=float, copy=True)
        result[..., 1] = side * (self.top_half_width_m - result[..., 2] * self.slope)
        return result

    def normal(self, side):
        """侧面外法向；两侧的 Z 分量均为正。"""
        normal = np.array([0., side, self.slope])
        return normal / np.linalg.norm(normal)

    def vertices(self):
        """固定握梁接触仿真使用的凸棱柱顶点。"""
        return np.array([[x, side * width, z]
                         for z, width in [(0., self.top_half_width_m),
                                          (-self.height_m, self.bottom_half_width_m)]
                         for x in [-self.half_length_m, self.half_length_m]
                         for side in [-1, 1]])

    def lateral_side(self, point_grip_m, force_normal_grip):
        """按表面位置与受压方向判断侧面；不把上沿侧向接触误当未接触。"""
        point, normal = np.asarray(point_grip_m), np.asarray(force_normal_grip)
        length = np.linalg.norm(normal)
        side = 1 if point[1] > 0 else -1
        if (length > 0 and abs(point[0]) < self.half_length_m - .0005
                and -self.height_m - .0001 <= point[2] <= .0001
                and abs(point[1] - self.project(point, side)[1]) < .001
                and side * normal[1] < -.5 * length):
            return side
        return None


def rank_anchors(quality, voxel_size_m, volume_from_grip, finger_x_offsets_m, *,
                 side=1, shape=None, threshold=.1, budget=3):
    """先给四指留出梁长，再按位置与原网络分数选至多三个不同区域。

    finger_x_offsets_m 是预型中四指相对拇指的长轴位置。锚点编号完全由
    当前网络数组计算；不以图像左右或历史体素编号决定握梁端部。
    """
    q, offsets = np.asarray(quality), np.asarray(finger_x_offsets_m, dtype=float)
    frame = rigid(volume_from_grip)
    shape = shape or GripShape()
    if (q.ndim != 3 or not np.isfinite(q).all() or offsets.shape != (4,)
            or not np.isfinite(offsets).all() or side not in (-1, 1)
            or not np.isfinite(voxel_size_m) or voxel_size_m <= 0
            or not np.isfinite(threshold) or not 0 < threshold <= 1
            or type(budget) is not int or not 1 <= budget <= 3):
        raise ValueError('网络、体素尺寸、四指排列或候选预算无效')
    ids = np.argwhere(q >= threshold)
    local = transform(ids * voxel_size_m, np.linalg.inv(frame))
    projected = shape.project(local, side)
    # 把四指长轴分布的中心放在梁内；空间不够会记录超出量，仍需后续碰撞检查。
    desired_x = float(np.clip(-.5 * (offsets.min() + offsets.max()),
                             -.75 * shape.half_length_m, .75 * shape.half_length_m))
    valid = ((local[:, 1] * side > 0) &
             (abs(local[:, 0]) <= shape.half_length_m - .003) &
             (local[:, 2] <= -.001) & (local[:, 2] >= -min(.014, shape.height_m - .001)) &
             (np.linalg.norm(projected - local, axis=1) <= .003))
    rows = []
    for i in np.flatnonzero(valid):
        point = projected[i]
        overflow = float(np.maximum(abs(point[0] + offsets) - shape.half_length_m, 0).sum())
        score = float(q[tuple(ids[i])])
        cost = float((point[0] - desired_x) ** 2 + (point[2] + .003) ** 2 + .25 * overflow ** 2
                     - score * .0005 ** 2)
        rows.append(dict(feature_index_xyz=ids[i].tolist(), raw_network_score=score,
                         raw_feature_grip_m=local[i].tolist(), anchor_grip_m=point.tolist(),
                         anchor_volume_m=transform(point, frame).tolist(),
                         finger_overflow_m=overflow, placement_cost=cost,
                         desired_anchor_x_m=desired_x, thumb_side=side))
    rows.sort(key=lambda row: (row['placement_cost'], row['feature_index_xyz']))
    selected = []
    for row in rows:
        if all(np.linalg.norm(np.array(row['anchor_grip_m']) - old['anchor_grip_m']) >= .004
               for old in selected):
            selected.append(row)
        if len(selected) == budget:
            break
    return selected


def anchored_pose(rotation, pad_hand_m, anchor_volume_m):
    """消去根平移：t = 锚点 − R·拇指FK点，位置约束精确满足。"""
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = np.asarray(anchor_volume_m) - pose[:3, :3] @ np.asarray(pad_hand_m)
    return rigid(pose)


@dataclass
class ContactSequence:
    """先确认拇指接触，再闭合其余手指；接触丢失或超时即停止。"""
    thumb_timeout_s: float = .8
    close_timeout_s: float = 1.4
    hold_s: float = .2
    phase: str = 'seek_thumb'
    reason: str = ''
    entered_s: float = 0.
    previous_s: float = -1.

    def update(self, time_s, thumb_contact, opposed_contact, *, feasible=True):
        """输入本次同刻反馈；返回阶段，failed/done 都不得自动重启。"""
        if not np.isfinite(time_s) or time_s < 0 or time_s < self.previous_s:
            raise ValueError('反馈时间必须有限且单调')
        self.previous_s = time_s
        if self.phase in ('failed', 'done'):
            return self.phase
        if not feasible:
            self.phase, self.reason = 'failed', 'collision_or_invalid_feedback'
        elif self.phase == 'seek_thumb':
            if thumb_contact:
                self.phase, self.entered_s = 'close_fingers', time_s
            elif time_s - self.entered_s > self.thumb_timeout_s:
                self.phase, self.reason = 'failed', 'thumb_contact_timeout'
        elif not thumb_contact:
            self.phase, self.reason = 'failed', 'thumb_contact_lost'
        elif self.phase == 'close_fingers':
            if opposed_contact:
                self.phase, self.entered_s = 'hold', time_s
            elif time_s - self.entered_s > self.close_timeout_s:
                self.phase, self.reason = 'failed', 'opposed_contact_timeout'
        elif not opposed_contact:
            self.phase, self.reason = 'failed', 'opposed_contact_lost'
        elif time_s - self.entered_s >= self.hold_s:
            self.phase = 'done'
        return self.phase
