"""完整实测/假设侧面点对整手的带符号间隙，安全区也保留边界方向。"""
import mujoco
import numpy as np
from scipy.spatial import cKDTree

from .observed_hand_geometry import primitive_inside_depth


def depth_and_gradient(kind, size, points, planes=None):
    """实体内深度及对局部点的导数；沿用原筛选定义，负值为外侧。"""
    p, size = np.asarray(points, dtype=float), np.asarray(size, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all():
        raise ValueError('碰撞点须为有限N×3数组')
    depth = primitive_inside_depth(kind, size, p, planes)
    gradient = np.zeros_like(p)
    if kind == 6:
        axis = (size[:3]-abs(p)).argmin(axis=1)
        gradient[np.arange(len(p)), axis] = -np.sign(p[np.arange(len(p)), axis])
    elif kind == 7:
        equations = np.asarray(planes)
        axis = (p@equations[:, :3].T+equations[:, 3]).argmax(axis=1)
        gradient = -equations[axis, :3]
    elif kind in (2, 3):
        nearest = p.copy()
        if kind == 3:
            nearest[:, 2] -= np.clip(p[:, 2], -size[1], size[1])
        gradient = -nearest/np.maximum(np.linalg.norm(nearest, axis=1, keepdims=True), 1e-15)
    elif kind == 5:
        radial = np.linalg.norm(p[:, :2], axis=1)
        mask = size[0]-radial <= size[1]-abs(p[:, 2])
        gradient[mask, :2] = -p[mask, :2]/np.maximum(radial[mask, None], 1e-15)
        gradient[~mask, 2] = -np.sign(p[~mask, 2])
    return depth, gradient


class SignedHandClearance:
    """局部约束保留近边界见证点，接受检查仍覆盖全部点和原生自碰。

    假设面只在独立类别追加，绝不改写实测标签。它仍是采样表面，不能
    据此称为重建了实体内部或证明了连续接近路径。
    """
    def __init__(self, kinematics, scene_points, target_mask, *, assumed_points=None,
                 tolerance_m=.0001, active_band_m=.003, witnesses_per_geom=16):
        self.kin = kinematics
        points, mask = np.asarray(scene_points, dtype=float), np.asarray(target_mask)
        extra = np.empty((0, 3)) if assumed_points is None else np.asarray(assumed_points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
                or mask.shape != (len(points),) or mask.dtype.kind != 'b'
                or extra.ndim != 2 or extra.shape[1] != 3
                or not np.isfinite(points).all() or not np.isfinite(extra).all()
                or not np.isfinite([tolerance_m, active_band_m]).all()
                or tolerance_m < 0 or active_band_m <= tolerance_m
                or type(witnesses_per_geom) is not int or witnesses_per_geom < 1):
            raise ValueError('完整场景、分类或间隙参数无效')
        self.points = np.concatenate([points, extra])
        self.labels = np.concatenate([mask.astype(int), np.full(len(extra), 2)])
        self.class_names = ('environment', 'target', 'assumed_side', 'self')
        self.tree = cKDTree(self.points)
        self.tolerance = float(tolerance_m)
        self.band = float(active_band_m)
        self.witnesses = witnesses_per_geom
        # 仅增大旁路查询模型的margin以发现正间隙碰撞对；原检查模型保持。
        self.near_model = mujoco.MjModel.from_xml_string(self.kin.hand.xml_string)
        self.near_model.geom_margin[:] = self.band
        self.near_model.geom_gap[:] = self.band
        self.near_data = mujoco.MjData(self.near_model)

    def _scene_geometry(self, geom, band):
        """扩张包围盒仅作广相；实体内所有点始终在查询中。"""
        hand, pose = self.kin.hand, self.kin.pose
        rg = hand.data.geom_xmat[geom].reshape(3, 3)
        R = pose[:3, :3]@rg
        t = pose[:3, :3]@hand.data.geom_xpos[geom]+pose[:3, 3]
        kind, planes, low, high = hand.shapes[geom]
        center = (low+high)*.5
        radius = np.linalg.norm((high-low)*.5+band)+1e-9
        rows = np.asarray(self.tree.query_ball_point(R@center+t, radius), dtype=int)
        local = (self.points[rows]-t)@R
        selected = np.all((local >= low-band) & (local <= high+band), axis=1)
        rows, local = rows[selected], local[selected]
        if not len(rows):
            return rows, np.empty(0), np.empty((0, 3)), R
        # 凸包分块，避免完整mesh面数乘近邻点数产生很大的临时矩阵。
        depth, grad = [], []
        for first in range(0, len(rows), 2048):
            d, g = depth_and_gradient(kind, hand.model.geom_size[geom], local[first:first+2048], planes)
            depth.append(d); grad.append(g)
        return rows, np.concatenate(depth), np.concatenate(grad), R

    def _self_distance(self, pair):
        """同一MuJoCo引擎的geom对带符号距离，禁止另一版本近似替代。"""
        return float(mujoco.mj_geomDistance(self.kin.hand.model, self.kin.hand.data,
                                          int(pair[0]), int(pair[1]), self.band*2, None))

    def evaluate(self, *, linearize=False):
        """返回各类最大超差；线性化时另给g(q)+JΔq≥0的米制行。"""
        k, h = self.kin, self.kin.hand
        maxima = {name: 0. for name in self.class_names}
        counts = {name: 0 for name in self.class_names}
        gaps, jacobians, keys = [], [], []
        band = self.band if linearize else 0.
        for geom in h.geoms:
            rows, depth, gradient, R = self._scene_geometry(geom, band)
            for group, name in enumerate(self.class_names[:3]):
                members = np.flatnonzero(self.labels[rows] == group)
                if not len(members):
                    continue
                excess = depth[members]-self.tolerance
                maxima[name] = max(maxima[name], float(excess.max()), 0.)
                counts[name] += int((excess > 0).sum())
                if not linearize:
                    continue
                active = members[depth[members] >= -self.band]
                if not len(active):
                    continue
                # 各局部法向方向先保留最严重点，避免一片面淹没另一面。
                groups = {}
                for i in active:
                    normal_key = tuple(np.round(gradient[i], 2))
                    if normal_key not in groups or depth[i] > depth[groups[normal_key]]:
                        groups[normal_key] = int(i)
                critical = sorted(groups.values(), key=lambda i: -depth[i])[:self.witnesses]
                top = active[np.argsort(-depth[active], kind='stable')[:self.witnesses]]
                chosen = sorted(set(critical) | set(top.tolist()))
                for i in chosen:
                    point_hand = k.pose[:3, :3].T@(self.points[rows[i]]-k.pose[:3, 3])
                    _, jp, _ = k.attached_point(int(h.model.geom_bodyid[geom]), point_hand)
                    gaps.append(self.tolerance-depth[i])
                    jacobians.append((R@gradient[i])@jp)
                    keys.append((name, int(geom), int(rows[i])))
        # 接受检查使用原模型生成的全部自碰；正间隙查询只用于求方向。
        for contact in h.data.contact:
            excess = -float(contact.dist)-self.tolerance
            maxima['self'] = max(maxima['self'], excess, 0.)
            counts['self'] += int(excess > 0)
        if linearize:
            self.near_data.qpos[:] = h.data.qpos
            mujoco.mj_forward(self.near_model, self.near_data)
            pairs = sorted({tuple(sorted(int(i) for i in c.geom)) for c in self.near_data.contact
                            if c.dist < self.band})
            pose, drivers = k.pose.copy(), k.drivers.copy()
            for pair in pairs:
                distance = self._self_distance(pair)
                if distance > self.band:
                    continue
                derivative = np.zeros(12)
                # 沿真实六驱动方向中央差分，不把接触见证点导数当作精确CCD导数。
                for j in range(6):
                    d = np.eye(6)[j]*1e-6
                    # 限位处用可行单侧差分；常规内点使用中央差分。
                    plus = np.minimum(drivers+d, k.drives.upper)
                    minus = np.maximum(drivers-d, k.drives.lower)
                    k.set_state(pose, plus); dp = self._self_distance(pair)
                    k.set_state(pose, minus); dm = self._self_distance(pair)
                    derivative[j+6] = (dp-dm)/(plus[j]-minus[j])
                k.set_state(pose, drivers)
                gaps.append(distance+self.tolerance); jacobians.append(derivative)
                keys.append(('self', int(pair[0]), int(pair[1])))
        return dict(maxima=maxima, counts=counts, feasible=all(v <= 1e-10 for v in maxima.values()),
                    gaps=np.asarray(gaps), jacobian=np.asarray(jacobians).reshape(-1, 12), keys=keys)
