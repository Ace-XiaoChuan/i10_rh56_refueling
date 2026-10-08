"""保留派生规划形状，使用随机 MuJoCo 场景更新各几何的世界位姿。"""
import copy
import numpy as np


def scene_snapshot(baseline, model, data):
    # 复制规划形状及碰撞规则，只替换位姿；凹腔分片不会被整体凸化。
    snapshot = copy.deepcopy(baseline)
    if snapshot['frame_id'] != 'world':
        raise ValueError('Expected world-frame planning scene')
    for entry in snapshot['objects']:
        # source_geom 将规划对象对应到 MuJoCo 几何；核对所属 body 防止错配。
        geom = entry['source_geom']
        if model.body(int(model.geom_bodyid[geom])).name != entry['source_body']:
            raise ValueError('Planning geometry/body mapping changed')
        # 使用前向计算后的世界坐标：位置单位 m，方向为 3×3 旋转矩阵。
        entry['position'] = data.geom_xpos[geom].tolist()
        entry['rotation_matrix'] = data.geom_xmat[geom].reshape(3, 3).tolist()
    return snapshot


def check_snapshot(actual, expected):
    """拒绝缺失或变化的形状、碰撞排除和位姿；仅容许浮点舍入误差。"""
    remainder = copy.deepcopy(actual)
    if len(actual['objects']) != len(expected['objects']):
        raise ValueError('Planning scene object count mismatch')
    # 位姿允许微小舍入差；核对后替换为期望值，再严格比较其余形状与规则。
    for a, e, r in zip(actual['objects'], expected['objects'], remainder['objects']):
        for key in ('position', 'rotation_matrix'):
            if not np.allclose(a[key], e[key], atol=1e-12, rtol=0):
                raise ValueError('Planning scene pose differs from sampled world')
            r[key] = e[key]
    if remainder != expected:
        raise ValueError('Planning scene shapes or collision rules changed')
