"""整盖邻域内的 RH56 优化；沿用空间损失，单目标中心由网络得分加权。"""

import numpy as np
import torch

from .rh56_spahybgen import RH56Optimization


class CapRH56Optimization(RH56Optimization):
    """替换多物体 DBSCAN 聚类，保留单个目标的网络预测中心及接触特征。"""

    def refine_contact_features_from_raw(self):
        """小目标也至少保留两个真实预测接触；中心不依赖物体真值或指定抓点。"""
        tips = torch.argwhere(self.tip_score_full > self.tip_score_full.max() * self.tip_downsample)
        if len(tips) < 2:
            raise ValueError("整盖范围内高分接触不足两个，停止生成")
        count = min(len(tips), max(2, int(len(tips) * self.focal_ratio)))
        rows = np.random.choice(len(tips), size=count, replace=False)
        focal = tips[torch.as_tensor(rows, dtype=torch.long, device=self.device)] * self.voxel_size
        self.batch_tip_focal_points = focal[None, None].repeat(self.batch, self.npts_cnt_pts, 1, 1)
        self.npts_tip_focal_points = count

        centers = torch.argwhere(self.scene_wrench_full > self.scene_wrench_full.max() * self.wrench_downsample)
        if not len(centers):
            raise ValueError("整盖范围内没有高分中心预测，停止生成")
        weights = self.scene_wrench_full[tuple(centers.T)]
        predicted_center = ((centers.float() * self.voxel_size) * weights[:, None]).sum(0) / weights.sum()
        self.index_good_wrenches_focal = centers
        self.npts_wrench_good_points = len(centers)
        self.wrench_focal_points = predicted_center[None]
        self.batch_wrench_focal_points = self.wrench_focal_points[None].repeat(self.batch, 1, 1)
        self.npts_wrench_focal_points = 1
        self.wrench_focal_points_weight = torch.ones(1, device=self.device)
