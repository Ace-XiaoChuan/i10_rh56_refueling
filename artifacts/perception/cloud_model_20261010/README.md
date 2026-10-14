# 云端最佳模型接收目录

2026-10-10。用于接收用户首轮 MMDetection Grounding DINO 微调的最佳权重及配套 BERT、配置、来源与评价记录。

已收到云端生成的 `cap-dino-model-20261010.tar.gz` 与 `cap-dino-model-20261010.tar.gz.sha256`，实际校验通过并解压到 `unpacked/`。权重、BERT 与评价记录的来源检查见 `receipt_validation.json`。

在本目录执行 `sha256sum -c cap-dino-model-20261010.tar.gz.sha256` 核对传输。保持原文件名；核对后再在新目录中解压并适配本机路径与推理接口。

云端打包与 JupyterLab 下载步骤见 `doc/cap_dino_cloud_training.md` 第 7 节。此包面向最佳模型接入；云端原始实验和其余 checkpoint 保留。

本机当前没有 MMDetection/MMCV/MMEngine，尚未用新权重在本机重跑定位。已完成绑定同一 checkpoint 的云端自动框到本机 SAM 2 的真实衔接，结果在 `artifacts/perception/cloud_dino_sam_20261010_01/`；这不是新照片本机定位或完整自动感知验收。
