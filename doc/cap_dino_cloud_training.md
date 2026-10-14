# Grounding DINO 云端训练

训练包在 AutoDL 的 RTX 3090 实例上运行 MMDetection Grounding DINO，训练结束后带回最佳权重、BERT 文件和配置。本机推理操作见项目中的 `doc/grounded_sam_perception.md`。

项目中的 `doc/assembly_contract.md` 维护数据范围和评价结果。训练参数读取 `cloud_config.json`，其项目来源为 `config/grounding_dino_cloud.json`。

## 准备训练包

在本机项目根目录执行，输出到新目录：

```bash
python3 tools/package_cap_dino_training.py \
  --data-bundle artifacts/perception/cap_training_transfer_20261010_01/fuel-cap-data \
  --output-dir artifacts/perception/my_training_package
```

`--data-bundle` 指向完整性检查通过的数据包目录。当前可用训练包为 `artifacts/perception/cap_dino_cloud_20261010_02/fuel-cap-training.zip`。

包内 `images/` 保存原图，`annotations/` 保存训练、验证和两步试跑标注，`split.json` 保存固定划分。包还包含依赖、官方配置节选和训练入口，模型权重通过下载步骤取得。

## 上传、解压与安装

通过 JupyterLab 上传 ZIP 到 `/root/autodl-tmp`，在云端终端执行：

```bash
cd /root/autodl-tmp
python -m zipfile -e fuel-cap-training.zip .
cd fuel-cap-training
python cloud_workflow.py verify
bash setup_cloud.sh
source .cloud-env/bin/activate
```

解压到新目录。安装脚本建立独立环境，复用云端 CUDA PyTorch，并检查 CUDA NMS 及可变形注意力前向/反向。之后每次新开终端都先激活 `.cloud-env`。

当前适配组合为 Python 3.10、PyTorch 2.1.2+cu121、MMCV 2.1.0、MMDetection 3.3.0。完整依赖随包固定。安装或算子检查报错时先处理环境。

## 下载、基线、试跑与训练

按顺序运行：

```bash
python cloud_workflow.py fetch
python cloud_workflow.py baseline
python cloud_workflow.py smoke
python cloud_workflow.py train
```

| 阶段 | 作用 | 输出 |
| --- | --- | --- |
| `fetch` | 下载固定来源的 Grounding DINO 与 BERT | `models/` |
| `baseline` | 原权重在验证集上定位和评价 | `outputs/baseline/` |
| `smoke` | 用一张有盖、一张无盖训练图检查前向、反向与参数更新 | `outputs/smoke/` |
| `train` | 从原预训练权重开始正式训练 | `outputs/train/` |

训练入口要求同一配置的基线和试跑已通过。结果目录已有内容时拒绝覆盖；另一轮实验使用新的包或输出目录。长任务可用 `screen` 或 `tmux`，结束后按平台操作关闭实例。

## 评价与框选对照

```bash
ls outputs/train/*.pth
python cloud_workflow.py evaluate \
  --checkpoint outputs/train/best_coco_bbox_mAP_epoch_4.pth
```

checkpoint 路径以实际保存文件为准。评价结果在 `outputs/evaluate/`，包含检测指标与逐图诊断。比较有盖定位、漏检、歧义和无盖误检；框 IoU 评价框的位置。

将项目 `tools/review_cap_dino_results.py` 放入训练包目录后运行：

```bash
python review_cap_dino_results.py
```

`outputs/review/comparison.html` 内嵌原图，可直接下载查看。蓝框为人工标注，红框为过阈值候选，黄虚线为低分预测。`prediction_diagnostics.json` 保存逐图分数和重叠情况。

## 带回模型

在云端训练目录执行：

```bash
tar --exclude='outputs/train/epoch_*.pth' \
  -czf cap-dino-model-20261010.tar.gz \
  outputs models/bert models/model_sources.json \
  cloud_config.json split.json checksums.json \
  vendor requirements.txt constraints.txt \
  cloud_workflow.py cloud_hooks.py common.py check_transfer.py
sha256sum cap-dino-model-20261010.tar.gz \
  > cap-dino-model-20261010.tar.gz.sha256
```

归档保留最佳权重，省去定期 checkpoint。BERT 配套文件一起带回，供本机离线加载。通过 JupyterLab 下载压缩包和摘要文件，在本机接收目录检查：

```bash
cd /home/ace/i10_rh56_refueling_ws/artifacts/perception/cloud_model_20261010
sha256sum -c cap-dino-model-20261010.tar.gz.sha256
tar -tzf cap-dino-model-20261010.tar.gz
```

本机使用独立 MMDetection 检测环境加载该权重，再接共享环境中的 SAM 2。

## 资料

- [MMCV 安装](https://mmcv.readthedocs.io/en/latest/get_started/installation.html)
- [MMDetection v3.3.0 Grounding DINO 配置](https://github.com/open-mmlab/mmdetection/tree/v3.3.0/configs/grounding_dino)
- [AutoDL 下载文件](https://www.autodl.com/docs/down/)
