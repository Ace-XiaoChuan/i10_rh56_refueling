# Grounding DINO 云端首轮训练操作

2026-10-10。目标实例为用户已提供输出的 AutoDL 单卡 RTX 3090、Python 3.10.8、PyTorch 2.1.2+cu121、CUDA 12.1。此包已完成本地数据/来源准备；云端编译算子、模型下载、真实试跑和训练尚未完成，以你执行后生成的结果为准。

## 文件与数据范围

`fuel-cap-training.zip` 是包含原图、固定划分、训练依赖与程序的准备包，解压到新目录 `fuel-cap-training/`，无需覆盖旧 `fuel-cap-data/`。COCO 图片根目录为包内 `images/`，实际训练、验证和试跑标注位于 `annotations/instances_train.json`、`instances_val.json`、`instances_smoke.json`。

用户已确认训练/验证数量，最终图片名与摘要见 `split.json`。所有照片来自同一次拍摄、同一场景；这是同场景学习实验，没有独立测试集。`cloud_config.json` 是本次随包保存的实际配置，修改它会使旧试跑或基线不能直接用于新配置训练。项目当前参数合同维护在 `doc/assembly_contract.md` 的云端准备节。

代码与模型引用都使用固定来源。`vendor/` 是 MMDetection v3.3.0 的少量官方原始配置与许可证，未修改第三方代码；本包新增 `common.py`、`cloud_hooks.py`、`cloud_workflow.py` 属于项目独立实现。当前训练采用 MMDetection 后端，训练后的 `.pth` 不能假定能直接加载进旧 Transformers 接口；本包先完成同一新后端下的原权重/微调对照，之后再验证推理接口衔接。

## 1. 上传并解压新包

在 JupyterLab 中将 ZIP 上传到 `/root/autodl-tmp`，打开终端：

```bash
cd /root/autodl-tmp
python -m zipfile -e fuel-cap-training.zip .
cd fuel-cap-training
python cloud_workflow.py verify
```

解压前确认 `fuel-cap-training/` 不存在，避免混入上轮文件。原图、来源记录和程序摘要检查通过后才进行下一步。

## 2. 安装并检查环境

```bash
bash setup_cloud.sh
source .cloud-env/bin/activate
```

安装器建立独立 `.cloud-env` 并复用镜像的 CUDA PyTorch，依赖安装到这个环境，之后每次新开终端都先激活它。NumPy/OpenCV 等版本固定，避免依赖解析时升级到不兼容的组合；MMCV 明确使用官方预编译 wheel，不隐式转为源码编译或自动改 PyTorch。

最后会执行 `cloud_workflow.py check`，实际检查 CUDA NMS 以及可变形注意力的前向/反向，并保存 `outputs/check/result.json`。输出应包含 `status: passed`。安装或算子报错时保留日志、停在该步，不继续模型训练。

检查通过表示当前依赖能调用关键算子，不等于真实 Grounding DINO 已能训练。补丁版本 wheel 兼容以此云端实测为准，不以包可以导入替代。

## 3. 下载固定来源预训练模型

```bash
python cloud_workflow.py fetch
```

会在 `models/` 下载 MMDetection 官方 Grounding DINO checkpoint，以及固定 revision 的 `google-bert/bert-base-uncased`，保存实际摘要。这需要联网，模型与 BERT 合计约 GB 量级，下载时间不计训练速度。网络失败时保留失败目录，不会自动覆盖或续传；保留日志后再明确处理下载问题。此包不包含大模型文件。

## 4. 跑原权重验证基线

```bash
python cloud_workflow.py baseline
```

只对留出的验证照片做推理和评价，不更新权重。输出在 `outputs/baseline/`，包含实际配置、环境清单、COCO 检测预测、指标及 `localization_diagnosis.json`。后者逐图保留全部框，并报告漏检、歧义、单框偏差以及无盖误检；框 IoU 不是像素分割 IoU。

这一基线是同一 MMDetection 配置加载预训练权重的结果，不等同于历史 Transformers 自动推理报告逐位复现。

## 5. 跑两步训练检查

```bash
python cloud_workflow.py smoke
```

顺序使用训练划分内的一张有盖、一张无盖图，实际完成损失、反向传播与优化器更新。冻结 BERT；记录真实视觉参数更新、图编号、每步耗时和峰值显存，输出在 `outputs/smoke/`。

只有成功且见到视觉参数更新、两张试跑图均被处理，才生成 `result.json` 的通过记录。首次模型加载和 CUDA 初始化会使首步偏慢，两步耗时只是粗略参考，不当成稳定吞吐量或完整训练耗时预测。

## 6. 正式首轮训练

```bash
python cloud_workflow.py train
```

入口要求同一输入/配置/模型/依赖下的基线与试跑已经通过；正式训练重新从原预训练 checkpoint 初始化，不接着试跑的参数训练。输出在 `outputs/train/`，包含实际配置、依赖、逐步损失与参数更新记录、验证指标和 checkpoint。训练参数读取随包 `cloud_config.json`，不自动放大 batch 或换数据划分。

输出目录已存在会拒绝覆盖，不自动恢复或重试。错误保存在该实验的 `failure.json`；日志和部分 checkpoint 不等于整轮成功。先检查失败原因，再明确建立另一轮结果，不删除或改写原失败记录冒充成功。

长任务可以按平台后台运行方式使用 `screen` 或 `tmux`，同时保存终端输出。实例保持运行会计费；训练结束不会自动关机。请依据平台提供的定时关机功能安排预算，不在不确定模型运行时间时强行设过短终止时间。

## 7. 验证微调权重并回传

列出实际输出的 checkpoint：

```bash
ls outputs/train/*.pth
```

用文件列表中实际存在的、按验证指标保存的最佳模型路径执行下面的命令。`实际模型文件.pth` 是需要替换的占位符：

```bash
python cloud_workflow.py evaluate --checkpoint outputs/train/实际模型文件.pth
```

评价依然使用相同验证照片、输入尺寸、文本类别与固定阈值，结果在 `outputs/evaluate/`。不从多个预测框中自动选一个直接交给机器人。比较 `outputs/baseline/localization_diagnosis.json` 与 `outputs/evaluate/localization_diagnosis.json`，结合逐图框位置判断定位改进。

训练和验证完成后打包本次输出及设置：

```bash
tar -czf cap-dino-results.tar.gz outputs cloud_config.json split.json checksums.json models/model_sources.json
```

下载此文件回本机，确认能解压并包含权重与日志，再按需要关机；实例的释放和数据盘保留规则查平台。新模型回到项目时还需验证与现有推理接口、SAM 2 的兼容，不把同批次验证结果视为跨拍摄条件或机器人任务通过。

## 官方来源

- [MMCV 官方安装说明](https://mmcv.readthedocs.io/en/latest/get_started/installation.html)
- [CUDA 12.1 / PyTorch 2.1 的官方 MMCV wheel](https://download.openmmlab.com/mmcv/dist/cu121/torch2.1/index.html)
- [MMDetection v3.3.0 Grounding DINO 配置](https://github.com/open-mmlab/mmdetection/tree/v3.3.0/configs/grounding_dino)
- [Google BERT 固定版本](https://huggingface.co/google-bert/bert-base-uncased/tree/86b5e0934494bd15c9632b12f734a8a67f723594)
- [AutoDL 终端与计费说明](https://www.autodl.com/docs/quick_start/)
