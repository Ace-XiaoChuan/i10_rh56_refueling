# 油箱盖图像定位与分割

本机微调 Grounding DINO 自动定位整盖，SAM 2 根据检测框生成掩膜。输入是一张或多张 RGB 图片，输出包括框、掩膜和叠加图。

## 本机自动推理

在项目根目录运行：

```bash
~/mujoco-env/bin/python tools/segment_cap_local.py \
  --image pic/带油箱盖_正面.jpg \
  --image-source offline_image \
  --output-dir artifacts/perception/my_local_prediction
```

`--image` 可重复传入多张图。真实相机图片用 `--image-source real_camera_rgb`。输出目录必须是新目录。

默认模型目录为 `artifacts/perception/cloud_model_20261010/unpacked/`，检测解释器为 `tools/cap_dino_runtime/.venv/bin/python`。可用 `--bundle`、`--detector-python` 指定其他已准备好的路径。模型配套配置提供类别、输入尺度和阈值。

检测环境安装见[独立环境说明](../tools/cap_dino_runtime/README.md)。检测进程结束后，共享环境加载 SAM 2；两者通过图片和 JSON 交换数据。

| 输出 | 内容 |
| --- | --- |
| `index.html` | 整批图片的结果浏览页 |
| `result.json` | 批次状态、输入、模型及环境信息 |
| `candidates.npz` | 原图像素 XYXY 框、原图大小布尔掩膜及分数 |
| `overlay.png` | 框与半透明掩膜叠加图 |

退出码 0 表示每张图都有唯一非空候选，2 表示推理完成但有图片未检出、歧义或空掩膜，运行故障返回非零。多候选保留全部结果供检查。

当前结果是二维图像信息。检测匹配分数与 SAM 掩膜质量分别保存；定位评价和三维接口状态见[项目说明](assembly_contract.md#感知模型与数据)。

已有结果可检查输入、模型、实现、环境及输出摘要：

```bash
~/mujoco-env/bin/python tools/segment_cap_local.py \
  --output-dir artifacts/perception/my_local_prediction --verify-only
```

## 共享 Python 3.10 环境

仿真、SAM 2 和原预训练定位使用 `~/mujoco-env`，`.venv`、`.venv-perception` 是兼容软链接。共享依赖记录于 `dependencies/python310-requirements.txt`。

新机器准备共享环境时，在项目根目录执行；已有环境跳过已完成的步骤：

```bash
~/mujoco-env/bin/python -m pip install -r dependencies/python310-requirements.txt
git clone https://github.com/facebookresearch/sam2.git dependencies/sam2
git -C dependencies/sam2 checkout 2b90b9f5ceec907a1c18123530e92e794ad901a4
SAM2_BUILD_CUDA=0 ~/mujoco-env/bin/python -m pip install \
  --no-build-isolation --no-deps -e dependencies/sam2
python3 tools/fetch_perception_models.py --output-dir .cache/perception/models
~/mujoco-env/bin/python -m pip check
```

入口不存在时创建 `ln -s ~/mujoco-env .venv` 和 `ln -s ~/mujoco-env .venv-perception`。模型下载工具也使用新输出目录。

## 人工框与点提示诊断

这个工具用于查看“给对了框以后，SAM 能否分好盖体”。

```bash
.venv-perception/bin/python tools/diagnose_sam_image.py \
  --image assets/hardware_photos/fuel_tank/be44de650d565ff38af7dd108c384037.jpg
```

可用 `--automatic-result /路径/result.json` 加载同图自动结果作为 A 组对照。每次打开创建独立会话，目录显示在终端和窗口底部。

1. 在原图拖框，包住整个可见盖体和握梁；也可输入 XYXY 像素坐标。
2. 点击“运行 B：仅人工框”，查看盖体边界。
3. 在盖内加正点，在误分的背景加负点；右键添加负点。
4. 点击“运行 C：框＋正负点”，在 A/B/C 页比较结果。
5. 保存提示，之后可载入同图继续操作或无窗口重跑。

改框后旧 B/C 显示失效；改点后 B 仍可保留。推理时暂停编辑，关闭窗口会等待结果保存。

会话包含 `annotations/` 提示、`trials/` 实验、`session.json` 及 `index.html`。载入提示时核对图片尺寸和摘要。位置都使用原图坐标，缩放只改变显示。

```bash
.venv-perception/bin/python tools/diagnose_sam_image.py \
  --image /绝对路径/照片.jpg \
  --prompts /绝对路径/annotations/0001.json \
  --headless --mode box_points \
  --output-dir artifacts/perception/my_manual_prediction

.venv-perception/bin/python tools/diagnose_sam_image.py \
  --verify-only --output-dir /绝对路径/已有会话
```

`--mode box` 只用框，`box_points` 用框和点。人工提示好的同时自动框差，优先检查定位；框和点都难以分好时，检查图像质量、标注及分割模型。

## 仿真 RGB-D 与原预训练对照

采集保存 RGB、配准深度、相机内参及世界变换：

```bash
.venv/bin/python tools/capture_cap_rgbd.py \
  --output-dir artifacts/perception/my_observation

.venv-perception/bin/python tools/segment_cap_image.py \
  --image artifacts/perception/my_observation/rgb.png \
  --image-source mujoco_rendered_rgb \
  --observation-json artifacts/perception/my_observation/observation.json \
  --prompt 'fuel cap.' \
  --output-dir artifacts/perception/my_pretrained_prediction
```

`segment_cap_image.py` 使用原 Transformers 预训练检测器，适合基线对照；本机微调模型使用本页首个入口。

采集中的 `evaluation_only.npz` 保存可见盖体真值，仅用于独立评价：

```bash
.venv-perception/bin/python tools/evaluate_cap_mask.py \
  --prediction-dir artifacts/perception/my_pretrained_prediction \
  --evaluation-only artifacts/perception/my_observation/evaluation_only.npz \
  --output artifacts/perception/my_mask_evaluation.json
```

评价输出各候选的 IoU、精确率和召回率。深度反投影与彩色点云查看见[RGB-D 点云工具](rgbd_point_cloud.md)，使用同份 `observation.json` 的内参、`world_from_optical` 和光轴 Z 深度。

## 已保存云端框的分割

已有云端预测可单独交给本机 SAM 2：

```bash
~/mujoco-env/bin/python tools/segment_cap_cloud_predictions.py \
  --output-dir artifacts/perception/my_cloud_box_segmentation
```

此入口读取模型包绑定的原始图片和自动框，适合检查跨环境衔接。新图片的自动定位使用本机推理入口。

## 模型资料

- [MMDetection Grounding DINO](https://github.com/open-mmlab/mmdetection/tree/v3.3.0/configs/grounding_dino)
- [Grounding DINO Transformers 接口](https://huggingface.co/docs/transformers/v4.49.0/en/model_doc/grounding-dino)
- [SAM 2 官方仓库](https://github.com/facebookresearch/sam2)
