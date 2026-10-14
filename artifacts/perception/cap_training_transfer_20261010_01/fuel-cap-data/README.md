# 在借用的 Windows 电脑上准备油箱盖定位训练

2026-10-10。用户可直接坐在借用电脑前操作，候选显卡为 RTX 3070、RTX 4060 或更高型号。具体型号、显存、驱动及现有环境尚未确认。采用 Windows → WSL 2 → Ubuntu → 独立 GPU 训练环境的准备方式，第一轮无需远程登录。

## 先理解两台电脑的分工

本机保存项目、原图、人工标注，完成模型训练后的推理与 SAM 2 联合验证。借用电脑运行训练程序，使用它自己的 CPU、内存和显卡；训练时图片、代码与权重都要能在借用电脑上读取。训练完成后，把输出的模型文件、配置、数据划分和评价记录带回本机。

这不是在本机 Python 中直接选择另一台电脑的显卡。将来使用 SSH 或远程桌面，只是改成从本机操作借用电脑，计算仍在借用电脑上执行。

## 本次提供的数据包

解压 `fuel-cap-data.zip` 后得到：

```text
fuel-cap-data/
├── images/                       原始照片，保持文件名与像素
├── annotations/
│   ├── instances_all.json         未划分的 COCO 检测标注
│   └── source_boxes/              所选人工框的原始记录
├── metadata/                     用户确认、原导出与历史来源证据
├── transfer_manifest.json        便携数据路径与来源摘要
├── model_reference.json          原 Grounding DINO 权重版本与下载来源
├── checksums.json                数据包各文件的 SHA-256
├── check_transfer.py              数据完整性及可选 CUDA 检查
├── index.html                    原图与人工框预览
└── README.md                     本说明
```

此包是可搬运的数据与准备说明，尚不是一键训练包。当前数据完成状态与数量以 `transfer_manifest.json` 和 COCO 文件为准：没有训练/验证/测试划分，没有训练配置或已微调权重。两张正右遮挡照片仅保留排除记录，未复制到训练图片目录。

`metadata/` 和原人工框文件中的本机路径用于历史追溯；借用电脑上的实际数据位置以便携清单的相对路径为准，不要求建立 `/home/ace/...`。训练读取图片根目录 `images/` 和标注文件 `annotations/instances_all.json`。

虚拟环境、ROS、MuJoCo 场景、SpaHybGen 和 SAM 2 不随包复制；本轮只训练定位模块。预训练模型也未放入此轻量包，当前推理权重约 690 MB，可在借用电脑按固定来源下载，或之后单独复制。`model_reference.json` 记录的是当前 Transformers 权重，不能直接把它当作 MMDetection 的 `.pth` checkpoint；训练框架、权重转换或导出兼容仍须核对。

## 第一步：在 Windows 中查显卡与 WSL

打开 Windows 的 PowerShell 或 Windows Terminal，执行：

```powershell
nvidia-smi
wsl --list --verbose
```

`nvidia-smi` 中查看显卡名称、总显存、已占用显存和驱动版本。其显示的 `CUDA Version` 是驱动支持的 CUDA 版本上限，不等于已经安装的 PyTorch CUDA 版本。

常见桌面 RTX 3070 和 RTX 4060 都为 8 GB；RTX 4060 Ti 有 8 GB 与 16 GB 版本，不能只根据数字大小选机器。如果有多个选择，优先考虑显存更大的设备，之后以实际试跑确认。

`wsl --list --verbose` 会显示已安装的 Linux 发行版及 WSL 版本。如果已有可用 Ubuntu 且 VERSION 为 2，优先使用它，不必重复安装。如果没有可用发行版，可在管理员 PowerShell 中执行下面的安装命令，按提示完成必要的重启及 Linux 用户名、密码设置：

```powershell
wsl --install -d Ubuntu-22.04
```

如果 WSL 报需要更新，按其提示更新。之后在开始菜单打开 Ubuntu，在 Ubuntu 终端运行：

```bash
nvidia-smi
python3 --version
```

Ubuntu 内应能看到同一张 NVIDIA 显卡。WSL 使用 Windows 提供的 NVIDIA 驱动支持，不要在 Ubuntu 中另外安装 Linux 显卡驱动。GPU 在 Ubuntu 中不可见时先处理 WSL/Windows 驱动问题，再安装训练框架。

## 第二步：把数据包放到 Ubuntu 的主目录

把 ZIP 用 U 盘或普通文件传输复制到借用电脑，再在 Windows 中解压。此步骤不涉及云服务或账号。

在借用电脑的 Ubuntu 终端执行：

```bash
mkdir -p ~/cap-training-work
cd ~/cap-training-work
explorer.exe .
```

最后一条命令会在 Windows 资源管理器中打开这个 Ubuntu 文件夹。将解压出的整个 `fuel-cap-data` 文件夹复制进去。复制后回到 Ubuntu：

```bash
cd ~/cap-training-work/fuel-cap-data
python3 check_transfer.py
```

检查通过说明数据传输完整，并不表示训练环境已经就绪。可以双击 `index.html` 查看人工框；文件名中的中文保留原样。实际训练时使用 Ubuntu 主目录中的副本，避免持续从 U 盘或 `/mnt/c/` 跨文件系统读取训练图片。

## 第三步：建立独立的 Python 训练环境

借用电脑已经有深度学习环境时，先核对它能否支持此次训练。需要新环境时，在 Ubuntu 中执行：

```bash
sudo apt update
sudo apt install python3.10-venv
python3.10 -m venv ~/cap-dino-train-env
source ~/cap-dino-train-env/bin/activate
python --version
```

这会在借用电脑建立独立 Python 环境。以后每次新开终端训练，先运行 `source ~/cap-dino-train-env/bin/activate`，确保命令使用正确解释器。

接下来安装 GPU 版 PyTorch、训练框架及其匹配的依赖。现阶段不提供未经核对的安装组合：最终版本须与确切显卡、驱动及训练实现相匹配，MMDetection 还需要匹配的 MMCV/CUDA 算子。不要用本机的 `python310-requirements.txt` 在这里装训练环境，它固定的是 CPU 版 PyTorch。

GPU 依赖安装后，在数据包目录内运行：

```bash
python check_transfer.py --gpu
```

程序会显示当前 Python、PyTorch、PyTorch CUDA 版本、CUDA 可用状态，并实际执行小规模 CUDA 矩阵运算。只有此运算通过，才能说明当前 Python 可以使用显卡；仍需用 Grounding DINO 单步训练检查损失、反向传播和显存。

## 第四步：准备并运行首轮微调

最终训练包还需加入：已确认的数据划分、固定版本的训练入口与依赖、训练配置、预训练权重、评价入口及回传说明。当前 16 张照片来自同一次拍摄、同一场景；首轮按角度留出时只能报告同场景表现，正式跨批次泛化仍需另一批照片。

配置将明确文本/类别、训练哪些参数、输入尺寸、batch size、学习率、轮数、负样本是否保留、随机种子和保存方式。先建立原权重基线，用少量训练步核对前向、损失、反向、权重更新及峰值显存，再运行正式首轮。当前尚未验证这些项目，不把说明文档中的准备命令当作已启动训练。

训练中按日志查看损失与验证表现，完整保存输出目录。显存不足时报错停下来修改配置，不把错误当作训练完成。长时间运行时保持借用电脑通电并暂时避免自动睡眠；中断后是否可恢复需检查是否存在完整 checkpoint。

## 第五步：带回模型与实验记录

建议把本次实验输出整体打包，至少保留模型权重、模型/处理器配置、实际训练配置、训练和验证图片清单、随机种子、依赖版本、原权重基线和微调评价结果、日志及输入来源摘要。

checkpoint 是保存下来的模型状态；部分格式还包含优化器和进度，用于继续训练。推理部署可能只需要其中的模型权重。文件扩展名相同不保证架构或框架兼容。如果采用 MMDetection，输出 `.pth` 不能假定能直接送进现有 Transformers 接口，需要转换验证或明确的新推理后端，之后再接 SAM 2。

将输出用 U 盘或文件传输带回本机，保存成独立实验目录。先验证输入输出格式、原图定位结果、无盖拒绝及与 SAM 2 的联动，再决定如何接入项目自动推理。

## 官方参考

- [Windows 安装 WSL](https://learn.microsoft.com/en-us/windows/wsl/install)
- [WSL GPU 计算与 Windows 驱动](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gpu-compute)
- [PyTorch 安装与 CUDA 可用性验证](https://pytorch.org/get-started/locally/)
- [MMDetection 安装与 CUDA 算子依赖](https://mmdetection.readthedocs.io/en/latest/get_started.html)
- [Grounding DINO 自定义数据集微调](https://github.com/open-mmlab/mmdetection/blob/main/configs/grounding_dino/README.md#custom-dataset)
- [NVIDIA RTX 3070 显存规格](https://www.nvidia.com/en-us/geforce/graphics-cards/30-series/rtx-3070-3070ti/)
- [NVIDIA RTX 4060/4060 Ti 显存规格](https://www.nvidia.com/en-us/geforce/graphics-cards/40-series/rtx-4060-4060ti/)
