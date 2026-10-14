#!/usr/bin/env bash
# 只在已确认的云端镜像中安装训练依赖，复用镜像的 CUDA PyTorch。
set -euo pipefail
cap_bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$cap_bundle_dir"
python - <<'PY'
import sys
import torch
assert sys.version_info[:2] == (3, 10), '需要 Python 3.10'
assert torch.__version__.split('+')[0] == '2.1.2', '需要已确认的 PyTorch 2.1.2 镜像'
assert torch.version.cuda == '12.1' and torch.cuda.is_available(), '需要可用 CUDA 12.1 PyTorch'
print('镜像基础环境符合要求；将复用现有 PyTorch。')
PY
if [[ ! -d .cloud-env ]]; then
    python -m venv --system-site-packages .cloud-env
fi
source .cloud-env/bin/activate
# 约束版本防止依赖解析时更换镜像的 PyTorch；不安装源码版 MMCV。
python -m pip install -r requirements.txt -c constraints.txt
python -m pip install --only-binary=:all: --no-deps mmcv==2.1.0 \
    -f https://download.openmmlab.com/mmcv/dist/cu121/torch2.1/index.html
python cloud_workflow.py check
printf '以后进入训练包目录后，执行 source .cloud-env/bin/activate 激活环境。\n'
