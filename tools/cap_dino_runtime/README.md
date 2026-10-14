# 本机检测环境

本目录的独立环境运行微调 Grounding DINO；SAM 2 和仿真使用 `~/mujoco-env`。两者通过图片和 JSON 交换数据。

在项目根目录准备 Linux x64、Python 3.10 CPU 环境：

```bash
python3 -m venv tools/cap_dino_runtime/.venv
tools/cap_dino_runtime/.venv/bin/python -I -m pip install \
  -r tools/cap_dino_runtime/requirements.txt \
  --extra-index-url https://download.pytorch.org/whl/cpu
tools/cap_dino_runtime/.venv/bin/python -I tools/check_cap_dino_runtime.py
```

已有环境直接运行最后一条检查命令。依赖固定于本目录 `requirements.txt`。

[图像推理用法](../../doc/grounded_sam_perception.md) · [模型与数据](../../doc/assembly_contract.md#感知模型与数据)
