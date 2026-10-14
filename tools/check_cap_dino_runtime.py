#!/usr/bin/env python3
"""检查独立 CPU 推理环境与必要算子；不加载模型或读取图像标注。"""

import argparse
import importlib.metadata
import json
from pathlib import Path
import platform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    import torch
    from mmcv.ops import nms
    from mmcv.ops.multi_scale_deform_attn import multi_scale_deformable_attn_pytorch

    expected = {"torch": "2.1.2+cpu", "torchvision": "0.16.2+cpu", "mmcv": "2.1.0",
                "mmengine": "0.10.7", "mmdet": "3.3.0", "numpy": "1.26.4",
                "transformers": "4.38.2"}
    actual = {name: importlib.metadata.version(name) for name in expected}
    if actual != expected or platform.python_version().split(".")[:2] != ["3", "10"]:
        raise ValueError("Python 或关键包版本与固定推理环境不一致")
    boxes = torch.tensor([[0., 0., 10., 10.], [1., 1., 9., 9.]])
    _, kept = nms(boxes, torch.tensor([.9, .8]), .5)
    if kept.tolist() != [0]:
        raise ValueError("CPU NMS 算子结果错误")
    # CPU 路径使用 PyTorch 可变形注意力，与 CUDA 编译路径分别验证。
    values = torch.ones(1, 4, 2, 4)
    shapes = torch.tensor([[2, 2]])
    locations = torch.full((1, 3, 2, 1, 2, 2), .5)
    weights = torch.full((1, 3, 2, 1, 2), .5)
    result = multi_scale_deformable_attn_pytorch(values, shapes, locations, weights)
    if result.shape != (1, 3, 8) or not torch.allclose(result, torch.ones_like(result)):
        raise ValueError("CPU 可变形注意力结果错误")
    report = {"status": "passed", "python": platform.python_version(), "packages": actual,
              "device": "cpu", "cpu_nms": True, "cpu_deformable_attention": True,
              "scope": "环境与最小算子检查；完整模型及图像结果须单独运行"}
    if args.output:
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
