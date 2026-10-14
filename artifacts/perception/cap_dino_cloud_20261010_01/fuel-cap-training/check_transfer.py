#!/usr/bin/env python3
"""核对便携数据包；可选检查当前 Python 是否能调用 CUDA 显卡。"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def digest(path):
    """分块读文件，避免大文件整体进入内存。"""
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def verify(root):
    """确认传输后的文件完整，且图片、类别与框保持原来的关联。"""
    checksums = json.loads((root / "checksums.json").read_text(encoding="utf-8"))
    for name, expected in checksums.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root) or digest(path) != expected:
            raise ValueError(f"文件摘要不匹配：{name}")
    coco = json.loads((root / "annotations/instances_all.json").read_text(encoding="utf-8"))
    manifest = json.loads((root / "transfer_manifest.json").read_text(encoding="utf-8"))
    images = {item["id"]: item for item in coco["images"]}
    categories = {item["id"] for item in coco["categories"]}
    if len(images) != len(coco["images"]):
        raise ValueError("图片编号重复")
    if len({item["id"] for item in coco["annotations"]}) != len(coco["annotations"]):
        raise ValueError("标注编号重复")
    positive_ids = set()
    for annotation in coco["annotations"]:
        image = images[annotation["image_id"]]
        if annotation["category_id"] not in categories:
            raise ValueError("标注引用未知类别")
        x, y, width, height = annotation["bbox"]
        if not (0 <= x < x + width <= image["width"]
                and 0 <= y < y + height <= image["height"]):
            raise ValueError("标注框超出图片范围")
        positive_ids.add(annotation["image_id"])
    for image in images.values():
        if not (root / "images" / image["file_name"]).is_file():
            raise ValueError("缺少原图")
    negative_ids = set(images) - positive_ids
    expected_negative_ids = {item["image_id"] for item in manifest["images"]
                             if item["label"] == "cap_absent_negative"}
    if negative_ids != expected_negative_ids:
        raise ValueError("空标注与用户确认的无盖照片不一致")
    print(f"数据完整性通过：{len(images)} 张图，{len(coco['annotations'])} 个框，"
          f"{len(negative_ids)} 张无盖空标注；未划分、未训练。")


def check_gpu():
    """先报告驱动信息，再实际执行少量 CUDA 运算；不加载训练模型。"""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,memory.total,memory.free,driver_version",
             "--format=csv,noheader"], capture_output=True, text=True, timeout=20,
        )
        print("显卡驱动查询：", result.stdout.strip() or result.stderr.strip())
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f"显卡驱动查询失败：{error}")
    try:
        import torch
    except ImportError:
        print("当前 Python 未安装 PyTorch；请先准备独立 GPU 训练环境。")
        return False
    print(f"Python：{sys.version.split()[0]}；PyTorch：{torch.__version__}；"
          f"PyTorch CUDA：{torch.version.cuda}；CUDA 可用：{torch.cuda.is_available()}")
    if not torch.cuda.is_available():
        return False
    # 真正创建 CUDA 张量并等待计算完成，避免只看显卡名称就认定可用。
    values = torch.ones((256, 256), device="cuda")
    product = values @ values
    torch.cuda.synchronize()
    if product[0, 0].item() != 256:
        raise ValueError("CUDA 运算结果异常")
    print(f"CUDA 运算通过：{torch.cuda.get_device_name(0)}。"
          "此检查不代表 Grounding DINO 训练或显存适配通过。")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--gpu", action="store_true", help="额外检查当前 Python 的 CUDA 运算")
    args = parser.parse_args()
    verify(args.root.resolve())
    if args.gpu and not check_gpu():
        raise SystemExit(2)


if __name__ == "__main__":
    main()
