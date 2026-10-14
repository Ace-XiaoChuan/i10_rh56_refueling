"""协调独立检测进程与现有 SAM 2；两阶段使用同一输入图片并保留所有候选。"""

import json
import os
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image

from .finetuned_dino import candidate_arrays, digest
from .grounded_sam import assemble_result


def detect_images(images, bundle, python, output, root):
    """跨环境只交换文件与 JSON，检测完成退出后再加载 SAM，减少内存峰值。"""
    images = [Path(path).resolve() for path in images]
    output, root = Path(output).resolve(), Path(root).resolve()
    if not images or not Path(python).is_file():
        raise ValueError("须提供图片及已安装的独立检测 Python")
    command = [str(python), str(root / "tools/infer_finetuned_cap.py"),
               "--bundle", str(Path(bundle).resolve()), "--output", str(output)]
    for path in images:
        command.extend(("--image", str(path)))
    # 避免用户终端的 ROS/Python 搜索路径混入独立检测环境。
    child_env = os.environ.copy()
    for name in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
        child_env.pop(name, None)
    for name in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
        child_env[name] = "4"
    log = output.with_suffix(".log")
    with log.open("x", encoding="utf-8") as stream:
        subprocess.run(command, env=child_env, check=True, stdout=stream, stderr=subprocess.STDOUT)
    report = json.loads(output.read_text())
    if (report["status"] != "completed" or len(report["images"]) != len(images)
            or report["detector"]["backend"] != "mmdetection_local_finetuned"
            or report["detector"]["rerun_locally"] is not True):
        raise ValueError("检测进程没有返回本轮完成的真实推理")
    for name, expected in report["implementation"].items():
        if digest(root / name) != expected:
            raise ValueError("检测过程中实现发生变化")
    for path, row in zip(images, report["images"]):
        if row["image_path"] != str(path) or digest(path) != row["image_sha256"]:
            raise ValueError("检测输出与输入图片不匹配")
        width, height = Image.open(path).size
        if row["image_size_wh"] != [width, height]:
            raise ValueError("检测返回的原图尺寸不一致")
        boxes, scores = candidate_arrays(row["raw_predictions"], (height, width),
                                        report["detector"]["score_threshold"])
        if boxes.tolist() != row["boxes_xyxy"] or scores.tolist() != row["scores"]:
            raise ValueError("候选与固定阈值下的全部预测不一致")
    return report


def segment_detection(rgb, detection, backend):
    """每个原图像素框提示 SAM；没有检测框时不调用分割器。"""
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("SAM 输入须为 uint8 RGB 图片")
    # PIL 转出的数组可能只读；Torch 共享内存转换要求可写，复制不改变原图。
    if not rgb.flags.writeable:
        rgb = rgb.copy()
    height, width = rgb.shape[:2]
    if detection["image_size_wh"] != [width, height]:
        raise ValueError("分割图片与检测尺寸不一致")
    masks, qualities = [], []
    for box in detection["boxes_xyxy"]:
        prompts = {"box_xyxy_px": box, "point_coords_px": [], "point_labels": []}
        mask, quality = backend.predict(rgb, prompts)
        masks.append(mask)
        qualities.append(quality)
    masks_array = np.stack(masks) if masks else np.empty((0, height, width), dtype=bool)
    return assemble_result(detection["boxes_xyxy"], detection["scores"], detection["labels"],
                           masks_array, qualities, (height, width))

