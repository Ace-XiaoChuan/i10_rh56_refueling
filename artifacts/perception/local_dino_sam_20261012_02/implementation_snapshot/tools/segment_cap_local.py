#!/usr/bin/env python3
"""新图片 → 本机微调 Grounding DINO → 现有 SAM 2；保存二维结果与真实来源。"""

import argparse
import html
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from PIL import Image, ImageDraw

from fetch_perception_models import SAM_COMMIT, SAM_CONFIG
from fuel_cap_task.finetuned_dino import digest
from fuel_cap_task.finetuned_perception import detect_images, segment_detection
from fuel_cap_task.sam_diagnostic import SAMBackend


def save_json(path, value):
    """目录由入口拒绝覆盖；每个阶段更新本次运行的检查点。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def verify(output):
    """仅核对完成结果的来源与产物，不重新运行神经网络。"""
    report = json.loads((output / "result.json").read_text())
    if report["status"] != "completed":
        raise ValueError("本次推理没有完成")
    detector = json.loads((output / "detector.json").read_text())
    if detector["status"] != "completed":
        raise ValueError("检测结果没有完成")
    for mapping in (report["implementation"], detector["implementation"]):
        for name, expected in mapping.items():
            if digest(ROOT / name) != expected:
                raise ValueError(f"实现已变化：{name}")
    for path, expected in detector["detector"]["source_sha256"].items():
        if digest(path) != expected:
            raise ValueError(f"检测来源已变化：{path}")
    if "sam" in report:
        sam = report["sam"]
        for key in ("checkpoint", "models_manifest"):
            if digest(sam[key + "_path"]) != sam[key + "_sha256"]:
                raise ValueError("SAM 模型来源已变化")
        for name, expected in sam["sam_source_files"].items():
            if digest(Path(sam["sam_source"]) / name) != expected:
                raise ValueError("SAM 源码已变化")
    current_environment = {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()}
    if current_environment != report["environment"] or sys.version != report["python"]:
        raise ValueError("分割环境已变化")
    # 独立检测环境仅查询版本；不会在复核时加载模型或重跑定位。
    query = "import importlib.metadata as m,json;print(json.dumps({d.metadata['Name']:d.version for d in m.distributions()}))"
    child_env = os.environ.copy()
    for name in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
        child_env.pop(name, None)
    actual = json.loads(subprocess.check_output(
        [detector["detector"]["python_executable"], "-c", query], env=child_env, text=True))
    if actual != detector["detector"]["environment"]:
        raise ValueError("检测环境已变化")
    for row in report["images"]:
        if digest(row["image_path"]) != row["image_sha256"]:
            raise ValueError("原始图片已变化")
    for name, expected in report["outputs"].items():
        if digest(output / name) != expected:
            raise ValueError(f"推理产物已变化：{name}")
    print("本机推理的输入、模型、实现与产物摘要复核通过。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, action="append")
    parser.add_argument("--image-source", choices=("offline_image", "real_camera_rgb"))
    parser.add_argument("--bundle", type=Path,
                        default=ROOT / "artifacts/perception/cloud_model_20261010/unpacked")
    parser.add_argument("--detector-python", type=Path,
                        default=ROOT / "tools/cap_dino_runtime/.venv/bin/python")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if args.verify_only:
        verify(output)
        return 0
    if not args.image or not args.image_source:
        parser.error("推理须提供 --image 与 --image-source")
    # 先保存不可变输入副本；检测与分割始终读取同一份副本。
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report = {"status": "running", "images": [], "image_source": args.image_source,
              "valid_3d_pose": False, "target_semantics_validated": False,
              "frame_id": None, "timestamp_s": None, "timestamp_clock": "unknown",
              "python": sys.version,
              "environment": {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()},
              "implementation": {name: digest(ROOT / name) for name in (
                  "tools/segment_cap_local.py", "src/fuel_cap_task/finetuned_perception.py",
                  "src/fuel_cap_task/finetuned_dino.py", "src/fuel_cap_task/grounded_sam.py",
                  "src/fuel_cap_task/sam_diagnostic.py", "tools/fetch_perception_models.py")},
              "scope": "本机二维定位与分割，不产生三维位姿或机器人执行许可"}
    save_json(output / "result.json", report)
    try:
        paths, originals = [], []
        for index, image in enumerate(args.image):
            image = image.resolve()
            folder = output / f"image_{index:03d}"
            folder.mkdir()
            copy = folder / ("original" + image.suffix.lower())
            before = digest(image)
            shutil.copyfile(image, copy)
            if digest(copy) != before:
                raise ValueError("保存输入期间原图发生变化")
            paths.append(copy)
            originals.append((image, before))
        detection = detect_images(paths, args.bundle, args.detector_python, output / "detector.json", ROOT)
        report["detector"] = detection["detector"]
        save_json(output / "result.json", report)
        backend, gallery = None, []
        for index, (path, row, original) in enumerate(zip(paths, detection["images"], originals)):
            rgb = np.asarray(Image.open(path).convert("RGB"))
            if row["boxes_xyxy"] and backend is None:
                backend = SAMBackend(ROOT / ".cache/perception/models", ROOT / "dependencies/sam2",
                                     SAM_COMMIT, SAM_CONFIG)
                report["sam"] = backend.provenance
            prediction = segment_detection(rgb, row, backend)
            folder = path.parent
            np.savez_compressed(folder / "candidates.npz", boxes_xyxy=prediction.boxes_xyxy,
                                detector_scores=prediction.detector_scores, masks=prediction.masks,
                                mask_quality_scores=prediction.mask_quality_scores)
            # 展示只修改副本；每个框、掩膜和两类分数保持相同索引。
            overlay = rgb.copy()
            for mask in prediction.masks:
                overlay[mask] = (.55 * overlay[mask] + .45 * np.array([0, 210, 180])).astype(np.uint8)
            picture = Image.fromarray(overlay)
            draw = ImageDraw.Draw(picture)
            for box in prediction.boxes_xyxy:
                draw.rectangle(box.tolist(), outline="orange", width=3)
            picture.save(folder / "overlay.png")
            info = {"image_path": str(original[0]), "image_sha256": original[1],
                    "image_size_wh": row["image_size_wh"], "status": prediction.status,
                    "candidate_count": len(prediction.boxes_xyxy), "selected_index": prediction.selected_index,
                    "boxes_xyxy": prediction.boxes_xyxy.tolist(), "detector_scores": prediction.detector_scores.tolist(),
                    "sam_mask_quality_scores": prediction.mask_quality_scores.tolist(),
                    "mask_pixel_counts": [int(mask.sum()) for mask in prediction.masks], "folder": folder.name}
            report["images"].append(info)
            gallery.append(f'<section><h2>{html.escape(original[0].name)}：{prediction.status}</h2>'
                           f'<p>检测分数：{info["detector_scores"]}；掩膜质量：{info["sam_mask_quality_scores"]}</p>'
                           f'<img src="{folder.name}/{path.name}" alt="原图">'
                           f'<img src="{folder.name}/overlay.png" alt="本机检测与分割"></section>')
            save_json(output / "result.json", report)
            print(original[0].name, prediction.status, "候选数：", info["candidate_count"], flush=True)
        page = ('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>本机微调模型与 SAM 2</title>'
                '<style>body{font:16px sans-serif;margin:24px}img{max-width:46%;margin:1%;vertical-align:top}</style>'
                '<h1>本机微调 Grounding DINO 与 SAM 2</h1><p>检测与分割均为本次本机真实执行；'
                '检测分数与掩膜质量分别记录。</p>' + "".join(gallery) + '</html>')
        (output / "index.html").write_text(page, encoding="utf-8")
        report.update(status="completed", elapsed_seconds=time.perf_counter() - started,
                      outputs={str(path.relative_to(output)): digest(path) for path in output.rglob("*")
                               if path.is_file() and path.name != "result.json"})
        save_json(output / "result.json", report)
        print("已完成：", output)
        # 正常完成推理与有唯一候选分别表达；无盖/歧义返回 2。
        return 0 if all(row["status"] == "unique_candidate" for row in report["images"]) else 2
    except Exception:
        report.update(status="runtime_error", error=traceback.format_exc(),
                      elapsed_seconds=time.perf_counter() - started)
        save_json(output / "result.json", report)
        raise


if __name__ == "__main__":
    sys.exit(main())
