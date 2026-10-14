#!/usr/bin/env python3
"""将来源已绑定的云端自动框交给本机 SAM 2；不在本机重跑定位模型。"""

import argparse
import html
import json
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from PIL import Image

from cloud_cap_dino.common import digest, read_json
from fetch_perception_models import SAM_COMMIT, SAM_CONFIG
from fuel_cap_task.grounded_sam import assemble_result
from fuel_cap_task.sam_diagnostic import SAMBackend, overlay


def save_json(path, value):
    """保存真实执行证据，输出目录由入口拒绝覆盖。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def read_detections(bundle, images):
    """以权重、配置与原图摘要绑定预测；人工框不参与选框。"""
    settings = read_json(bundle / "cloud_config.json")
    result = read_json(bundle / "outputs/evaluate/result.json")
    request = read_json(bundle / "outputs/evaluate/request.json")
    diagnosis = read_json(bundle / "outputs/evaluate/localization_diagnosis.json")
    split = read_json(bundle / "split.json")
    checkpoint = bundle / "outputs/train/best_coco_bbox_mAP_epoch_4.pth"
    if result["status"] != "passed" or digest(checkpoint) != request["checkpoint_sha256"]:
        raise ValueError("评价未通过或最佳权重与评价不一致")
    if request["recipe_sha256"] != result["recipe_sha256"]:
        raise ValueError("评价请求与结果来源不同")
    threshold = settings["score_threshold"]
    if threshold != diagnosis["score_threshold"] or settings["classes"] != ["fuel cap"]:
        raise ValueError("类别或阈值与实际实验不一致")
    predictions_path = bundle / "outputs/evaluate/predictions.bbox.json"
    predictions = read_json(predictions_path)
    rows = diagnosis["images"]
    image_hashes = {row["file_name"]: row["sha256"] for row in split["val_images"]}
    if len(rows) != len(image_hashes) or {row["image_id"] for row in rows} != set(split["val_ids"]):
        raise ValueError("验证照片集合不一致")
    groups = {row["image_id"]: [] for row in rows}
    for prediction in predictions:
        numbers = prediction["bbox"] + [prediction["score"]]
        if (prediction["image_id"] not in groups or prediction["category_id"] != 1
                or not np.isfinite(numbers).all() or not 0 <= prediction["score"] <= 1
                or prediction["bbox"][2] <= 0 or prediction["bbox"][3] <= 0):
            raise ValueError("预测引用或数值无效")
        groups[prediction["image_id"]].append(prediction)
    prepared = []
    for row in rows:
        path = (images / row["file_name"]).resolve()
        if not path.is_relative_to(images) or digest(path) != image_hashes[row["file_name"]]:
            raise ValueError("图片路径或摘要不一致")
        raw = groups[row["image_id"]]
        if raw != row["all_predictions"]:
            raise ValueError("原始预测与已评价诊断不一致")
        candidates = [prediction for prediction in raw if prediction["score"] >= threshold]
        if len(candidates) != row["candidate_count"]:
            raise ValueError("候选数量与既定阈值不一致")
        prepared.append((row, path, candidates))
    sources = ("cloud_config.json", "split.json", "outputs/evaluate/result.json",
               "outputs/evaluate/request.json", "outputs/evaluate/predictions.bbox.json",
               "outputs/evaluate/localization_diagnosis.json")
    provenance = {"backend": "mmdetection_cloud_saved_predictions", "rerun_locally": False,
                  "bundle": str(bundle), "checkpoint": str(checkpoint),
                  "checkpoint_sha256": request["checkpoint_sha256"],
                  "recipe_sha256": result["recipe_sha256"], "score_threshold": threshold,
                  "source_sha256": {name: digest(bundle / name) for name in sources}}
    return prepared, provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=ROOT / "artifacts/perception/cloud_model_20261010/unpacked")
    parser.add_argument("--images", type=Path, default=ROOT / "pic")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    prepared, provenance = read_detections(args.bundle.resolve(), args.images.resolve())
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "detector": provenance, "images": [],
              "scope": "云端已保存自动框到本机真实 SAM 分割；未在本机重跑定位，不代表语义或物理验收",
              "implementation": {name: digest(ROOT / name) for name in (
                  "tools/segment_cap_cloud_predictions.py", "tools/cloud_cap_dino/common.py",
                  "src/fuel_cap_task/grounded_sam.py", "src/fuel_cap_task/sam_diagnostic.py",
                  "tools/fetch_perception_models.py")}}
    save_json(output / "result.json", report)
    started, backend, gallery = time.perf_counter(), None, []
    try:
        for row, path, candidates in prepared:
            rgb = np.asarray(Image.open(path).convert("RGB"))
            height, width = rgb.shape[:2]
            boxes, masks, qualities, scores = [], [], [], []
            folder = output / f'image_{row["image_id"]}'
            folder.mkdir()
            shutil.copyfile(path, folder / "original.jpg")
            # 每个候选都按原图坐标分割；没有候选则不调用 SAM，不补人工框。
            for index, candidate in enumerate(candidates):
                if backend is None:
                    backend = SAMBackend(ROOT / ".cache/perception/models", ROOT / "dependencies/sam2",
                                         SAM_COMMIT, SAM_CONFIG)
                    report["sam"] = backend.provenance
                x, y, w, h = candidate["bbox"]
                box = np.clip([x, y, x + w, y + h], [0, 0, 0, 0], [width, height, width, height])
                if box[2] <= box[0] or box[3] <= box[1]:
                    raise ValueError("裁剪后的框无有效面积")
                prompts = {"box_xyxy_px": box.tolist(), "point_coords_px": [], "point_labels": []}
                mask, quality = backend.predict(rgb, prompts)
                boxes.append(box)
                scores.append(candidate["score"])
                masks.append(mask)
                qualities.append(quality)
                overlay(rgb, mask, prompts).save(folder / f"candidate_{index}.png")
            masks_array = np.stack(masks) if masks else np.empty((0, height, width), dtype=bool)
            assembled = assemble_result(boxes, scores, ["fuel cap"] * len(boxes), masks_array,
                                        qualities, (height, width))
            np.savez_compressed(folder / "masks.npz", boxes_xyxy=assembled.boxes_xyxy,
                                detector_scores=assembled.detector_scores, masks=assembled.masks,
                                mask_quality_scores=assembled.mask_quality_scores)
            info = {"image_id": row["image_id"], "file_name": row["file_name"],
                    "image_path": str(path), "image_sha256": digest(path),
                    "image_size_wh": [width, height], "status": assembled.status,
                    "candidate_count": len(candidates), "boxes_xyxy": assembled.boxes_xyxy.tolist(),
                    "detector_scores": assembled.detector_scores.tolist(),
                    "sam_mask_quality_scores": assembled.mask_quality_scores.tolist(),
                    "mask_pixel_counts": [int(mask.sum()) for mask in masks],
                    "outputs": {file.name: digest(file) for file in folder.iterdir() if file.is_file()}}
            report["images"].append(info)
            pictures = [f'<img src="{folder.name}/original.jpg" alt="原图">']
            pictures += [f'<img src="{folder.name}/candidate_{i}.png" alt="自动框与 SAM 掩膜">'
                         for i in range(len(candidates))]
            gallery.append(f'<section><h2>{html.escape(row["file_name"])}：{assembled.status}</h2>'
                           f'<p>定位分数 {info["detector_scores"]}；SAM 质量分数 '
                           f'{info["sam_mask_quality_scores"]}</p>{"".join(pictures)}</section>')
            save_json(output / "result.json", report)
            print(row["file_name"], assembled.status, "SAM:", qualities, flush=True)
        page = ('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>云端自动框与本机 SAM</title>'
                '<style>body{font:16px sans-serif;margin:24px}img{max-width:46%;vertical-align:top;margin:1%}</style>'
                '<h1>云端自动框与本机 SAM 2</h1><p>定位来自已绑定权重的云端预测；分割为本机此次真实执行。'
                '分数不是语义验收结论；无框时不补人工提示。</p>' + "".join(gallery) + '</html>')
        (output / "index.html").write_text(page, encoding="utf-8")
        report.update(status="completed", elapsed_seconds=time.perf_counter() - started,
                      gallery_sha256=digest(output / "index.html"))
        save_json(output / "result.json", report)
        print("已完成：", output)
    except Exception:
        report.update(status="runtime_error", error=traceback.format_exc())
        save_json(output / "result.json", report)
        raise


if __name__ == "__main__":
    main()
