#!/usr/bin/env python3
"""读取既有云端结果，生成可离线打开的框对照与低分预测诊断；不运行模型。"""

import argparse
import base64
import hashlib
import html
import json
import math
from pathlib import Path


def read_json(path):
    """读取云端保存的原始结果。"""
    return json.loads(path.read_text(encoding="utf-8"))


def box_iou(left, right):
    """计算原图像素 XYWH 框交并比，仅供带标注图片的事后诊断。"""
    x, y, w, h = left
    a, b, c, d = right
    intersection = max(0, min(x + w, a + c) - max(x, a)) * max(
        0, min(y + h, b + d) - max(y, b))
    union = w * h + c * d - intersection
    return intersection / union if union > 0 else 0.0


def rectangle(box, kind, label):
    """以 SVG 覆盖原图，不重采样或改写照片；边框宽度随屏幕显示保持稳定。"""
    x, y, w, h = box
    return (f'<g class="{kind}"><title>{html.escape(label)}</title>'
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" '
            'fill="none" stroke-width="2" vector-effect="non-scaling-stroke"/></g>')


def review(root, output):
    """核对两阶段来源，保留既定阈值并输出全部验证图的可视化。"""
    # 来源与图片必须一致；不修改训练包摘要，不覆盖旧评价。
    coco = read_json(root / "annotations/instances_val.json")
    checksums = read_json(root / "checksums.json")
    truth = {row["id"]: [] for row in coco["images"]}
    for annotation in coco["annotations"]:
        truth[annotation["image_id"]].append(annotation["bbox"])
    reports, results = {}, {}
    for phase in ("baseline", "evaluate"):
        folder = root / "outputs" / phase
        results[phase] = read_json(folder / "result.json")
        reports[phase] = read_json(folder / "localization_diagnosis.json")
        if results[phase]["status"] != "passed":
            raise ValueError(f"{phase} 未完成")
        if {row["image_id"] for row in reports[phase]["images"]} != set(truth):
            raise ValueError(f"{phase} 的验证图不一致")
    if results["baseline"]["recipe_sha256"] != results["evaluate"]["recipe_sha256"]:
        raise ValueError("两个阶段来源不同，不能直接对照")
    threshold = reports["baseline"]["score_threshold"]
    for field in ("score_threshold", "diagnostic_iou_threshold"):
        if reports["baseline"][field] != reports["evaluate"][field]:
            raise ValueError("两阶段评价阈值不一致")

    sections, diagnostic_rows = [], []
    for image in coco["images"]:
        name = image["file_name"]
        path = (root / "images" / name).resolve()
        if not path.is_relative_to(root / "images"):
            raise ValueError("图片路径超出数据目录")
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != checksums[f"images/{name}"]:
            raise ValueError(f"图片摘要不一致：{name}")
        image_url = "data:image/jpeg;base64," + base64.b64encode(data).decode("ascii")
        panels = []
        for phase in ("baseline", "evaluate"):
            row = next(item for item in reports[phase]["images"] if item["image_id"] == image["id"])
            if row["file_name"] != name or row["ground_truth_xywh"] != truth[image["id"]]:
                raise ValueError("诊断图名或人工框与数据不一致")
            predictions = row["all_predictions"]
            for pred in predictions:
                numbers = pred["bbox"] + [pred["score"]]
                if (pred["image_id"] != image["id"] or pred["category_id"] != 1
                        or not all(math.isfinite(value) for value in numbers)
                        or not 0 <= pred["score"] <= 1
                        or pred["bbox"][2] <= 0 or pred["bbox"][3] <= 0):
                    raise ValueError("预测编号、类别或数值无效")
            ranked = sorted(predictions, key=lambda pred: pred["score"], reverse=True)
            candidates = [pred for pred in ranked if pred["score"] >= threshold]
            if len(candidates) != row["candidate_count"]:
                raise ValueError("原始预测与候选数量不一致")

            # 对照人工框找最高 IoU 仅用于分析漏检，不是自动定位或放宽阈值。
            def overlap(pred):
                return max((box_iou(pred["bbox"], box) for box in truth[image["id"]]), default=0.0)

            best_overlap = max(ranked, key=overlap) if ranked and truth[image["id"]] else None
            low = [pred for pred in ranked[:5] if pred["score"] < threshold]
            if best_overlap is not None and best_overlap["score"] < threshold and best_overlap not in low:
                low.append(best_overlap)
            summary = {"phase": phase, "file_name": name, "status": row["status"],
                       "candidate_count": len(candidates),
                       "max_prediction_score": ranked[0]["score"] if ranked else None,
                       "max_iou_any_prediction": overlap(best_overlap) if best_overlap else None,
                       "max_iou_prediction_score": best_overlap["score"] if best_overlap else None,
                       "top5": [{"score": pred["score"], "bbox_xywh": pred["bbox"],
                                 "box_iou": overlap(pred) if truth[image["id"]] else None}
                                for pred in ranked[:5]]}
            diagnostic_rows.append(summary)
            boxes = [rectangle(box, "truth", "人工标注") for box in truth[image["id"]]]
            boxes += [rectangle(pred["bbox"], "candidate", f'候选 score={pred["score"]:.4f}')
                      for pred in candidates]
            boxes += [rectangle(pred["bbox"], "low", f'低分诊断 score={pred["score"]:.4f}')
                      for pred in low]
            details = html.escape(json.dumps(summary, ensure_ascii=False, indent=2))
            panels.append(f'<article><h3>{phase}：{html.escape(row["status"])}</h3>'
                          f'<svg viewBox="0 0 {image["width"]} {image["height"]}">'
                          f'<image href="{image_url}" width="{image["width"]}" '
                          f'height="{image["height"]}"/>{"".join(boxes)}</svg>'
                          f'<details><summary>分数与框 IoU 诊断</summary><pre>{details}</pre></details></article>')
        sections.append(f'<section><h2>{html.escape(name)}</h2><div class="panels">'
                        f'{"".join(panels)}</div></section>')

    document = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>油箱盖定位微调对照</title><style>
body{font:16px sans-serif;margin:24px;background:#f5f6f8;color:#18202a}
.panels{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}
article{background:white;padding:12px;border-radius:8px}svg{width:100%;height:auto}
.truth{stroke:#00bfff}.candidate{stroke:#ff3030}.low{stroke:#ffdf00;stroke-dasharray:7 5}
body.hide-truth .truth,body.hide-candidate .candidate,body.hide-low .low{display:none}
pre{white-space:pre-wrap;overflow-wrap:anywhere}label{margin-right:18px}
@media(max-width:800px){.panels{grid-template-columns:1fr}}
</style><h1>油箱盖定位微调对照</h1>
<p>同批次验证图。蓝框：人工标注；红框：通过原分数阈值的候选；黄虚线：低分诊断。
低分框只供分析，不计识别成功。最高 IoU 使用人工框事后计算，不能作为自动选框规则。</p>
<p>''' + f'原分数阈值：{threshold}。' + '''</p>
<label><input type="checkbox" checked onchange="document.body.classList.toggle('hide-truth',!this.checked)">人工框</label>
<label><input type="checkbox" checked onchange="document.body.classList.toggle('hide-candidate',!this.checked)">候选框</label>
<label><input type="checkbox" checked onchange="document.body.classList.toggle('hide-low',!this.checked)">低分诊断框</label>
''' + "".join(sections) + "</html>"
    output.mkdir(parents=True, exist_ok=False)
    (output / "comparison.html").write_text(document, encoding="utf-8")
    source_hashes = {f"outputs/{phase}/{name}": hashlib.sha256(
                        (root / "outputs" / phase / name).read_bytes()).hexdigest()
                     for phase in ("baseline", "evaluate")
                     for name in ("result.json", "localization_diagnosis.json")}
    diagnostics = {"recipe_sha256": results["evaluate"]["recipe_sha256"],
                   "source_sha256": source_hashes,
                   "score_threshold_unchanged": threshold, "images": diagnostic_rows,
                   "scope": "读取既有预测，不重新推理，不调整阈值；最高 IoU 使用人工框作事后诊断"}
    (output / "prediction_diagnostics.json").write_text(
        json.dumps(diagnostics, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"离线对照图：{output / 'comparison.html'}")
    for row in diagnostic_rows:
        print(row["phase"], row["file_name"], "最高分:", row["max_prediction_score"],
              "全部预测最高IoU:", row["max_iou_any_prediction"],
              "该框分数:", row["max_iou_prediction_score"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    review(root, args.output.resolve() if args.output else root / "outputs/review")


if __name__ == "__main__":
    main()
