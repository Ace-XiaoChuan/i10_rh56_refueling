"""云端实验的数据划分、来源摘要与框评价；此模块不加载神经网络。"""

import hashlib
import json
import math
from pathlib import Path
import random


def digest(path):
    """计算输入和结果摘要，绑定实际文件内容。"""
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def read_json(path):
    """读取含中文文件名的 JSON。"""
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, value):
    """使用可读 JSON 保存记录，调用方负责拒绝覆盖。"""
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + "\n", encoding="utf-8")


def make_split(coco, settings):
    """按用户确认的数量分组抽取，保留无盖图，固定编号不重排来源。"""
    image_ids = {item["id"] for item in coco["images"]}
    if len(image_ids) != len(coco["images"]):
        raise ValueError("图片编号重复")
    positive = {item["image_id"] for item in coco["annotations"]}
    if not positive <= image_ids:
        raise ValueError("框引用未知图片")
    negative = image_ids - positive
    if len(positive) != settings["train_positive"] + settings["val_positive"]:
        raise ValueError("有盖照片数量与确认方案不一致")
    if len(negative) != settings["train_negative"] + settings["val_negative"]:
        raise ValueError("无盖照片数量与确认方案不一致")
    rng = random.Random(settings["seed"])
    positive_order, negative_order = sorted(positive), sorted(negative)
    rng.shuffle(positive_order)
    rng.shuffle(negative_order)
    val_ids = set(positive_order[:settings["val_positive"]]
                  + negative_order[:settings["val_negative"]])
    train_ids = image_ids - val_ids

    def subset(ids):
        return {"info": {"description": "同批次油箱盖定位首轮实验，人工框与用户确认无盖"},
                "images": [row for row in coco["images"] if row["id"] in ids],
                "annotations": [row for row in coco["annotations"] if row["image_id"] in ids],
                "categories": coco["categories"]}

    # 试跑一张有盖、一张无盖，顺序明确；它们只来自训练划分。
    smoke_ids = {min(train_ids & positive), min(train_ids & negative)}
    return subset(train_ids), subset(val_ids), subset(smoke_ids)


def box_iou_xywh(left, right):
    """原图像素 XYWH 框的交并比，不能当作像素分割 IoU。"""
    lx, ly, lw, lh = left
    rx, ry, rw, rh = right
    intersection = max(0, min(lx + lw, rx + rw) - max(lx, rx)) * max(
        0, min(ly + lh, ry + rh) - max(ly, ry))
    union = lw * lh + rw * rh - intersection
    return intersection / union if union > 0 else 0.0


def evaluate_predictions(coco, predictions, score_threshold, iou_threshold):
    """逐图报告全部预测、歧义和无盖误检，不自动挑一个框用于执行。"""
    ground_truth = {item["id"]: [] for item in coco["images"]}
    for annotation in coco["annotations"]:
        ground_truth[annotation["image_id"]].append(annotation["bbox"])
    groups = {item["id"]: [] for item in coco["images"]}
    for prediction in predictions:
        if prediction["image_id"] not in groups or prediction["category_id"] != 1:
            raise ValueError("预测引用未知图片或类别")
        numbers = prediction["bbox"] + [prediction["score"]]
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError("预测包含非有限值")
        if prediction["bbox"][2] <= 0 or prediction["bbox"][3] <= 0:
            raise ValueError("预测框宽高无效")
        if not 0 <= prediction["score"] <= 1:
            raise ValueError("预测分数超出范围")
        groups[prediction["image_id"]].append(prediction)
    counts = {name: 0 for name in ("localized_unique", "missed", "ambiguous",
                                   "mislocalized_unique", "negative_false_positive",
                                   "negative_correct_rejection")}
    rows = []
    for image in coco["images"]:
        truth = ground_truth[image["id"]]
        all_predictions = groups[image["id"]]
        candidates = [row for row in all_predictions if row["score"] >= score_threshold]
        best_iou = max((box_iou_xywh(candidate["bbox"], target)
                        for candidate in candidates for target in truth), default=0.0)
        if not truth:
            status = "negative_false_positive" if candidates else "negative_correct_rejection"
        elif not candidates:
            status = "missed"
        elif len(candidates) > 1:
            status = "ambiguous"
        else:
            status = "localized_unique" if best_iou >= iou_threshold else "mislocalized_unique"
        counts[status] += 1
        rows.append({"image_id": image["id"], "file_name": image["file_name"],
                     "status": status, "ground_truth_xywh": truth,
                     "candidate_count": len(candidates), "best_box_iou_diagnostic": best_iou,
                     "all_predictions": all_predictions})
    return {"counts": counts, "images": rows, "score_threshold": score_threshold,
            "diagnostic_iou_threshold": iou_threshold,
            "scope": "同批次验证；框 IoU 与阈值仅作定位诊断，不代表语义或机器人执行验收"}
