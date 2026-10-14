"""检查划分泄漏、无盖误检和多候选评价，避免训练对照被错误统计。"""

import importlib.util
from pathlib import Path

import pytest


_spec = importlib.util.spec_from_file_location(
    "cap_cloud_common", Path(__file__).resolve().parents[1] / "tools/cloud_cap_dino/common.py")
common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(common)


def sample_data():
    """构造独立检测样本；最后五张为空标注，不额外造一个无盖类别。"""
    return {"images": [{"id": index, "file_name": f"{index}.jpg"} for index in range(1, 17)],
            "categories": [{"id": 1, "name": "fuel cap"}],
            "annotations": [{"id": index, "image_id": index, "category_id": 1,
                             "bbox": [10, 10, 20, 20]} for index in range(1, 12)]}


def test_split_retains_negatives_without_leakage():
    """训练与验证互斥，所有原图保留，试跑不能用验证图片。"""
    data = sample_data()
    settings = {"seed": 20261010, "train_positive": 8, "train_negative": 4,
                "val_positive": 3, "val_negative": 1}
    train, val, smoke = common.make_split(data, settings)
    train_ids = {row["id"] for row in train["images"]}
    val_ids = {row["id"] for row in val["images"]}
    assert not train_ids & val_ids
    assert train_ids | val_ids == set(range(1, 17))
    assert len(train["images"]) - len(train["annotations"]) == 4
    assert len(val["images"]) - len(val["annotations"]) == 1
    assert {row["id"] for row in smoke["images"]} <= train_ids
    assert len(smoke["annotations"]) == 1 and len(smoke["images"]) == 2


def test_empty_predictions_count_misses_and_negative_rejections():
    report = common.evaluate_predictions(sample_data(), [], .35, .5)
    assert report["counts"]["missed"] == 11
    assert report["counts"]["negative_correct_rejection"] == 5


def test_multiple_boxes_are_ambiguous_even_with_a_correct_box():
    prediction = {"image_id": 1, "category_id": 1, "bbox": [10, 10, 20, 20], "score": .9}
    report = common.evaluate_predictions(sample_data(), [prediction, prediction], .35, .5)
    assert report["counts"]["ambiguous"] == 1
    assert report["counts"]["localized_unique"] == 0


def test_negative_prediction_does_not_become_a_positive_match():
    prediction = {"image_id": 12, "category_id": 1, "bbox": [10, 10, 20, 20], "score": .9}
    report = common.evaluate_predictions(sample_data(), [prediction], .35, .5)
    assert report["counts"]["negative_false_positive"] == 1


def test_box_iou_is_not_segmentation_iou():
    assert common.box_iou_xywh([0, 0, 10, 10], [5, 0, 10, 10]) == pytest.approx(1 / 3)
    assert common.box_iou_xywh([0, 0, 10, 10], [20, 20, 10, 10]) == 0


def test_nonfinite_prediction_is_rejected():
    prediction = {"image_id": 1, "category_id": 1, "bbox": [10, 10, 20, 20], "score": float("nan")}
    with pytest.raises(ValueError, match="非有限"):
        common.evaluate_predictions(sample_data(), [prediction], .35, .5)
