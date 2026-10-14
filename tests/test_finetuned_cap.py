"""检查本地模型来源、阈值与分割边界；真实模型结果由独立运行验证。"""

import json

import numpy as np
import pytest

from fuel_cap_task.finetuned_dino import bound_model, candidate_arrays, digest
from fuel_cap_task.finetuned_perception import segment_detection


def test_threshold_preserves_all_candidates_and_original_boxes():
    """阈值边界保留，画幅外坐标裁剪，多候选不能被缩成最高分一个。"""
    boxes = np.array([[-1., 0., 5., 5.], [2., 2., 12., 12.], [1., 1., 3., 3.]], dtype=np.float32)
    before = boxes.copy()
    raw = {"boxes_xyxy": boxes, "scores": [.35, .9, .1], "labels": [0, 0, 0]}
    candidates, scores = candidate_arrays(raw, (10, 10), .35)
    assert candidates.tolist() == [[0, 0, 5, 5], [2, 2, 10, 10]]
    assert scores.tolist() == pytest.approx([.35, .9])
    assert np.array_equal(boxes, before)


@pytest.mark.parametrize("raw", [
    {"boxes_xyxy": [[0, 0, 1, 1]], "scores": [float("nan")], "labels": [0]},
    {"boxes_xyxy": [[0, 0, 1, 1]], "scores": [.9], "labels": [1]},
    {"boxes_xyxy": [[20, 20, 21, 21]], "scores": [.9], "labels": [0]},
    {"boxes_xyxy": [[0, 0, 1, 1]], "scores": [], "labels": []},
])
def test_invalid_detector_outputs_are_rejected(raw):
    with pytest.raises(ValueError):
        candidate_arrays(raw, (10, 10), .35)


def test_empty_detector_output_does_not_call_sam():
    boxes, scores = candidate_arrays({"boxes_xyxy": [], "scores": [], "labels": []}, (4, 6), .35)
    prediction = segment_detection(np.zeros((4, 6, 3), dtype=np.uint8),
                                  {"boxes_xyxy": boxes.tolist(), "scores": scores.tolist(),
                                   "labels": [], "image_size_wh": [6, 4]}, None)
    assert prediction.status == "not_detected"
    assert prediction.selected_index is None
    assert prediction.masks.shape == (0, 4, 6)


def test_ambiguous_detection_segments_every_box_in_order():
    """分割可返回所有候选，任务有效性仍须保持歧义状态。"""
    class Backend:
        def __init__(self):
            self.boxes = []

        def predict(self, rgb, prompts):
            self.boxes.append(prompts["box_xyxy_px"])
            return np.ones(rgb.shape[:2], dtype=bool), .8

    backend = Backend()
    boxes = [[0, 0, 2, 2], [2, 1, 5, 3]]
    prediction = segment_detection(np.zeros((4, 6, 3), dtype=np.uint8),
                                  {"boxes_xyxy": boxes, "scores": [.6, .9],
                                   "labels": ["fuel cap", "fuel cap"], "image_size_wh": [6, 4]}, backend)
    assert backend.boxes == boxes
    assert prediction.status == "ambiguous" and prediction.selected_index is None
    assert len(prediction.masks) == 2


def test_changed_model_is_rejected_before_loading(tmp_path):
    """来源核对仅用小文件测试，不加载网络或触碰真实最佳权重。"""
    checkpoint = tmp_path / "outputs/train/best_coco_bbox_mAP_epoch_4.pth"
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b"original")
    settings = {"classes": ["fuel cap"]}
    (tmp_path / "cloud_config.json").write_text(json.dumps(settings))
    request = tmp_path / "outputs/evaluate/request.json"
    request.parent.mkdir()
    request.write_text(json.dumps({"settings": settings, "checkpoint_sha256": digest(checkpoint)}))
    checkpoint.write_bytes(b"changed")
    with pytest.raises(ValueError, match="最佳权重"):
        bound_model(tmp_path)


def test_readonly_rgb_is_safe_for_sam_without_changing_original():
    """真实 PIL 输入可只读；后端拿到可写副本，输入照片仍保持原值。"""
    class Backend:
        def predict(self, rgb, prompts):
            assert rgb.flags.writeable
            rgb[0, 0] = 255
            return np.ones(rgb.shape[:2], dtype=bool), .8

    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    rgb.flags.writeable = False
    prediction = segment_detection(rgb, {"boxes_xyxy": [[0, 0, 2, 2]], "scores": [.9],
                                        "labels": ["fuel cap"], "image_size_wh": [6, 4]}, Backend())
    assert not rgb.any()
    assert prediction.status == "unique_candidate"
