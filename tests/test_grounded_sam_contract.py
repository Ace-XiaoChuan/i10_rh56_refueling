"""检查零候选/歧义/空掩膜失效与像素合同；不伪造模型推理成功。"""

import numpy as np
import pytest

from fuel_cap_task.grounded_sam import assemble_result


def result(n=1, *, masks=None, boxes=None):
    """仅构造接口输入，验证候选数量不会被静默缩减。"""
    if masks is None:
        masks = np.ones((n, 4, 6), dtype=bool)
    if boxes is None:
        boxes = np.tile([1., 1., 5., 3.], (n, 1))
    return assemble_result(boxes, [.5] * n, ["fuel cap"] * n, masks, [.9] * n, (4, 6))


def test_no_detection_is_invalid():
    prediction = result(0)
    assert prediction.status == "not_detected"
    assert prediction.selected_index is None
    assert prediction.masks.shape == (0, 4, 6)


def test_ambiguous_does_not_select_highest_score():
    prediction = result(2)
    assert prediction.status == "ambiguous"
    assert prediction.selected_index is None
    assert len(prediction.boxes_xyxy) == 2


def test_empty_mask_is_invalid():
    prediction = result(masks=np.zeros((1, 4, 6), dtype=bool))
    assert prediction.status == "empty_mask"
    assert prediction.selected_index is None


def test_unique_candidate_keeps_separate_scores():
    prediction = result()
    assert prediction.status == "unique_candidate"
    assert prediction.selected_index == 0
    assert prediction.detector_scores[0] == .5
    assert prediction.mask_quality_scores[0] == pytest.approx(.9)


@pytest.mark.parametrize("boxes", [[[.1, .1, -.1, 2]], [[0, 0, 7, 3]], [[0, 0, float('nan'), 3]]])
def test_invalid_pixel_boxes_rejected(boxes):
    with pytest.raises(ValueError):
        result(boxes=boxes)


@pytest.mark.parametrize("masks", [np.ones((1, 4, 6), dtype=np.uint8), np.ones((1, 6, 4), dtype=bool)])
def test_non_boolean_or_transposed_mask_rejected(masks):
    with pytest.raises(ValueError):
        result(masks=masks)


def test_index_count_mismatch_rejected():
    with pytest.raises(ValueError):
        assemble_result([[1, 1, 5, 3]], [.5], [], np.ones((1, 4, 6), dtype=bool), [.9], (4, 6))
