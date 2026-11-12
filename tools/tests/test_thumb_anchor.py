"""自动选点须随握梁变换，并按四指排列避开错误端部。"""
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from fuel_cap_task.thumb_anchor import rank_anchors, anchored_pose, ContactSequence, GripShape


def test_right_hand_selects_end_with_room_for_other_fingers():
    quality = np.zeros((80, 80, 80))
    # 网络体素原点在握梁左下方，两个候选关于中心对称。
    frame = np.eye(4)
    frame[:3, 3] = [.04, .04, .04]
    quality[20, 49, 37] = .95
    quality[60, 49, 37] = .45
    rows = rank_anchors(quality, .001, frame, [-.045, -.028, -.011, .009])
    assert rows[0]['feature_index_xyz'] == [60, 49, 37]
    assert rows[0]['anchor_grip_m'][0] > 0
    assert rows[0]['finger_overflow_m'] == 0


def test_anchor_and_hand_pose_follow_observed_grip_rigid_motion():
    move = np.eye(4)
    move[:3, :3] = Rotation.from_euler('xyz', [.2, -.4, .8]).as_matrix()
    move[:3, 3] = [.3, -.2, .7]
    rotation = Rotation.from_euler('x', .3).as_matrix()
    pad, anchor = np.array([.01, .02, .03]), np.array([.02, .009, -.003])
    first = anchored_pose(rotation, pad, anchor)
    second = anchored_pose(move[:3, :3] @ rotation, pad,
                           move[:3, :3] @ anchor + move[:3, 3])
    np.testing.assert_allclose(second, move @ first, atol=1e-12)
    np.testing.assert_allclose(first[:3, :3] @ pad + first[:3, 3], anchor)


def test_no_network_features_returns_no_anchor_and_invalid_input_fails():
    assert rank_anchors(np.zeros((4, 4, 4)), .001, np.eye(4), [-.04, -.02, 0, .01]) == []
    with pytest.raises(ValueError):
        rank_anchors(np.full((4, 4, 4), np.nan), .001, np.eye(4), [-.04, -.02, 0, .01])


def test_opposing_fingers_wait_for_thumb_contact_and_hold_requires_continuity():
    seq = ContactSequence()
    assert seq.update(0., False, False) == 'seek_thumb'
    assert seq.update(.1, True, False) == 'close_fingers'
    assert seq.update(.2, True, True) == 'hold'
    assert seq.update(.3, True, True) == 'hold'
    assert seq.update(.41, True, True) == 'done'
    lost = ContactSequence()
    lost.update(0., False, False)
    lost.update(.1, True, False)
    lost.update(.2, True, True)
    assert lost.update(.25, False, False) == 'failed'


def test_thumb_timeout_returns_failure_instead_of_closing_blindly():
    seq = ContactSequence(thumb_timeout_s=.5)
    seq.update(0., False, False)
    assert seq.update(.51, False, False) == 'failed'


def test_losing_opposition_at_end_of_hold_does_not_report_success():
    seq = ContactSequence()
    seq.update(.1, True, False)
    seq.update(.2, True, True)
    assert seq.update(.41, True, False) == 'failed'


def test_side_contact_near_top_edge_counts_but_contact_on_top_does_not():
    shape = GripShape()
    point = [.024, .0085, -1.6e-7]
    assert shape.lateral_side(point, [0., -.97, -.24]) == 1
    assert shape.lateral_side(point, [0., 0., -1.]) is None


def test_small_finger_overflow_does_not_force_a_deep_anchor():
    quality = np.zeros((80, 80, 80))
    frame = np.eye(4)
    frame[:3, 3] = [.04, .04, .04]
    quality[58, 49, 27] = .7
    quality[62, 49, 38] = .45
    rows = rank_anchors(quality, .001, frame, [-.045, -.028, -.011, .009])
    assert rows[0]['feature_index_xyz'] == [62, 49, 38]
