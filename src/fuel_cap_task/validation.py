"""校验适配器回执的表示和语义，不代替物理判据或标定。"""

from dataclasses import fields
from math import isclose, isfinite, hypot
from numbers import Real

from .interfaces import CapEstimate, Pose, StopFeedback, TaskFailure
from .execution_contract import (
    Feedback, FeedbackContext, MotionProgress, SupportFeedback,
    TwistEndReason, TwistFeedback,
)


def finite_real(value: object) -> bool:
    # Python 中 bool 属于数字，但接口不能把确认标志当作测量值。
    return isinstance(value, Real) and not isinstance(value, bool) and isfinite(value)


def _require(condition: bool, detail: str) -> None:
    if not condition:
        raise TaskFailure(f"interface contract: {detail}")


def _vector(value: object, size: int, name: str) -> None:
    _require(isinstance(value, (tuple, list)) and len(value) == size
             and all(finite_real(item) for item in value),
             f"{name} must contain {size} finite real numbers")


def validate_estimate(cap: CapEstimate) -> None:
    _require(isinstance(cap, CapEstimate), "expected CapEstimate")
    _require(cap.valid is True, "cap estimate is invalid")
    _require(isinstance(cap.pose, Pose), "expected Pose")
    for name, value in (("frame_id", cap.pose.frame_id), ("source", cap.source),
                        ("timestamp_clock", cap.timestamp_clock)):
        _require(isinstance(value, str) and bool(value.strip()), f"{name} must be nonempty")
    _vector(cap.pose.position_m, 3, "position_m")
    _vector(cap.pose.orientation_xyzw, 4, "orientation_xyzw")
    _require(isclose(hypot(*cap.pose.orientation_xyzw), 1.0, rel_tol=0, abs_tol=1e-6),
             "orientation_xyzw must be a unit quaternion")
    _require(finite_real(cap.timestamp_s), "timestamp_s must be finite")
    _require(cap.confidence is None or (finite_real(cap.confidence) and 0 <= cap.confidence <= 1),
             "confidence must be None or a finite value in [0, 1]")


def validate_feedback(feedback: object, expected: type) -> None:
    _require(isinstance(feedback, expected), f"expected {expected.__name__}")
    if expected is StopFeedback:
        _require(type(feedback.motion_stopped) is bool or feedback.motion_stopped is None,
                 "motion_stopped must be boolean or None")
        _require(type(feedback.grasp_held) is bool or feedback.grasp_held is None,
                 "grasp_held must be boolean or None")
        _require(isinstance(feedback.detail, str), "stop detail must be a string")
        return

    # 新回执必须显式提供来源；不替旧回执补造默认成功或来源。
    _require(isinstance(feedback, Feedback), "expected sourced execution feedback")
    context = feedback.context
    _require(isinstance(context, FeedbackContext), "expected FeedbackContext")
    _require(context.valid is True, "execution feedback is invalid")
    for name in ("source", "timestamp_clock"):
        value = getattr(context, name)
        _require(isinstance(value, str) and bool(value.strip()), f"{name} must be nonempty")
    _require(finite_real(context.timestamp_s), "feedback timestamp must be finite")
    _require(isinstance(context.detail, str), "feedback detail must be a string")
    for field in fields(expected):
        value = getattr(feedback, field.name)
        if field.name == "context":
            continue
        if field.name == "progress":
            validate_progress(value)
        elif field.name == "cap":
            validate_estimate(value)
        elif field.name == "end_reason":
            _require(isinstance(value, TwistEndReason), "expected TwistEndReason")
        else:
            _require(type(value) is bool or value is None, f"{field.name} must be boolean or None")

    # 矛盾结论必须拒绝，不能按某个看似成功的字段选分支。
    if isinstance(feedback, TwistFeedback):
        if feedback.end_reason is TwistEndReason.DETACHED:
            _require(feedback.thread_released is True, "detached requires confirmed release")
        elif feedback.end_reason is TwistEndReason.SEGMENT_BOUNDARY:
            _require(feedback.thread_released is False, "segment boundary requires unreleased cap")
    if isinstance(feedback, SupportFeedback) and feedback.release_allowed is True:
        _require(feedback.thread_released is False, "release permission requires unreleased cap")
        _require(bool(context.detail.strip()), "release permission requires decision evidence")


def validate_progress(progress: object) -> None:
    """累计旋转沿合同旋松方向；轴向进度可未知，不补成零。"""
    _require(isinstance(progress, MotionProgress), "expected MotionProgress")
    _require(finite_real(progress.cap_rotation_deg) and progress.cap_rotation_deg >= 0,
             "cap rotation must be finite and nonnegative")
    _require(progress.axial_progress_m is None or finite_real(progress.axial_progress_m),
             "axial progress must be finite or None")
