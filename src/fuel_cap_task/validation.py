"""Runtime adapter contracts, independent of physics and calibration."""

from dataclasses import fields
from math import isclose, isfinite, hypot
from numbers import Real

from .interfaces import CapEstimate, Pose, StopFeedback, TaskFailure


def finite_real(value: object) -> bool:
    # bool is a number in Python, but never a measurement in this interface.
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
    for field in fields(expected):
        value = getattr(feedback, field.name)
        if field.name in ("cap_rotation_deg", "axial_progress_m"):
            _require((field.name == "axial_progress_m" and value is None) or finite_real(value),
                     f"{field.name} must be finite (axial progress may be None)")
        elif field.name == "detail":
            _require(isinstance(value, str), "stop detail must be a string")
        else:
            _require(type(value) is bool or (expected is StopFeedback and value is None),
                     f"{field.name} must be boolean" + (" or None" if expected is StopFeedback else ""))
