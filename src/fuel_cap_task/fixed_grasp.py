"""读取固定抓姿候选及来源，不计算空间目标或发送运动指令。

支持现有 manual_grasp_candidate 格式 v1：位置单位为米，四元数顺序
为 XYZW，cap_from_hand 将手根 base 的坐标转换到盖体系 cap_grip_center。
盖体系 X 沿握梁、Z 朝开口外侧；额外的接触实验记录保留在原 JSON 中。
"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re

from .interfaces import Pose


DEFAULT_FIXED_GRASP = Path(__file__).resolve().parents[2] / "config/fixed_grasp.json"


@dataclass(frozen=True)
class GraspProvenance:
    """记录本次读取的文件摘要及原候选来源；场景摘要仅覆盖该 XML 文件。"""

    candidate_path: str
    candidate_sha256: str
    schema_version: int
    mode: str
    recorded_source_scene: str
    source_scene_path: str
    source_scene_sha256: str
    mujoco_version: str
    selection_source: str | None
    selection_status: str | None


@dataclass(frozen=True)
class FixedGrasp:
    """经校验的固定抓姿；六通道依次为小指、无名指、中指、食指、拇指弯曲、拇指旋转。

    cap_from_hand 为手根在盖体系中的位姿。通道采用完整的 0～1000 目标值，
    不接受依赖当前状态的 -1 保持指令；这些值不是关节角。
    """

    cap_from_hand: Pose
    pregrasp_channels: tuple[int, int, int, int, int, int]
    grasp_channels: tuple[int, int, int, int, int, int]
    provenance: GraspProvenance


def _text(value: object, name: str) -> str:
    """要求来源字段为非空字符串，不改写其原始内容。"""
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} 必须是非空字符串")
    return value


def _optional_text(record: dict, name: str) -> str | None:
    """允许旧候选缺少选取说明；存在时仍校验格式。"""
    value = record.get(name)
    return None if value is None else _text(value, name)


def _vector(value: object, size: int, name: str) -> tuple[float, ...]:
    """读取有限数值向量，拒绝布尔值、字符串和溢出数值。"""
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} 必须包含 {size} 个有限数值")
    result = []
    for item in value:
        if type(item) not in (int, float):
            raise ValueError(f"{name} 必须包含 {size} 个有限数值")
        try:
            number = float(item)
        except OverflowError as exc:
            raise ValueError(f"{name} 必须包含 {size} 个有限数值") from exc
        if not math.isfinite(number):
            raise ValueError(f"{name} 必须包含 {size} 个有限数值")
        result.append(number)
    return tuple(result)


def _channels(value: object, name: str) -> tuple[int, int, int, int, int, int]:
    """固定候选必须明确指定六个通道，不截断小数或裁剪越界指令。"""
    if (not isinstance(value, list) or len(value) != 6
            or any(type(item) is not int or not 0 <= item <= 1000 for item in value)):
        raise ValueError(f"{name} 必须是六个 0～1000 的整数，顺序为小指到拇指旋转")
    return tuple(value)


def load_fixed_grasp(
    candidate_path: str | Path,
    *,
    source_scene_path: str | Path | None = None,
    expected_mujoco_version: str | None = None,
) -> FixedGrasp:
    """从候选 JSON 读取位姿、通道和可追溯来源，返回不可变记录。

    candidate_path 为必填路径，相对路径按当前工作目录解释。默认检查候选
    source_scene 指向的 XML；其中的相对路径按候选所在目录解释。仓库迁移后
    可用 source_scene_path 指定新位置，但内容摘要仍须与候选一致。
    expected_mujoco_version 可选，由调用方传当前运行版本；省略时仅记录
    候选版本，不声明运行时兼容。此函数只依赖标准库和项目的 Pose 类型。

    文件不存在时抛出 FileNotFoundError，格式、数值或来源不匹配时抛出
    ValueError。读取成功不表示已经到位、建立抓握或完成运动。
    """
    path = Path(candidate_path).expanduser().resolve()
    # 摘要和解析使用同一份字节，保证来源记录对应真正读取的参数。
    raw = path.read_bytes()
    try:
        record = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(f"候选文件不是有效 JSON：{path}") from exc
    if not isinstance(record, dict):
        raise ValueError("候选 JSON 顶层必须是对象")
    if type(record.get("schema_version")) is not int or record["schema_version"] != 1:
        raise ValueError("schema_version 必须是整数 1")
    if record.get("mode") != "manual_grasp_candidate":
        raise ValueError("mode 必须是 manual_grasp_candidate")

    # 格式 v1 的坐标含义固定，不能仅凭三个数猜测其坐标系或单位。
    pose = record.get("cap_from_hand")
    if not isinstance(pose, dict):
        raise ValueError("cap_from_hand 必须是位姿对象")
    position = _vector(pose.get("position_m"), 3, "cap_from_hand.position_m")
    orientation = _vector(pose.get("orientation_xyzw"), 4, "cap_from_hand.orientation_xyzw")
    if not math.isclose(math.hypot(*orientation), 1.0, rel_tol=0, abs_tol=1e-6):
        raise ValueError("cap_from_hand.orientation_xyzw 必须是单位四元数")
    # 若后续文件显式写了坐标系，只允许与 v1 约定一致的值。
    if "frame_id" in pose and pose["frame_id"] != "cap_grip_center":
        raise ValueError("cap_from_hand.frame_id 必须是 cap_grip_center")
    pregrasp = _channels(record.get("pregrasp_channels"), "pregrasp_channels")
    grasp = _channels(record.get("grasp_channels"), "grasp_channels")

    recorded_scene = _text(record.get("source_scene"), "source_scene")
    expected_hash = _text(record.get("source_scene_sha256"), "source_scene_sha256").lower()
    if re.fullmatch(r"[0-9a-f]{64}", expected_hash) is None:
        raise ValueError("source_scene_sha256 必须是 64 位十六进制摘要")
    version = _text(record.get("mujoco_version"), "mujoco_version")
    if expected_mujoco_version is not None:
        _text(expected_mujoco_version, "expected_mujoco_version")
        if version != expected_mujoco_version:
            raise ValueError(f"MuJoCo 版本不匹配：候选为 {version}，当前为 {expected_mujoco_version}")

    # 允许明确迁移路径，保留原记录；模型内容改变时仍拒绝沿用旧来源。
    scene = Path(source_scene_path if source_scene_path is not None else recorded_scene).expanduser()
    if source_scene_path is None and not scene.is_absolute():
        scene = path.parent / scene
    scene = scene.resolve()
    actual_hash = hashlib.sha256(scene.read_bytes()).hexdigest()
    if actual_hash != expected_hash:
        raise ValueError(f"源场景 SHA-256 不匹配，需重新核对候选适配：{scene}")

    provenance = GraspProvenance(
        candidate_path=str(path),
        candidate_sha256=hashlib.sha256(raw).hexdigest(),
        schema_version=record["schema_version"],
        mode=record["mode"],
        recorded_source_scene=recorded_scene,
        source_scene_path=str(scene),
        source_scene_sha256=actual_hash,
        mujoco_version=version,
        selection_source=_optional_text(record, "selection_source"),
        selection_status=_optional_text(record, "selection_status"),
    )
    return FixedGrasp(Pose("cap_grip_center", position, orientation), pregrasp, grasp, provenance)
