"""人工空间提示的离线诊断；绕过检测器，不提供语义或三维验收结论。"""

from dataclasses import dataclass
import hashlib
import html
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np
from PIL import Image, ImageDraw


def digest(path):
    """分块记录输入、权重和产物摘要。"""
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def write_json(path, value):
    """新记录拒绝覆盖；派生浏览页另外生成。"""
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


@dataclass(frozen=True)
class Viewport:
    """等比显示后的整数像素尺寸；分别换算两轴，消除缩放取整误差。"""

    image_wh: tuple
    display_wh: tuple
    offset_xy: tuple

    @classmethod
    def fit(cls, image_wh, canvas_wh):
        width, height = image_wh
        cw, ch = canvas_wh
        scale = min(max(1, cw) / width, max(1, ch) / height)
        dw, dh = max(1, round(width * scale)), max(1, round(height * scale))
        return cls(tuple(image_wh), (dw, dh), ((cw - dw) / 2, (ch - dh) / 2))

    def to_image(self, x, y, *, clamp=False):
        """空白边缘不产生点；拖框到画幅外时可截断到有效像素中心。"""
        ox, oy = self.offset_xy
        dw, dh = self.display_wh
        width, height = self.image_wh
        if not clamp and not (ox <= x < ox + dw and oy <= y < oy + dh):
            return None
        return [min(width - 1, max(0., (x - ox) * width / dw)),
                min(height - 1, max(0., (y - oy) * height / dh))]

    def to_canvas(self, x, y):
        """提示始终保存在原图坐标，只在绘制时转成画布坐标。"""
        width, height = self.image_wh
        dw, dh = self.display_wh
        ox, oy = self.offset_xy
        return [ox + x * dw / width, oy + y * dh / height]


def validate_prompts(value, image):
    """检查照片绑定、原图 XYXY 框及点标签，禁止交换图片后沿用旧提示。"""
    if value.get("schema") != "sam_manual_prompts_v1":
        raise ValueError("提示文件 schema 不支持")
    for key in ("image_sha256", "image_size_wh"):
        if value.get(key) != image[key]:
            raise ValueError(f"提示与照片不匹配：{key}")
    width, height = image["image_size_wh"]
    box = np.asarray(value.get("box_xyxy_px"), dtype=float)
    if (box.shape != (4,) or not np.isfinite(box).all()
            or not (0 <= box[0] < box[2] <= width - 1
                    and 0 <= box[1] < box[3] <= height - 1)):
        raise ValueError("框须为原图有效像素坐标 [x0,y0,x1,y1]，且具有正面积")
    points = np.asarray(value.get("point_coords_px", []), dtype=float)
    labels = np.asarray(value.get("point_labels", []))
    if points.size == 0:
        points = np.empty((0, 2))
    if (points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all()
            or labels.shape != (len(points),) or not np.isin(labels, [0, 1]).all()
            or (points < 0).any() or (points >= [width, height]).any()):
        raise ValueError("点须为原图 [x,y]，标签 1 为正点、0 为负点")
    if not isinstance(value.get("annotation_source"), str) or not value["annotation_source"].strip():
        raise ValueError("必须说明提示来源")
    return box, points, labels.astype(np.int32)


def used_prompts(value, mode, image):
    """B 组显式忽略全部点；C 组必须同时给框和至少一个点。"""
    box, points, labels = validate_prompts(value, image)
    if mode not in ("box", "box_points"):
        raise ValueError("诊断模式须为 box 或 box_points")
    if mode == "box_points" and not len(points):
        raise ValueError("C 组需要至少一个正点或负点")
    return {"box_xyxy_px": box.tolist(),
            "point_coords_px": points.tolist() if mode == "box_points" else [],
            "point_labels": labels.tolist() if mode == "box_points" else []}


def overlay(rgb, mask, prompts):
    """仅在展示副本上叠色与画提示；模型一直使用原始 RGB。"""
    pixels = np.array(rgb, copy=True)
    if mask is not None:
        pixels[mask] = (.55 * pixels[mask] + .45 * np.array([0, 210, 180])).astype(np.uint8)
    picture = Image.fromarray(pixels)
    draw = ImageDraw.Draw(picture)
    draw.rectangle(prompts["box_xyxy_px"], outline="orange", width=3)
    for (x, y), label in zip(prompts["point_coords_px"], prompts["point_labels"]):
        color = "lime" if label else "red"
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color, outline="white", width=1)
    return picture


class SAMBackend:
    """仅加载已绑定的 SAM 2；设置与现有自动实验一致，不加载 Grounding DINO。"""

    def __init__(self, models, source, expected_commit, expected_config):
        self.models, self.source = Path(models).resolve(), Path(source).resolve()
        manifest_path = self.models / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        commit = subprocess.check_output(["git", "-C", str(self.source), "rev-parse", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(["git", "-C", str(self.source), "status", "--porcelain", "--untracked-files=no"], text=True).strip()
        if (commit != expected_commit or commit != manifest["sam_source_commit"]
                or manifest["sam_config"] != expected_config or dirty):
            raise ValueError("SAM 源码或配置不符合冻结版本")
        checkpoint = self.models / "sam2.1_hiera_tiny.pt"
        if digest(checkpoint) != manifest["files"][checkpoint.name]["sha256"]:
            raise ValueError("SAM 权重摘要不匹配")
        names = subprocess.check_output(["git", "-C", str(self.source), "ls-files", "-z"]).decode().split("\0")
        source_hashes = {name: digest(self.source / name) for name in names
                         if name and (name.endswith((".py", ".yaml")) or name in ("LICENSE", "pyproject.toml"))}
        self.provenance = {
            "models_manifest_path": str(manifest_path), "models_manifest_sha256": digest(manifest_path),
            "checkpoint_path": str(checkpoint), "checkpoint_sha256": digest(checkpoint),
            "sam_source": str(self.source), "sam_commit": commit, "sam_config": expected_config,
            "sam_source_files": source_hashes, "device": "cpu", "dtype": "float32", "threads": 4,
            "apply_postprocessing": False, "multimask_output": False,
            "python": sys.version,
            "environment": {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()},
        }
        import sam2
        if Path(sam2.__file__).resolve().parent != self.source / "sam2":
            raise ValueError("实际导入的 SAM 路径不匹配")
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        torch.set_num_threads(4)
        self.torch = torch
        self.predictor = SAM2ImagePredictor(build_sam2(
            expected_config, str(checkpoint), device="cpu", mode="eval", apply_postprocessing=False))
        self.image_key = None

    def predict(self, rgb, prompts):
        """缓存同一原图特征，仅改变空间提示；输出一张原图大小的布尔掩膜。"""
        key = hashlib.sha256(rgb.tobytes()).hexdigest()
        with self.torch.inference_mode():
            if key != self.image_key:
                self.predictor.set_image(rgb)
                self.image_key = key
            points = np.asarray(prompts["point_coords_px"], dtype=np.float32).reshape(-1, 2)
            masks, quality, _ = self.predictor.predict(
                box=np.asarray(prompts["box_xyxy_px"], dtype=np.float32),
                point_coords=points if len(points) else None,
                point_labels=np.asarray(prompts["point_labels"], dtype=np.int32) if len(points) else None,
                multimask_output=False, return_logits=False)
        mask = np.asarray(masks, dtype=bool).reshape(rgb.shape[:2])
        score = float(np.asarray(quality).reshape(-1)[0])
        if not np.isfinite(score):
            raise ValueError("SAM 返回非有限质量分数")
        return mask, score


class DiagnosticSession:
    """会话保存不可变输入和逐次实验；历史自动结果仅导入为对照图。"""

    def __init__(self, image_path, output, root, reference=None):
        self.root, self.output = Path(root).resolve(), Path(output).resolve()
        path = Path(image_path).resolve()
        picture = Image.open(path).convert("RGB")
        self.rgb = np.asarray(picture).copy()
        self.image = {"image_path": str(path), "image_sha256": digest(path),
                      "image_size_wh": list(picture.size)}
        self.reference = self._read_reference(reference) if reference else None
        self.output.mkdir(parents=True, exist_ok=False)
        self.input_path = self.output / ("source" + path.suffix)
        shutil.copyfile(path, self.input_path)
        if digest(self.input_path) != self.image["image_sha256"]:
            raise ValueError("保存照片期间输入发生变化")
        (self.output / "annotations").mkdir()
        (self.output / "trials").mkdir()
        self.annotation_index = self.trial_index = 0
        self.implementation = {str(self.root / name): digest(self.root / name) for name in (
            "tools/diagnose_sam_image.py", "tools/fetch_perception_models.py",
            "src/fuel_cap_task/sam_diagnostic.py", "src/fuel_cap_task/sam_diagnostic_gui.py",
            "dependencies/perception-requirements.txt")}
        self.reference_picture = None
        if self.reference:
            shutil.copyfile(self.reference["overlay_path"], self.output / "automatic_overlay.png")
            self.reference_picture = Image.open(self.output / "automatic_overlay.png").convert("RGB")
        write_json(self.output / "session.json", {
            "schema": "sam_diagnostic_session_v1", **self.image,
            "source_copy": self.input_path.name, "automatic_reference": self.reference,
            "implementation": self.implementation, "valid_3d_pose": False,
            "target_semantics_validated": False})
        self.build_index()

    def _read_reference(self, reference):
        """核对同图对照与展示产物摘要，不将旧结果冒充重新推理。"""
        path = Path(reference).resolve()
        value = json.loads(path.read_text())
        if value.get("image_sha256") != self.image["image_sha256"]:
            raise ValueError("自动对照与当前照片不匹配")
        expected = value.get("outputs", {}).get("overlay.png")
        overlay_path = path.parent / "overlay.png"
        if not expected or digest(overlay_path) != expected:
            raise ValueError("自动对照叠加图摘要不匹配")
        if value.get("status") == "runtime_error":
            raise ValueError("自动对照不是完成的推理")
        if Image.open(overlay_path).size != tuple(self.image["image_size_wh"]):
            raise ValueError("自动对照图尺寸不匹配")
        return {"result_path": str(path), "result_sha256": digest(path),
                "overlay_path": str(overlay_path), "overlay_sha256": expected,
                "status": value["status"], "candidate_count": value.get("candidate_count"),
                "verification_scope": "same_image_and_display_output_hashes_only"}

    def save_annotation(self, prompts):
        """保存框和全部点；B/C 实际使用的提示另在实验中冻结。"""
        validate_prompts(prompts, self.image)
        self.annotation_index += 1
        path = self.output / "annotations" / f"{self.annotation_index:04d}.json"
        write_json(path, prompts)
        return path

    def run(self, prompts, mode, backend_factory):
        """先冻结提示，再运行模型；失败也保留记录，不覆盖或自动重试。"""
        used = used_prompts(prompts, mode, self.image)
        annotation = self.save_annotation(prompts)
        self.trial_index += 1
        folder = self.output / "trials" / f"{self.trial_index:04d}_{mode}"
        folder.mkdir()
        report = {"schema": "sam_manual_diagnostic_v1", "status": "runtime_error",
                  "input_path": str(self.input_path), **self.image, "mode": mode,
                  "annotation_path": str(annotation), "annotation_sha256": digest(annotation),
                  "annotation_source": prompts["annotation_source"], "used_prompts": used,
                  "implementation": self.implementation, "valid_3d_pose": False,
                  "target_semantics_validated": False,
                  "score_meaning": "predicted_mask_quality_not_semantic_correctness"}
        write_json(folder / "request.json", report)
        started = time.perf_counter()
        try:
            backend = backend_factory()
            report["model"] = backend.provenance
            mask, score = backend.predict(self.rgb, used)
            Image.fromarray(mask.astype(np.uint8) * 255).save(folder / "mask.png")
            picture = overlay(self.rgb, mask, used)
            picture.save(folder / "overlay.png")
            report.update(status="mask_predicted" if mask.any() else "empty_mask",
                          sam_mask_quality_score=score, mask_pixels=int(mask.sum()))
        except Exception:
            (folder / "failure.txt").write_text(traceback.format_exc(), encoding="utf-8")
        report["elapsed_s"] = time.perf_counter() - started
        report["outputs"] = {path.name: digest(path) for path in folder.iterdir()}
        write_json(folder / "result.json", report)
        self.build_index()
        if report["status"] == "runtime_error":
            raise RuntimeError(f"推理失败，实际异常见 {folder / 'failure.txt'}")
        return report, picture, folder

    def build_index(self):
        """生成派生对照页；逐次实验和照片副本不可变。"""
        sections = ['<h1>SAM 人工提示诊断</h1><p>人工提示对照，不代表自动定位或语义验收通过。</p>',
                    f'<h2>原图</h2><img src="{html.escape(self.input_path.name)}">']
        if self.reference:
            sections.append('<h2>A：历史自动结果</h2><img src="automatic_overlay.png">')
        for path in sorted((self.output / "trials").glob("*/result.json")):
            report = json.loads(path.read_text())
            relative = path.parent.relative_to(self.output).as_posix()
            group = "B：仅人工框" if report["mode"] == "box" else "C：人工框＋点"
            sections.append(f'<h2>{path.parent.name} · {group}</h2><p>{report["status"]} · '
                            f'<a href="{relative}/result.json">提示与结果</a></p>')
            if report["status"] != "runtime_error":
                sections.append(f'<img src="{relative}/overlay.png">')
        (self.output / "index.html").write_text(
            '<!doctype html><html lang="zh"><meta charset="utf-8"><title>SAM 诊断</title>'
            '<style>body{max-width:1100px;margin:24px auto;font-family:sans-serif;padding:12px}'
            'img{max-width:100%;border:1px solid #bbb}a{color:#1769aa}</style>'
            + "\n".join(sections) + "</html>", encoding="utf-8")


def verify_session(output):
    """只读复核照片副本、提示、实现、模型与实验产物；不加载模型。"""
    output = Path(output).resolve()
    session = json.loads((output / "session.json").read_text())
    if digest(output / session["source_copy"]) != session["image_sha256"]:
        raise ValueError("照片副本已改变")
    for path, expected in session["implementation"].items():
        if digest(path) != expected:
            raise ValueError(f"诊断实现已改变：{path}")
    reference = session["automatic_reference"]
    if reference and digest(output / "automatic_overlay.png") != reference["overlay_sha256"]:
        raise ValueError("自动对照副本已改变")
    for annotation in (output / "annotations").glob("*.json"):
        validate_prompts(json.loads(annotation.read_text()), session)
    reports = sorted((output / "trials").glob("*/result.json"))
    for path in reports:
        report = json.loads(path.read_text())
        if report["status"] == "runtime_error":
            raise ValueError(f"会话包含未完成推理：{path}")
        annotation = Path(report["annotation_path"])
        if digest(annotation) != report["annotation_sha256"]:
            raise ValueError("实验提示已改变")
        if used_prompts(json.loads(annotation.read_text()), report["mode"], session) != report["used_prompts"]:
            raise ValueError("实验使用的提示不匹配")
        for name, expected in report["outputs"].items():
            if digest(path.parent / name) != expected:
                raise ValueError(f"实验产物已改变：{name}")
        model = report["model"]
        for key in ("checkpoint", "models_manifest"):
            if digest(model[key + "_path"]) != model[key + "_sha256"]:
                raise ValueError(f"{key} 已改变")
        for name, expected in model["sam_source_files"].items():
            if digest(Path(model["sam_source"]) / name) != expected:
                raise ValueError(f"SAM 源码已改变：{name}")
        environment = {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()}
        if sys.version != model["python"] or environment != model["environment"]:
            raise ValueError("当前环境与推理记录不匹配")
    return len(reports)
