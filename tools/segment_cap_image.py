#!/usr/bin/env python3
"""离线调用 Grounding DINO + SAM 2，保存所有候选和来源；不驱动机器人。"""

import argparse
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from fetch_perception_models import digest, DINO_REVISION, SAM_COMMIT, SAM_CONFIG


def save_json(path, value):
    """持久化结果与失败，不覆盖历史目录。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def environment():
    """记录实际包版本，复核时不能只相信 requirements 文件。"""
    return {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()}


def source_files(source):
    """仅绑定 SAM 的源码、配置和构建元信息，不读取示例视频资产。"""
    paths = subprocess.check_output(["git", "-C", str(source), "ls-files", "-z"]).decode().split("\0")
    return {name: digest(source / name) for name in paths
            if name and (name.endswith((".py", ".yaml")) or name in ("LICENSE", "pyproject.toml"))}


def implementation_files():
    """绑定本次实际执行的项目实现与依赖清单。"""
    names = ("tools/segment_cap_image.py", "tools/fetch_perception_models.py",
             "src/fuel_cap_task/grounded_sam.py", "dependencies/perception-requirements.txt")
    return {name: digest(ROOT / name) for name in names}


def verify(output):
    """只读复核输入、代码、模型和产物；不重新运行神经网络。"""
    report = json.loads((output / "result.json").read_text())
    if report["status"] == "runtime_error":
        raise ValueError("failed inference is not a completed result")
    if digest(report["image_path"]) != report["image_sha256"]:
        raise ValueError("image changed")
    if report.get("observation_json") and digest(report["observation_json"]) != report["observation_sha256"]:
        raise ValueError("observation metadata changed")
    if implementation_files() != report["implementation"]:
        raise ValueError("implementation changed since inference")
    if environment() != report["environment"] or sys.version != report["python"]:
        raise ValueError("inference environment changed")
    source = Path(report["sam_source"])
    if source_files(source) != report["sam_source_files"]:
        raise ValueError("SAM source changed")
    models = Path(report["models_directory"])
    if digest(models / "manifest.json") != report["models_manifest_sha256"]:
        raise ValueError("model manifest changed")
    for name, entry in report["models_manifest"]["files"].items():
        if digest(models / name) != entry["sha256"]:
            raise ValueError(f"model file changed: {name}")
    for name, expected in report["outputs"].items():
        if digest(output / name) != expected:
            raise ValueError(f"output changed: {name}")
    print("input, implementation, environment, source, models and output hashes verified")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--image-source", choices=["mujoco_rendered_rgb", "real_camera_rgb", "offline_image"])
    parser.add_argument("--observation-json", type=Path)
    parser.add_argument("--prompt", default="fuel cap.")
    parser.add_argument("--box-threshold", type=float, default=.35)
    parser.add_argument("--text-threshold", type=float, default=.25)
    parser.add_argument("--models-dir", type=Path, default=ROOT / ".cache/perception/models")
    parser.add_argument("--sam-source", type=Path, default=ROOT / "dependencies/sam2")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if args.verify_only:
        verify(output)
        return 0
    if args.image is None or args.image_source is None:
        parser.error("--image and --image-source required for inference")
    if args.image_source == "mujoco_rendered_rgb" and args.observation_json is None:
        parser.error("simulation input requires --observation-json")
    if not args.prompt.strip() or not all(0 < x < 1 for x in (args.box_threshold, args.text_threshold)):
        parser.error("nonempty prompt and thresholds in 0..1 required")
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report = {"status": "runtime_error", "image_path": str(args.image.resolve()),
              "image_source": args.image_source, "prompt": args.prompt,
              "box_threshold": args.box_threshold, "text_threshold": args.text_threshold,
              "device": "cpu", "dtype": "float32", "threads": 4,
              "valid_3d_pose": False, "target_semantics_validated": False,
              "frame_id": None, "timestamp_s": None, "timestamp_clock": "unknown",
              "implementation": implementation_files(), "python": sys.version}
    save_json(output / "result.json", report)
    try:
        # 验证来源后才加载本地模型；预测接口只接受 RGB 和文本。
        models, source = args.models_dir.resolve(), args.sam_source.resolve()
        manifest = json.loads((models / "manifest.json").read_text())
        if (manifest["dino_revision"] != DINO_REVISION
                or manifest["dino_model_id"] != "IDEA-Research/grounding-dino-tiny"
                or manifest["sam_config"] != SAM_CONFIG):
            raise ValueError("model manifest differs from frozen experiment")
        commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(["git", "-C", str(source), "status", "--porcelain",
                                         "--untracked-files=no"], text=True).strip()
        if commit != SAM_COMMIT or commit != manifest["sam_source_commit"] or dirty:
            raise ValueError("SAM checkout must match frozen clean source")
        for name, entry in manifest["files"].items():
            if digest(models / name) != entry["sha256"]:
                raise ValueError(f"model hash mismatch: {name}")
        report.update(image_sha256=digest(args.image), models_directory=str(models),
                      models_manifest=manifest, models_manifest_sha256=digest(models / "manifest.json"),
                      sam_source=str(source), sam_source_files=source_files(source), environment=environment())
        if args.observation_json is not None:
            observation = json.loads(args.observation_json.read_text())
            if observation["files"][args.image.name] != report["image_sha256"]:
                raise ValueError("observation RGB hash mismatch")
            report.update(observation_json=str(args.observation_json.resolve()),
                          observation_sha256=digest(args.observation_json),
                          frame_id=observation["frame_id"], timestamp_s=observation["timestamp_s"],
                          timestamp_clock=observation["timestamp_clock"])
        # editable 安装应导入绑定的 checkout，而非另一套同名包。
        import sam2
        if Path(sam2.__file__).resolve().parent != source / "sam2":
            raise ValueError("imported SAM package differs from bound checkout")
        from PIL import Image, ImageDraw
        import numpy as np
        from fuel_cap_task.grounded_sam import GroundedSAM
        image = Image.open(args.image).convert("RGB")
        rgb = np.array(image)
        predictor = GroundedSAM(models / "grounding-dino-tiny", models / "sam2.1_hiera_tiny.pt", SAM_CONFIG)
        result = predictor.predict(rgb, args.prompt, box_threshold=args.box_threshold,
                                   text_threshold=args.text_threshold)
        np.savez_compressed(output / "candidates.npz", boxes_xyxy=result.boxes_xyxy,
                            detector_scores=result.detector_scores, masks=result.masks,
                            mask_quality_scores=result.mask_quality_scores)
        candidates = []
        overlay = rgb.copy()
        colors = np.array([[0, 210, 180], [240, 80, 160], [160, 150, 20]], dtype=float)
        for i, mask in enumerate(result.masks):
            overlay[mask] = (.55 * overlay[mask] + .45 * colors[i % len(colors)]).astype(np.uint8)
            Image.fromarray(mask.astype(np.uint8) * 255).save(output / f"mask_{i:03d}.png")
            candidates.append({"index": i, "label": result.labels[i],
                               "box_xyxy_px": result.boxes_xyxy[i].tolist(),
                               "detector_text_match_score": float(result.detector_scores[i]),
                               "sam_mask_quality_score": float(result.mask_quality_scores[i]),
                               "mask_pixels": int(mask.sum()), "mask_file": f"mask_{i:03d}.png"})
        picture = Image.fromarray(overlay)
        draw = ImageDraw.Draw(picture)
        for i, box in enumerate(result.boxes_xyxy):
            draw.rectangle(box.tolist(), outline="orange", width=2)
            draw.text((box[0], max(0, box[1] - 15)), f"{i}: {result.labels[i]}", fill="orange")
        draw.text((8, 8), result.status, fill="orange")
        picture.save(output / "overlay.png")
        report.update(status=result.status, selected_index=result.selected_index,
                      candidate_count=len(candidates), candidates=candidates,
                      image_size_wh=list(image.size), elapsed_s=time.perf_counter() - started,
                      score_meanings={"detector": "text matching, not calibrated semantic correctness",
                                      "sam": "predicted mask quality, not detector confidence"},
                      outputs={p.name: digest(p) for p in output.iterdir() if p.name != "result.json"})
        save_json(output / "result.json", report)
        print(json.dumps({"status": result.status, "candidates": candidates,
                          "elapsed_s": report["elapsed_s"]}, ensure_ascii=False))
        return 0 if result.selected_index is not None else 2
    except Exception:
        (output / "failure.txt").write_text(traceback.format_exc())
        report["elapsed_s"] = time.perf_counter() - started
        save_json(output / "result.json", report)
        raise


if __name__ == "__main__":
    sys.exit(main())
