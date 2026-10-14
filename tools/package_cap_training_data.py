#!/usr/bin/env python3
"""打包已确认的整盖检测数据，保留来源；不划分数据或开始训练。"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import zipfile


def digest(path):
    """分块计算 SHA-256，核对复制前后的数据。"""
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def write_json(path, value):
    """保存 UTF-8 JSON，中文图片名保持原样。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--annotation-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    source = args.annotation_dir.resolve()
    manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
    # 输出拒绝覆盖；原照片、标注及环境始终只读。
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    bundle = output / "fuel-cap-data"
    for name in ("images", "annotations/source_boxes", "metadata"):
        (bundle / name).mkdir(parents=True, exist_ok=False)
    shutil.copyfile(source / "annotations_coco.json", bundle / "annotations/instances_all.json")
    portable_images = []
    for item in manifest["images"]:
        image = project / item["path"]
        if digest(image) != item["sha256"]:
            raise ValueError(f"原图摘要不一致：{image.name}")
        destination = bundle / "images" / image.name
        shutil.copyfile(image, destination)
        portable = {key: item[key] for key in
                    ("image_id", "sha256", "width", "height", "capture_batch", "label")}
        portable["path"] = destination.relative_to(bundle).as_posix()
        if "source_annotation" in item:
            annotation = project / item["source_annotation"]
            if digest(annotation) != item["source_annotation_sha256"]:
                raise ValueError("人工框记录摘要不一致")
            destination = bundle / "annotations/source_boxes" / f"{item['image_id']:04d}.json"
            shutil.copyfile(annotation, destination)
            portable["source_annotation"] = destination.relative_to(bundle).as_posix()
        portable_images.append(portable)
    for name in ("manifest.json", "annotation_audit.json", "user_confirmation.json",
                 "capture_batch_confirmation.json", "export_validation.json", "report.md"):
        shutil.copyfile(source / name, bundle / "metadata" / name)
    # 来源记录保留项目历史路径；实际训练和核查使用便携清单中的相对路径。
    write_json(bundle / "transfer_manifest.json", {
        "schema": "cap_training_transfer_v1", "date": "2026-10-10",
        "status": "data_only_unsplit_not_trained", "image_root": "images",
        "annotation_file": "annotations/instances_all.json", "images": portable_images,
        "excluded_images": manifest["excluded_images"], "dataset_split": None,
        "training_started": False,
        "metadata_scope": "metadata/ 为历史证据，内部原项目路径不作为借用电脑上的数据路径",
    })
    models = json.loads((project / ".cache/perception/models/manifest.json").read_text(encoding="utf-8"))
    write_json(bundle / "model_reference.json", {
        "model_id": models["dino_model_id"], "revision": models["dino_revision"],
        "files": {name: record for name, record in models["files"].items()
                  if name.startswith("grounding-dino-tiny/")},
        "weights_included": False,
        "meaning": "当前推理预训练权重的来源记录；MMDetection 训练权重格式兼容仍待核对",
    })
    shutil.copyfile(project / "tools/check_cap_training_transfer.py", bundle / "check_transfer.py")
    shutil.copyfile(project / "doc/cap_training_data.md", bundle / "README.md")
    gallery = (source / "index.html").read_text(encoding="utf-8")
    (bundle / "index.html").write_text(gallery.replace("../../../pic/", "images/"), encoding="utf-8")
    files = sorted(path for path in bundle.rglob("*") if path.is_file())
    write_json(bundle / "checksums.json", {path.relative_to(bundle).as_posix(): digest(path)
                                          for path in files})
    archive = output / "fuel-cap-data.zip"
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as packed:
        for path in sorted(bundle.rglob("*")):
            if path.is_file():
                packed.write(path, path.relative_to(output).as_posix())
    write_json(output / "archive_summary.json", {
        "archive": archive.name, "sha256": digest(archive), "bytes": archive.stat().st_size,
        "images": len(portable_images), "annotation_source": str(source),
        "training_started": False,
    })
    print(f"已打包：{archive}；大小 {archive.stat().st_size / 1024**2:.2f} MiB。")


if __name__ == "__main__":
    main()
