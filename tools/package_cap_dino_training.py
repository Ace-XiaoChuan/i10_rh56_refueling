#!/usr/bin/env python3
"""从已验证数据包生成独立云端训练包，固定划分和程序来源。"""

import argparse
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile


PROJECT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("cap_cloud_common", PROJECT / "tools/cloud_cap_dino/common.py")
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-bundle", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    source = args.data_bundle.resolve()
    # 先复核既有数据包；输出拒绝覆盖，不改变旧 ZIP。
    subprocess.run([sys.executable, str(source / "check_transfer.py"), "--root", str(source)], check=True)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    bundle = output / "fuel-cap-training"
    shutil.copytree(source, bundle)
    shutil.copyfile(PROJECT / "tools/check_cap_training_transfer.py", bundle / "check_transfer.py")
    settings = common.read_json(PROJECT / "config/grounding_dino_cloud.json")
    shutil.copyfile(PROJECT / "config/grounding_dino_cloud.json", bundle / "cloud_config.json")
    train, val, smoke = common.make_split(common.read_json(bundle / "annotations/instances_all.json"), settings)
    for name, value in (("train", train), ("val", val), ("smoke", smoke)):
        common.write_json(bundle / "annotations" / f"instances_{name}.json", value)
    transfer = common.read_json(bundle / "transfer_manifest.json")
    image_lookup = {row["image_id"]: row for row in transfer["images"]}
    common.write_json(bundle / "split.json", {
        "source": "user_confirmed_12_train_4_validation", "seed": settings["seed"],
        "train_ids": [row["id"] for row in train["images"]],
        "val_ids": [row["id"] for row in val["images"]],
        "smoke_ids": [row["id"] for row in smoke["images"]],
        "train_images": [{"file_name": row["file_name"], "sha256": image_lookup[row["id"]]["sha256"]}
                         for row in train["images"]],
        "val_images": [{"file_name": row["file_name"], "sha256": image_lookup[row["id"]]["sha256"]}
                       for row in val["images"]],
        "scope": "同一次拍摄、同一场景；没有独立测试集，不代表跨批次泛化",
    })
    transfer["dataset_split"] = "split.json"
    transfer["status"] = "cloud_training_prepared_not_gpu_validated"
    common.write_json(bundle / "transfer_manifest.json", transfer)
    for script in (PROJECT / "tools/cloud_cap_dino").iterdir():
        if script.suffix in (".py", ".sh"):
            shutil.copyfile(script, bundle / script.name)
    shutil.copyfile(PROJECT / "dependencies/grounding-dino-cloud-requirements.txt", bundle / "requirements.txt")
    (bundle / "constraints.txt").write_text("torch==2.1.2\ntorchvision==0.16.2\nnumpy==1.26.4\n", encoding="utf-8")
    vendor_source = PROJECT / ".cache/perception/training_sources/mmdetection_v3.3.0"
    sources = common.read_json(vendor_source / "source_manifest.json")
    for name, record in sources.items():
        if common.digest(vendor_source / name) != record["sha256"]:
            raise ValueError("官方配置来源摘要变化")
    shutil.copytree(vendor_source, bundle / "vendor")
    shutil.copyfile(PROJECT / "doc/cap_dino_cloud_training.md", bundle / "README.md")
    common.write_json(bundle / "checksums.json", {
        path.relative_to(bundle).as_posix(): common.digest(path)
        for path in sorted(bundle.rglob("*")) if path.is_file() and path.name != "checksums.json"})
    subprocess.run([sys.executable, str(bundle / "cloud_workflow.py"), "verify"], check=True)
    archive = output / "fuel-cap-training.zip"
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as packed:
        for path in sorted(bundle.rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts:
                packed.write(path, path.relative_to(output).as_posix())
    common.write_json(output / "archive_summary.json", {"archive": archive.name,
        "sha256": common.digest(archive), "bytes": archive.stat().st_size,
        "training_started": False, "gpu_validated": False})
    print(f"已生成云端准备包：{archive}，{archive.stat().st_size / 1024**2:.2f} MiB。")


if __name__ == "__main__":
    main()
