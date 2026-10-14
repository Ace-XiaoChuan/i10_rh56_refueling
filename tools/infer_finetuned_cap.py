#!/usr/bin/env python3
"""独立检测进程：加载一次最佳模型，对新输入图片真实推理并保存来源。"""

import argparse
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--image", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # 检测结果文件不得覆盖；失败也保留此次真实运行的异常。
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump({"status": "running"}, stream)
    started = time.perf_counter()
    result = {"status": "runtime_error", "images": [], "valid_3d_pose": False}
    try:
        import numpy as np
        from PIL import Image
        from fuel_cap_task.finetuned_dino import FinetunedDINO, digest

        detector = FinetunedDINO(args.bundle)
        result["detector"] = detector.provenance
        result["implementation"] = {name: digest(ROOT / name) for name in (
            "tools/infer_finetuned_cap.py", "src/fuel_cap_task/finetuned_dino.py",
            "tools/cap_dino_runtime/requirements.txt")}
        for path in args.image:
            before = digest(path)
            rgb = np.asarray(Image.open(path).convert("RGB"))
            prediction = detector.predict(rgb)
            if digest(path) != before:
                raise ValueError("推理过程中输入图片发生变化")
            prediction.update(image_path=str(path.resolve()), image_sha256=before)
            result["images"].append(prediction)
            print(path.name, "候选数：", len(prediction["scores"]), flush=True)
        result["status"] = "completed"
    except Exception:
        result["error"] = traceback.format_exc()
        raise
    finally:
        result["elapsed_seconds"] = time.perf_counter() - started
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                               encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
