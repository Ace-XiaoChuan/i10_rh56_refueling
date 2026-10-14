#!/usr/bin/env python3
"""交互框选或载入原图提示，独立诊断 SAM 2；不改变自动定位接口。"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from fetch_perception_models import SAM_COMMIT, SAM_CONFIG
from fuel_cap_task.sam_diagnostic import DiagnosticSession, SAMBackend, verify_session

DEFAULT_IMAGE = ROOT / "assets/hardware_photos/fuel_tank/be44de650d565ff38af7dd108c384037.jpg"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--prompts", type=Path, help="载入与照片摘要绑定的提示 JSON")
    parser.add_argument("--automatic-result", type=Path, help="同图历史自动结果的 result.json，仅作 A 组对照")
    parser.add_argument("--models-dir", type=Path, default=ROOT / ".cache/perception/models")
    parser.add_argument("--sam-source", type=Path, default=ROOT / "dependencies/sam2")
    parser.add_argument("--headless", action="store_true", help="不打开窗口，使用已保存提示运行一个实验")
    parser.add_argument("--mode", choices=("box", "box_points"), default="box")
    parser.add_argument("--verify-only", action="store_true", help="只读复核已有会话，不进行推理")
    args = parser.parse_args()
    if args.verify_only:
        if args.output_dir is None:
            parser.error("--verify-only 需要 --output-dir")
        count = verify_session(args.output_dir)
        print(f"输入、提示、实现与产物复核通过；完成实验数：{count}")
        return 0
    if args.headless and args.prompts is None:
        parser.error("--headless 需要 --prompts")
    if args.output_dir is None:
        args.output_dir = ROOT / "artifacts/perception/sam_manual" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    prompts = json.loads(args.prompts.read_text()) if args.prompts else None
    session = DiagnosticSession(args.image, args.output_dir, ROOT, args.automatic_result)
    if prompts is not None:
        from fuel_cap_task.sam_diagnostic import used_prompts
        used_prompts(prompts, args.mode if args.headless else "box", session.image)
    backend = None

    def backend_factory():
        """首次显式运行才加载模型，之后复用模型及同图特征。"""
        nonlocal backend
        if backend is None:
            backend = SAMBackend(args.models_dir, args.sam_source, SAM_COMMIT, SAM_CONFIG)
        return backend

    print(f"诊断目录：{session.output}", flush=True)
    if args.headless:
        report, _, folder = session.run(prompts, args.mode, backend_factory)
        print(json.dumps({"status": report["status"], "sam_mask_quality_score": report["sam_mask_quality_score"],
                          "mask_pixels": report["mask_pixels"], "output": str(folder)}, ensure_ascii=False))
        return 0 if report["status"] == "mask_predicted" else 2
    import tkinter as tk
    from fuel_cap_task.sam_diagnostic_gui import DiagnosticGUI
    root = tk.Tk()
    DiagnosticGUI(root, session, backend_factory, prompts)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
