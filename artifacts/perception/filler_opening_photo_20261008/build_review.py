"""汇总实际推理与独立读图意见；不重跑模型，不更改预测或接受逻辑。"""

import hashlib
import html
import json
from pathlib import Path
import shutil

BASE = Path(__file__).resolve().parent
CASES = [
    ("installed_cap", "带盖安装场景：默认提示", "误检油箱面板和外门，没有定位中央盖体。"),
    ("cap_front_control", "单独盖体正面：正对照", "目视与整盖外轮廓一致；没有人工标注，不能报告 IoU。"),
    ("fuel_cap", "无盖场景：缺席对照", "盖体缺席，却误检大片油箱面板及周边结构。"),
    ("fuel_filler_opening", "无盖场景：加油口提示", "返回两个候选：大片面板及凹腔，未取得唯一精确中央加油口。"),
    ("installed_black_cap", "带盖安装场景：描述词对照一", "歧义：误检面板及下方支撑部件，没有定位中央盖体。"),
    ("installed_descriptive_cap", "带盖安装场景：描述词对照二", "仍误检油箱面板，没有定位中央盖体。"),
]


def digest(path):
    """绑定原始图像、预测文件和汇总实现。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """保留原始图像副本供左右对照；读图意见不回写模型结果。"""
    (BASE / "inputs").mkdir(exist_ok=True)
    entries, sections = [], []
    for name, title, assessment in CASES:
        folder = BASE / name
        report = json.loads((folder / "result.json").read_text())
        if report["status"] == "runtime_error":
            raise ValueError("inference not completed: " + name)
        image = Path(report["image_path"])
        if digest(image) != report["image_sha256"]:
            raise ValueError("input image changed")
        for filename, expected in report["outputs"].items():
            if digest(folder / filename) != expected:
                raise ValueError("inference output changed")
        saved_image = BASE / "inputs" / image.name
        shutil.copyfile(image, saved_image)
        area = report["image_size_wh"][0] * report["image_size_wh"][1]
        candidates = [dict(candidate, mask_image_fraction=candidate["mask_pixels"] / area)
                      for candidate in report["candidates"]]
        entries.append({"run": name, "title": title, "prompt": report["prompt"],
            "software_status": report["status"], "selected_index": report["selected_index"],
            "image_sha256": report["image_sha256"], "result_sha256": digest(folder / "result.json"),
            "assessment_source": "assistant_visual_inspection", "assessment": assessment,
            "candidates": candidates, "valid_3d_pose": False, "pixel_ground_truth_available": False})
        rows = []
        links = []
        for c in candidates:
            rows.append(f'<tr><td>{c["index"]}</td><td>{html.escape(c["label"])}</td>'
                        f'<td>{c["detector_text_match_score"]:.6f}</td>'
                        f'<td>{c["sam_mask_quality_score"]:.6f}</td>'
                        f'<td>{c["mask_image_fraction"]:.2%}</td></tr>')
            links.append(f'<a href="{name}/{c["mask_file"]}">掩膜 {c["index"]}</a>')
        sections.append(f'<section><h2>{html.escape(title)}</h2>'
            f'<p>提示：<code>{html.escape(report["prompt"])}</code>；软件状态：'
            f'<code>{report["status"]}</code>；候选数：{report["candidate_count"]}</p>'
            f'<p class="assessment">独立读图：{html.escape(assessment)}</p>'
            f'<div class="views"><figure><a href="inputs/{image.name}"><img src="inputs/{image.name}"></a>'
            f'<figcaption>原始照片（点击放大）</figcaption></figure>'
            f'<figure><a href="{name}/overlay.png"><img src="{name}/overlay.png"></a>'
            f'<figcaption>实际检测框与掩膜（点击放大）</figcaption></figure></div>'
            f'<table><tr><th>候选</th><th>模型标签</th><th>文本匹配</th><th>SAM 质量</th>'
            f'<th>掩膜占图比例</th></tr>{"".join(rows)}</table>'
            f'<p>{" · ".join(links)} · <a href="{name}/result.json">原始结果与来源</a></p></section>')
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>实物照片定位与分割检查</title>
<style>body{max-width:1200px;margin:30px auto;padding:0 20px;font:16px/1.6 sans-serif;color:#263238}
section{border-top:1px solid #ccc;padding:16px 0}.views{display:flex;gap:12px}figure{margin:0;width:50%}
img{max-width:100%;height:auto}figcaption{color:#555}table{border-collapse:collapse}td,th{padding:6px 12px;border:1px solid #ccc}
.assessment{background:#f3f5f7;padding:12px}@media(max-width:700px){.views{display:block}figure{width:100%}}</style>
<h1>实物照片定位与分割检查 · 2026-10-08</h1>
<p>当前 Grounding DINO + SAM 2：单独盖体照片有正向证据，实物安装场景未通过整盖定位。
六组均使用原始整图、固定模型及阈值，没有人工框、裁剪、训练或机器人动作。</p>
<p>橙框为检测框，彩色区域为模型掩膜。高匹配分数、高 SAM 质量和唯一候选均不保证目标语义正确。
以下独立读图意见不回写预测；没有人工像素标注或深度，不报告 IoU、三维位姿或稳定帧率。</p>
'''
    (BASE / "index.html").write_text(page + "\n".join(sections) + "</html>\n")
    review = {"inference_reexecuted": False, "runtime_acceptance_changed": False,
              "reviewer": "assistant_visual_inspection", "installed_scene_feasibility_passed": False,
              "cases": entries, "builder_sha256": digest(Path(__file__)),
              "index_sha256": digest(BASE / "index.html"),
              "saved_image_sha256": {p.name: digest(p) for p in (BASE / "inputs").iterdir()}}
    (BASE / "review.json").write_text(json.dumps(review, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print("Saved six-case photo review:", BASE / "index.html")


if __name__ == "__main__":
    main()
