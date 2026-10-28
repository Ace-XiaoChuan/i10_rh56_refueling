#!/usr/bin/env python3
"""将已有点云检查结果提取成单个可离线打开的 HTML，保留原坐标和单位。"""

import argparse
import hashlib
import json
from pathlib import Path
import re


PAGE = '''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>油箱盖三维点云</title>
<style>
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;color:#182532;background:#fff}
#app{height:100vh;height:100dvh;display:flex;flex-direction:column}
header{display:flex;align-items:center;justify-content:space-between;gap:16px;flex-wrap:wrap;padding:16px 24px;border-bottom:1px solid #e5eaf0}
h1{font-size:20px;margin:0}.controls{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
label{font-size:14px}select,button{font:inherit;font-size:14px;border:1px solid #cad3de;border-radius:6px;background:#fff;padding:7px 12px;color:inherit}
button{cursor:pointer}button:hover{background:#f0f5fa}#cloud{flex:1;min-height:0}
footer{display:flex;justify-content:space-between;gap:12px;flex-wrap:wrap;padding:10px 24px;border-top:1px solid #e5eaf0;color:#586677;font-size:13px}
@media(max-width:640px){header{padding:12px;gap:10px}footer{padding:8px 12px}}
</style>
</head>
<body>
<div id="app">
<header>
<h1>油箱盖三维点云</h1>
<div class="controls">
<label>选择点云 <select id="sample" aria-label="选择点云"></select></label>
<button id="reset" type="button">重置视角</button>
<button id="fullscreen" type="button">全屏展示</button>
</div>
</header>
<div id="cloud" role="img" aria-label="可旋转、缩放的油箱盖三维点云"></div>
<footer><span id="status" role="status">正在加载点云…</span><span>拖动旋转 · 滚轮缩放 · 右键拖动平移</span></footer>
</div>
<script id="cloud-data" type="application/json">__POINT_CLOUD_DATA__</script>
<script>/*__PLOTLY__*/</script>
<script>
"use strict";
const samples = JSON.parse(document.getElementById("cloud-data").textContent);
const selector = document.getElementById("sample");
const plot = document.getElementById("cloud");
const statusLabel = document.getElementById("status");
const resetButton = document.getElementById("reset");
const fullscreenButton = document.getElementById("fullscreen");
const initialCamera = {eye:{x:1.25,y:1.25,z:1.25},center:{x:0,y:0,z:0},up:{x:0,y:0,z:1}};
for (const [index, sample] of samples.entries()) {
    const option = document.createElement("option");
    option.value = String(index);
    option.textContent = sample.label;
    selector.appendChild(option);
}
async function showCloud() {
    selector.disabled = true;
    resetButton.disabled = true;
    try {
        const sample = samples[Number(selector.value)];
        // 深拷贝绘图输入，避免交互操作改写另一份点云的初始视图。
        const traces = JSON.parse(JSON.stringify(sample.traces));
        const layout = JSON.parse(JSON.stringify(sample.layout));
        layout.autosize = true;
        layout.margin = {l:0,r:0,t:40,b:0};
        layout.uirevision = selector.value;
        layout.scene.camera = JSON.parse(JSON.stringify(initialCamera));
        for (const menu of layout.updatemenus || []) {
            if (menu.buttons.length === 2) {
                menu.buttons[0].label = "坐标着色";
                menu.buttons[1].label = "文件颜色";
            }
        }
        await Plotly.react(plot, traces, layout, {
            responsive:true, displaylogo:false,
            toImageButtonOptions:{format:"png",filename:"fuel_cap_point_cloud",width:1600,height:1000,scale:2}
        });
        const frame = sample.frame_id === "camera" ? "相机坐标系" : sample.frame_id;
        statusLabel.textContent = `${sample.point_count.toLocaleString()} 个点 · ${frame} · 单位 ${sample.length_unit}`;
    } catch (error) {
        statusLabel.textContent = "三维场景加载失败，请使用支持 WebGL 的浏览器。";
        console.error(error);
    } finally {
        selector.disabled = false;
        resetButton.disabled = false;
    }
}
selector.addEventListener("change", showCloud);
resetButton.addEventListener("click", () => Plotly.relayout(plot, {"scene.camera":initialCamera}));
fullscreenButton.hidden = !document.fullscreenEnabled;
fullscreenButton.addEventListener("click", async () => {
    try {
        if (document.fullscreenElement) await document.exitFullscreen();
        else await document.getElementById("app").requestFullscreen();
    } catch (error) {
        console.error(error);
    }
});
document.addEventListener("fullscreenchange", () => {
    fullscreenButton.textContent = document.fullscreenElement ? "退出全屏" : "全屏展示";
    if (plot._fullLayout) Plotly.Plots.resize(plot);
});
showCloud();
</script>
</body>
</html>
'''


def digest(path):
    """核对归档摘要，避免从已变化的页面提取数据。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract_samples(folder, batch):
    """直接提取原页面的 Plotly 输入，不重新计算、缩放或合并点云。"""
    decoder = json.JSONDecoder()
    samples = []
    for index, report in enumerate(batch["clouds"]):
        page = folder / f"cloud_{index:03d}" / "index.html"
        if digest(page) != report["outputs_sha256"]["index.html"]:
            raise ValueError(f"页面摘要与检查报告不一致：{page}")
        source = page.read_text(encoding="utf-8")
        payload = source.split("Plotly.newPlot('cloud', ", 1)[1]
        traces, end = decoder.raw_decode(payload)
        layout, _ = decoder.raw_decode(payload[end:].lstrip(", "))
        samples.append({"label": f"点云 {index + 1}", "length_unit": report["length_unit"],
                        "frame_id": report["frame_id"],
                        "point_count": sum(len(trace["x"]) for trace in traces),
                        "source_name": Path(report["input_path"]).name,
                        "source_sha256": report["input_sha256"],
                        "traces": traces, "layout": layout})
    if not samples:
        raise ValueError("检查结果中没有可提取的点云")
    return samples


def main():
    """将点云数据和现有 Plotly 脚本内嵌到新文件，不依赖服务器或其他文件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspection-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    folder = args.inspection_dir.resolve()
    output = args.output.resolve()
    if output.exists():
        parser.error(f"输出文件已存在，请指定新文件：{output}")
    batch = json.loads((folder / "result.json").read_text(encoding="utf-8"))
    javascript = folder / "plotly.min.js"
    if digest(javascript) != batch["plotly_sha256"]:
        raise ValueError("Plotly 脚本摘要与检查报告不一致")
    samples = extract_samples(folder, batch)
    payload = json.dumps(samples, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    # 防止文件名等文本结束内嵌的 JSON 或脚本标签。
    for character in ("<", ">", "&", "\u2028", "\u2029"):
        payload = payload.replace(character, f"\\u{ord(character):04x}")
    library = re.sub(r"</script", r"<\\/script", javascript.read_text(encoding="utf-8"), flags=re.I)
    page = PAGE.replace("__POINT_CLOUD_DATA__", payload, 1).replace("/*__PLOTLY__*/", library, 1)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(page, encoding="utf-8")
    print(json.dumps({"output": str(output), "clouds": len(samples),
                      "point_counts": [sample["point_count"] for sample in samples],
                      "size_bytes": output.stat().st_size}, ensure_ascii=False))


if __name__ == "__main__":
    main()
