# 整理后项目运行补验（2026-10-09）

本轮从项目保留的 `.venv`、`.venv-perception` 和 `.venv-spahybgen` 入口实际运行检查；未安装或修改依赖，未修改任务实现、模型或正式参数。

两个共享入口的 `sys.prefix` 可显示各自软链接路径，解析后均为 `/home/ace/mujoco-env`。共享环境为 Python 3.10.12、MuJoCo 3.13.0；SpaHybGen 仍为 Python 3.8.20、MuJoCo 3.0.0。

| 检查 | 结果 |
| --- | --- |
| 共享环境与感知入口导入、依赖检查 | 通过 |
| 规划模型来源 | 234 个输入、233 个生成文件通过校验 |
| 当前候选与已有接近轨迹 | 加载及 268 个路点的运动学复核通过 |
| 现有回归测试 | 68 项非 GUI 与 2 项 GUI，通过共 70 项 |
| 任务编排演示 | 提前脱离、重新抓握成功；支撑未知按预期停止（退出码 1） |
| SpaHybGen 独立环境 | 关键库导入及已有示例结果复核通过，未重跑优化 |
| 新 RGB-D 渲染 | 通过；[RGB 图](rgbd/rgb.png) |
| 新 Grounding DINO + SAM 推理 | 一个非空候选，新结果来源复核通过；[叠加图](automatic_segmentation/overlay.png) |
| 同图同用户框的 SAM 推理 | 掩膜 35894 像素，与历史结果逐像素一致；[叠加图](manual_sam/trials/0001_box/overlay.png) |
| MuJoCo 交互窗口 | 实际回放一轮并正常关闭，仅前向计算 |
| 8 个选定输入、模型、历史报告和入口源码 | 补验前后摘要不变 |

初次检查脚本误要求 `sys.prefix` 字符串必须是共享环境实际路径；库导入已成功。已改为解析软链接后比较，并实际重跑两项检查。原日志带 `_first_attempt` 后缀保留，未改 Python 环境。

详细命令、退出码与耗时见 [checks.json](checks.json)，像素对照见 [sam_comparison.json](sam_comparison.json)。

你可以在轻量工作区终端亲自打开窗口（每次关闭一个窗口后再运行下一条）：

```bash
.venv/bin/python tools/view_mjcf.py --with-hand
.venv-perception/bin/python tools/diagnose_sam_image.py
```

终端默认目录已设置为项目根目录。SAM 窗口中的新会话独立保存，需自行画框或载入提示后显式运行。

实际推理和窗口检查是软件环境验证。没有重新生成路径或运行 SpaHybGen 优化；不代表实物自动识别、物理抓稳、完整拧盖或真机通过。用户仍可自行确认窗口交互效果。
