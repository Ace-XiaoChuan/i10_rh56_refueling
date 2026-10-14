# Python 3.10 共享环境整理结果（2026-10-09）

仿真与感知共用 `/home/ace/mujoco-env`。工作区 `.venv`、`.venv-perception` 均为软链接，原实体目录及临时备份已删除；`.venv-spahybgen` 保持独立。正式当前状态以项目合同的环境合并节为准。

已有共享包仅改变 MuJoCo 和 setuptools 版本，其余原共享包版本保留；新增包和前后版本在 `shared_smoke.json`、`validation.json` 中记录。MuJoCo 原版本能加载场景，但当前候选与回放的版本绑定拒绝运行，因此采用项目记录版本，保留原来源检查。

- 全部回归：149 项通过、2 项因沙箱显示权限跳过。
- WSLg 补验：上述 2 项 GUI 测试通过。
- 依赖、原机器人库导入、NumPy–PyTorch 转换、候选加载、已有接近回放验证及规划来源检查通过。
- 已有初始构型筛选结果只读复核通过，没有重新搜索或规划。
- 新 RGB-D 渲染、Grounding DINO + SAM 推理及新结果复核通过。
- 同图同人工框的 SAM 掩膜与历史结果逐像素一致；新实验保存在 `sam_check/`，自动链保存在 `detector_check/`。
- 两个软链接、激活及 Git 忽略检查通过；31 个选定源码/候选/基线文件摘要不变。

在线安装计划因包源超时未能完成，实际安装使用离线包。`recovered_wheels.json` 记录缓存来源；`repacked_*_wheels.json` 记录逐项核对原安装 RECORD 摘要后重建的包来源及新 wheel 摘要；安装结果在 `install_report.json` 与 `sam_install_report.json` 中保存。早期失败的安装计划日志保留，不能当作成功安装记录。

工作区原命令可继续使用，也可执行 `source ~/mujoco-env/bin/activate`。本地 VS Code 配置已更新解释器与大目录排除规则。

本轮不改变视觉语义验收结论或物理验收边界，未测量 WSL 启动速度；旧视觉报告仍绑定其原完整环境。
