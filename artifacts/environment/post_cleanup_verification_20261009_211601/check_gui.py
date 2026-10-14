"""打开已有接近轨迹的 MuJoCo 窗口，完成一轮后自动关闭；不写入旧报告。"""
import json
from pathlib import Path
import sys

ROOT = Path('/home/ace/i10_rh56_refueling_ws')
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'tools')]
from fuel_cap_task.trajectory_replay import ApproachReplay
from replay_cap_approach import view

replay = ApproachReplay(ROOT / 'artifacts/planning/approach.json')
print(json.dumps(replay.validate(), ensure_ascii=False), flush=True)
view(replay, duration=4., auto_close=True)
print('MuJoCo 窗口完整回放并正常关闭；仅运行前向计算。', flush=True)
