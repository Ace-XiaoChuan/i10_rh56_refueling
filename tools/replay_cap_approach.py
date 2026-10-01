#!/usr/bin/env python3
"""Replay actual MTC approach in MuJoCo; no actuators, mj_step or dynamics."""
import argparse
import ctypes
import json
from pathlib import Path
from queue import SimpleQueue
import struct
import sys
import threading
import time
import zlib
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import mujoco
from fuel_cap_task.trajectory_replay import ApproachReplay
from preview_cap_ik import camera
from search_cap_grasp import write_png


def chunk(kind, payload):
    return struct.pack('!I', len(payload))+kind+payload+struct.pack('!I', zlib.crc32(kind+payload))


def render(replay, folder, frames=120):
    folder.mkdir(parents=True, exist_ok=True)
    model, data = replay.model, replay.data
    model.vis.global_.offwidth, model.vis.global_.offheight = 900, 700
    cam, opt = mujoco.MjvCamera(), mujoco.MjvOption()
    camera(cam, cap_position=replay.report['request']['cap_estimate']['pose']['position_m'])
    opt.geomgroup[0] = opt.geomgroup[3] = 0
    with (folder/'approach.png').open('wb') as output, mujoco.Renderer(model, height=700, width=900) as renderer:
        output.write(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('!2I5B', 900,700,8,2,0,0,0)))
        output.write(chunk(b'acTL', struct.pack('!2I', frames, 1)))
        sequence = 0
        for index in range(frames):
            replay.set_progress(index/(frames-1))
            renderer.update_scene(data, camera=cam, scene_option=opt)
            rgb = renderer.render()
            output.write(chunk(b'fcTL', struct.pack('!5I2H2B', sequence,900,700,0,0,1,15,0,0)));sequence += 1
            compressed = zlib.compress(b''.join(b'\0'+row.tobytes() for row in rgb))
            if index == 0:
                output.write(chunk(b'IDAT', compressed))
            else:
                output.write(chunk(b'fdAT', struct.pack('!I',sequence)+compressed));sequence += 1
            if index in (0,frames//2,frames-1):
                write_png(folder/f'frame_{index:03}.png', rgb)
        output.write(chunk(b'IEND',b''))
    (folder/'index.html').write_text('<meta charset="utf-8"><title>MTC 接近回放</title><h2>MTC → MuJoCo 接近轨迹</h2>'
                                    '<p>仅运动学回放；动画显示时长不是执行时间。刷新页面可重播。</p><img src="approach.png" width="900">')
    replay.set_progress(0)


def view(replay, duration, auto_close):
    # WSLg rendering threads must initialize X11 before opening GLFW.
    ctypes.CDLL('libX11.so.6').XInitThreads()
    import mujoco.viewer
    keys = SimpleQueue();before = set(threading.enumerate());threads=[]
    elapsed=0.;playing=True;near=False
    print('SPACE pause/resume | R restart | F full/near | C collision geometry. End holds last pose.',flush=True)
    try:
        with mujoco.viewer.launch_passive(replay.model,replay.data,key_callback=keys.put,show_left_ui=False,show_right_ui=False) as viewer:
            threads=[t for t in threading.enumerate() if t not in before and '_launch_internal' in t.name]
            camera(viewer.cam,cap_position=replay.report['request']['cap_estimate']['pose']['position_m'])
            viewer.opt.geomgroup[0]=viewer.opt.geomgroup[3]=0
            tick=time.monotonic();started=tick
            while viewer.is_running():
                now=time.monotonic();delta=now-tick;tick=now
                with viewer.lock():
                    while not keys.empty():
                        key=keys.get()
                        if key==32: playing=not playing
                        elif key==ord('R'):elapsed=0.;playing=True
                        elif key==ord('F'):
                            near=not near;camera(viewer.cam,near,replay.report['request']['cap_estimate']['pose']['position_m'])
                        elif key==ord('C'):viewer.opt.geomgroup[3]=not viewer.opt.geomgroup[3]
                    if playing:elapsed=min(duration,elapsed+delta)
                    replay.set_progress(elapsed/duration)
                viewer.sync()
                if auto_close and now-started > duration+1:break
                time.sleep(.01)
    finally:
        for thread in threads:thread.join()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory',type=Path,default=Path('artifacts/planning/approach.json'))
    parser.add_argument('--view',action='store_true')
    parser.add_argument('--duration',type=float,default=8.,help='Display seconds, not execution timing')
    parser.add_argument('--auto-close',action='store_true',help='Close viewer after one full playback (validation)')
    parser.add_argument('--render-dir',type=Path)
    args=parser.parse_args()
    if not 0 < args.duration < float('inf'):parser.error('Positive finite duration required')
    replay=ApproachReplay(args.trajectory);summary=replay.validate()
    out=args.trajectory.with_name('replay_validation.json');out.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    if args.render_dir:render(replay,args.render_dir)
    if args.view:view(replay,args.duration,args.auto_close)

if __name__=='__main__':main()
