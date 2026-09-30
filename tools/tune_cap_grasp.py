#!/usr/bin/env python3
"""MuJoCo manual cap-grasp placement and fixed-fixture closure panel."""
import argparse
from pathlib import Path
import sys
import time
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import mujoco
import numpy as np
from fuel_cap_task.manual_grasp import ManualGrasp
from fuel_cap_task.grasp_search import pose_arrays, quaternion, rotation


def rpy_degrees(r):
    pitch = np.arcsin(np.clip(-r[2, 0], -1, 1))
    if abs(np.cos(pitch)) > 1e-8:
        roll, yaw = np.arctan2(r[2, 1], r[2, 2]), np.arctan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = 0., np.arctan2(-r[0, 1], r[1, 1])
    return np.rad2deg([roll, pitch, yaw])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--load', type=Path)
    parser.add_argument('--output-dir', type=Path, default=Path('artifacts/manual_grasp'))
    parser.add_argument('--check', action='store_true', help='Load and validate initial setup without GUI or integration')
    args = parser.parse_args()
    session = ManualGrasp()
    if args.load:
        session.load(args.load)
    if args.check:
        print('Manual fixture OK; initial penetrations:', len(session.initial_penetrations))
        print('Candidate only; final channels:', session.scene.closed)
        return

    # Tk uses the main thread while MuJoCo renders through GLFW on another
    # thread. Initialize Xlib threading before either opens an X connection.
    if sys.platform.startswith('linux'):
        import ctypes
        x11 = ctypes.CDLL('libX11.so.6')
        if not x11.XInitThreads():
            raise RuntimeError('X11 thread initialization failed')
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    import mujoco.viewer
    root = tk.Tk()
    root.title('油箱盖手动调姿 — 未标定局部实验')
    frame = ttk.Frame(root, padding=12)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='盖体系位置 mm；固定轴 XYZ 姿态 deg。修改后点击应用。').grid(row=0, column=0, columnspan=4)
    values = [tk.StringVar() for _ in range(12)]
    controls = []
    for i, label in enumerate(('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw',
                               '小指', '无名指', '中指', '食指', '拇指弯曲', '拇指旋转')):
        ttk.Label(frame, text=label).grid(row=i+1, column=0, sticky='w')
        spin = ttk.Spinbox(frame, textvariable=values[i], width=18,
                           from_=0 if i >= 6 else -10000, to=1000 if i >= 6 else 10000,
                           increment=10 if i >= 6 else 1)
        spin.grid(row=i+1, column=1, columnspan=3, sticky='ew')
        controls.append(spin)
    ttk.Label(frame, text=f'最终闭合目标（只读）：{list(session.scene.closed)}').grid(row=13, column=0, columnspan=4)
    info, message = tk.StringVar(), tk.StringVar(value='初始几何种子，尚非有效抓姿。')
    ttk.Label(frame, textvariable=info, justify='left', wraplength=520).grid(row=16, column=0, columnspan=4, sticky='w')
    ttk.Label(frame, textvariable=message, wraplength=520).grid(row=17, column=0, columnspan=4, sticky='w')
    ttk.Label(frame, text='固定手根/盖体实验；不验证持起、拧盖或机械臂可达性。').grid(row=18, column=0, columnspan=4)

    def populate():
        p, r = pose_arrays(session.pose)
        for variable, value in zip(values, [*(p * 1000), *rpy_degrees(r), *session.channels]):
            variable.set(f'{value:.9g}')

    def apply():
        v = np.array([float(x.get()) for x in values])
        if not np.isfinite(v).all():
            raise ValueError('请输入有限数值')
        roll, pitch, yaw = np.deg2rad(v[3:6])
        r = rotation(2, yaw) @ rotation(1, pitch) @ rotation(0, roll)
        session.reset({'position_m': (v[:3]/1000).tolist(), 'orientation_xyzw': quaternion(r)}, v[6:])
        message.set(f'已应用；初态干涉 {len(session.initial_penetrations)} 对。')

    def close():
        apply()
        session.start()
        message.set('正在积分实际闭合，可随时暂停。')

    def reset():
        session.reset()
        populate()
        message.set('已返回本次已应用的接近预型。')

    def save():
        message.set(f'已保存：{session.save(args.output_dir).resolve()}')

    def load():
        path = filedialog.askopenfilename(filetypes=[('候选 JSON', '*.json')])
        if path:
            session.load(path)
            populate()
            message.set('已载入候选初态；需重新闭合验证。')

    def guarded(action):
        def call():
            try:
                with viewer.lock():
                    action()
            except Exception as error:
                session.stop()
                messagebox.showerror('操作未完成', str(error))
        return call

    for i, (label, action) in enumerate((('应用调姿', apply), ('闭合', close),
                                         ('暂停', session.stop), ('重置', reset),
                                         ('保存已应用候选', save), ('载入候选', load))):
        button = ttk.Button(frame, text=label, command=guarded(action))
        button.grid(row=14+i//4, column=i%4, sticky='ew')
        if label not in ('暂停', '重置'):
            controls.append(button)
    populate()
    alive = True
    def quit_panel():
        nonlocal alive
        alive = False
        session.stop()
    root.protocol('WM_DELETE_WINDOW', quit_panel)
    s = session.scene
    viewer_threads = []
    threads_before = set(threading.enumerate())
    try:
        with mujoco.viewer.launch_passive(s.model, s.data, show_left_ui=False, show_right_ui=False) as viewer:
            # This installed MuJoCo launches a daemon render thread. Join it
            # after close so interpreter shutdown cannot race GLFW teardown.
            viewer_threads = [t for t in threading.enumerate()
                              if t not in threads_before and '_launch_internal' in t.name]
            with viewer.lock():
                viewer.cam.lookat[:] = (s.data.site('cap_grip_center').xpos + s.data.body('base').xpos) / 2
                viewer.cam.distance = .65
                viewer.cam.azimuth, viewer.cam.elevation = 90, -20
                viewer.opt.geomgroup[0] = 0
                viewer.opt.geomgroup[3] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            collision = tk.BooleanVar()
            def toggle_collision():
                with viewer.lock():
                    viewer.opt.geomgroup[0] = viewer.opt.geomgroup[3] = int(collision.get())
            ttk.Checkbutton(frame, text='显示碰撞几何', variable=collision,
                            command=toggle_collision).grid(row=15, column=2, columnspan=2)
            while alive and viewer.is_running():
                with viewer.lock():
                    try:
                        session.advance()
                    except Exception as error:
                        message.set(f'实验已停止：{error}')
                    obs = session.explorer.observation()
                    forces = ', '.join(f'{f:.3g}' for f in obs['terminal_normal_forces_n'])
                    initial = session.initial_penetrations[:3]
                    detail = '\n'.join(f"{x['geoms']}: {1000*x['distance_m']:.2f} mm" for x in initial)
                    info.set(f'{session.status} | 仿真时间 {s.data.time:.3f} s\n'
                             f'拇/食/中末端对握梁法向力 N：{forces}\n'
                             f"最大接触深度 {obs['max_actual_contact_depth_m']*1000:.3f} mm；"
                             f"其他承载接触 {len(obs['other_loaded_contacts'])} 对\n"
                             f'初态干涉 {len(session.initial_penetrations)} 对\n{detail}')
                for control in controls:
                    control.configure(state='disabled' if session.run is not None else 'normal')
                root.update()
                viewer.sync()
                time.sleep(.01)
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
        for thread in viewer_threads:
            thread.join()
        root.destroy()


if __name__ == '__main__':
    main()
