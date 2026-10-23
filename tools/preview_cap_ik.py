#!/usr/bin/env python3
"""Solve the selected cap-relative grasp on the full i10 model and inspect endpoints."""
import argparse
import json
import os
from pathlib import Path
from queue import SimpleQueue
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
if '--render-dir' in sys.argv and '--view' not in sys.argv:
    os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
from fuel_cap_task.arm_ik import ArmIKPreview
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP


def camera(cam, near=False, cap_position=None):
    cam.lookat[:] = [0, .86, .42] if near else [0, .42, .5]
    if cap_position is not None:
        cam.lookat[2] += float(cap_position[2]) - .4
    cam.distance = .65 if near else 2.3
    cam.azimuth, cam.elevation = (90 if near else 135), -20


def target_marker(preview, scene):
    t = preview.world_from_hand
    for axis in range(3):
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        rgba = np.zeros(4,dtype=np.float32)
        rgba[axis],rgba[3] = 1,1
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_ARROW,np.ones(3)*.003,
                           t[:3,3],np.eye(3).ravel(),rgba)
        mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_ARROW,.003,t[:3,3],t[:3,3]+.06*t[:3,axis])
        scene.ngeom += 1


def render(preview, report, directory):
    from search_cap_grasp import write_png
    directory.mkdir(parents=True,exist_ok=True)
    m,d = preview.model,preview.data
    m.vis.global_.offwidth,m.vis.global_.offheight = 1000,800
    opt,cam = mujoco.MjvOption(),mujoco.MjvCamera()
    opt.geomgroup[0] = opt.geomgroup[3] = 0
    states = [('pregrasp',False)]
    if (preview.candidate.get('contact_trial') or {}).get('final_qpos'):
        states.append(('recorded_closure',True))
    with mujoco.Renderer(m,height=800,width=1000) as renderer:
        for name,closed in states:
            preview.show(report['solutions'][0],closed)
            for near in (False,True):
                camera(cam,near,preview.world_from_cap[:3,3])
                renderer.update_scene(d,camera=cam,scene_option=opt)
                target_marker(preview,renderer.scene)
                write_png(directory/f'{name}_{"near" if near else "full"}.png',renderer.render())
    preview.show(report['solutions'][0])


def view(preview, report):
    import mujoco.viewer
    keys = SimpleQueue()
    index,closed,near = 0,False,False
    def describe():
        s=report['solutions'][index]
        print(f"solution {index+1}/{len(report['solutions'])}; "
              f"{'RECORDED closure (no physics)' if closed else 'pregrasp'}; "
              f"hand error {1000*s['hand_position_error_m']:.4f} mm / "
              f"{np.rad2deg(s['hand_orientation_error_rad']):.5f} deg; "
              f"pregrasp interference contacts={s['interference_count']}",flush=True)
    print('N/P solution | O pregrasp | G recorded actual closure | C collision meshes | F full/near view',flush=True)
    describe()
    before=set(threading.enumerate())
    threads=[]
    try:
        with mujoco.viewer.launch_passive(preview.model,preview.data,key_callback=keys.put,
                                         show_left_ui=False,show_right_ui=False) as viewer:
            threads=[t for t in threading.enumerate() if t not in before and '_launch_internal' in t.name]
            with viewer.lock():
                camera(viewer.cam,cap_position=preview.world_from_cap[:3,3])
                viewer.opt.geomgroup[0]=viewer.opt.geomgroup[3]=0
            while viewer.is_running():
                with viewer.lock():
                    changed=False
                    while not keys.empty():
                        key=keys.get()
                        if key in (ord('N'),ord('P')):
                            index=(index+(1 if key==ord('N') else -1))%len(report['solutions'])
                        elif key==ord('O'): closed=False
                        elif key==ord('G'):
                            closed=bool((preview.candidate.get('contact_trial') or {}).get('final_qpos'))
                        elif key==ord('C'):
                            viewer.opt.geomgroup[0]=1-viewer.opt.geomgroup[0]
                            viewer.opt.geomgroup[3]=1-viewer.opt.geomgroup[3]
                        elif key==ord('F'):
                            near=not near
                            camera(viewer.cam,near,preview.world_from_cap[:3,3])
                        changed=True
                    if changed:
                        preview.show(report['solutions'][index],closed)
                        describe()
                    viewer.user_scn.ngeom=0
                    target_marker(preview,viewer.user_scn)
                viewer.sync()
                time.sleep(1/60)
    except KeyboardInterrupt:
        pass
    finally:
        for thread in threads: thread.join()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,default=DEFAULT_FIXED_GRASP)
    parser.add_argument('--output',type=Path,default=DEFAULT_FIXED_GRASP.parent/'arm_ik_current.json')
    parser.add_argument('--view',action='store_true')
    parser.add_argument('--render-dir',type=Path)
    parser.add_argument('--distance',type=float, help='Optional outward approach distance in metres')
    args=parser.parse_args()
    preview=ArmIKPreview(args.candidate, approach_distance_m=args.distance)
    report=preview.solve(progress=lambda s:print(s,flush=True))
    if report['solutions']:
        preview.show(report['solutions'][0],closed=bool((preview.candidate.get('contact_trial') or {}).get('final_qpos')))
        report['selected_recorded_closure_contacts']=preview.contacts()
        preview.show(report['solutions'][0])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Report:',args.output.resolve(),flush=True)
    if not report['solutions']:
        print('No converged IK found in this numerical search; no reachability proof.',flush=True)
        return 1
    s=report['solutions'][0]
    print(f"IK found; hand error {s['hand_position_error_m']*1000:.6f} mm, "
          f"{np.rad2deg(s['hand_orientation_error_rad']):.6f} deg; "
          f"enabled-geometry interference contacts: {s['interference_count']}",flush=True)
    print('Endpoint preview only; sensor/adapter and ground collision, paths and physics are unverified.',flush=True)
    if args.render_dir: render(preview,report,args.render_dir)
    if args.view: view(preview,report)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
