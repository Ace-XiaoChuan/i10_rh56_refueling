#!/usr/bin/env python3
"""Sample bounded tank/cap arrangements; save every endpoint result, including failures."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
from queue import SimpleQueue
import sys
import threading
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
if '--render-dir' in sys.argv and '--view' not in sys.argv:os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from fuel_cap_task.random_scenes import SceneSampler, SamplingSettings, classify, save_report
from preview_cap_ik import camera,target_marker

DEFAULT_CANDIDATE=Path('artifacts/manual_grasp/user_selected_20260930.json')


def select(sampler,row,solution=0):
    p=sampler.preview(row['sample'])
    if row['ik']['solutions']:p.show(row['ik']['solutions'][solution])
    # With no inverse solution, leave the initial arm and draw the target.
    return p


def view(sampler,report):
    import mujoco.viewer
    index=0
    print('N/P: next/previous sample | [/]: IK solution | C: collision meshes | F: full/near | Esc: exit',flush=True)
    # Each sample is a separately compiled model. Reopen the viewer on sample
    # switches instead of mutating static bodies behind MuJoCo collision caches.
    while True:
        row=report['results'][index];p=select(sampler,row)
        keys=SimpleQueue();next_index=None;solution=0;near=False
        print(f"sample {index+1}/{len(report['results'])}: {row['sample']} -> {row['status']}; "
              f"{len(row['ik']['solutions'])} IK solutions",flush=True)
        before=set(threading.enumerate());threads=[]
        try:
            with mujoco.viewer.launch_passive(p.model,p.data,key_callback=keys.put,
                                             show_left_ui=False,show_right_ui=False) as viewer:
                threads=[t for t in threading.enumerate() if t not in before and '_launch_internal' in t.name]
                with viewer.lock():
                    camera(viewer.cam,cap_position=p.world_from_cap[:3,3])
                    viewer.opt.geomgroup[0]=viewer.opt.geomgroup[3]=0
                while viewer.is_running() and next_index is None:
                    with viewer.lock():
                        while not keys.empty():
                            key=keys.get()
                            if key in (ord('N'),ord('P')):
                                next_index=(index+(1 if key==ord('N') else -1))%len(report['results'])
                            elif key in (ord('['),ord(']')) and row['ik']['solutions']:
                                solution=(solution+(1 if key==ord(']') else -1))%len(row['ik']['solutions'])
                                p.show(row['ik']['solutions'][solution])
                                print('solution',solution+1,'interference contacts:',row['ik']['solutions'][solution]['interference_count'],flush=True)
                            elif key==ord('C'):
                                viewer.opt.geomgroup[0]=1-viewer.opt.geomgroup[0]
                                viewer.opt.geomgroup[3]=1-viewer.opt.geomgroup[3]
                            elif key==ord('F'):
                                near=not near;camera(viewer.cam,near,p.world_from_cap[:3,3])
                                if near:viewer.cam.lookat[:]=p.world_from_cap[:3,3]
                        viewer.user_scn.ngeom=0;target_marker(p,viewer.user_scn)
                    viewer.sync();time.sleep(1/60)
        except KeyboardInterrupt:return
        finally:
            for thread in threads:thread.join()
        if next_index is None:return
        index=next_index


def render(sampler,report,folder):
    from search_cap_grasp import write_png
    folder.mkdir(parents=True,exist_ok=True)
    cards=[]
    for row in report['results']:
        p=select(sampler,row)
        p.model.vis.global_.offwidth=800;p.model.vis.global_.offheight=640
        opt,cam=mujoco.MjvOption(),mujoco.MjvCamera()
        opt.geomgroup[0]=opt.geomgroup[3]=0;camera(cam,cap_position=p.world_from_cap[:3,3])
        name=f"sample_{row['sample']['id']:03d}.png"
        with mujoco.Renderer(p.model,height=640,width=800) as renderer:
            renderer.update_scene(p.data,camera=cam,scene_option=opt)
            target_marker(p,renderer.scene);write_png(folder/name,renderer.render())
        sample=row['sample']
        cards.append(f'<article><h2>{sample["id"]}: {sample["group"]} — {row["status"]}</h2>'
                     f'<p>Opening/base m: {sample["opening_base_m"]}; tank yaw {sample["tank_yaw_deg"]:.2f}°; '
                     f'cap yaw {sample["cap_yaw_deg"]:.2f}°</p><img src="{name}"></article>')
    (folder/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Bounded scene endpoint checks</title>'
        '<style>body{font:16px sans-serif;margin:24px}article{display:inline-block;width:48%;vertical-align:top}img{width:100%}</style>'
        '<h1>Bounded scene endpoint checks</h1><p>Pregrasp only. No path/contact execution. '
        'Failures retained; ik_not_found shows initial arm and target marker. '
        'Clear means only no penetration over diagnostic threshold in enabled geometry.</p>'+''.join(cards),encoding='utf-8')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--candidate',type=Path,default=DEFAULT_CANDIDATE)
    ap.add_argument('--per-group',type=int,default=4)
    ap.add_argument('--seed',type=int,default=0)
    ap.add_argument('--output',type=Path,default=Path('artifacts/random_scenes/report.json'))
    ap.add_argument('--load-report',type=Path)
    ap.add_argument('--view',action='store_true')
    ap.add_argument('--render-dir',type=Path)
    args=ap.parse_args()
    if args.load_report:
        report=json.loads(args.load_report.read_text())
        if report.get('schema_version')!=1 or report.get('mode')!='bounded_scene_endpoint_checks' or not report.get('results'):
            ap.error('Unsupported or empty report')
        sampler=SceneSampler(args.candidate,SamplingSettings(**report['sampling_settings']))
        if (report['source_scene_sha256']!=sampler.reference.scene_hash or report['candidate_sha256']!=hashlib.sha256(args.candidate.read_bytes()).hexdigest()
                or report['mujoco_version']!=mujoco.__version__):ap.error('Scene/candidate/MuJoCo changed; regenerate report')
    else:
        try:settings=SamplingSettings(per_group=args.per_group,random_seed=args.seed)
        except ValueError as e:ap.error(str(e))
        sampler=SceneSampler(args.candidate,settings)
        samples=sampler.samples()
        report={'schema_version':1,'mode':'bounded_scene_endpoint_checks','sampling_settings':asdict(settings),
                'source_scene_sha256':sampler.reference.scene_hash,'candidate_sha256':hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
                'candidate_path':str(args.candidate.resolve()),'mujoco_version':mujoco.__version__,
                'planned_samples':samples,'complete':False,'results':[],'status_counts':{},
                'path_verified':False,'physical_grasp_verified':False}
        save_report(args.output,report)
        try:
            for sample in samples:
                p=sampler.preview(sample);ik=p.solve()
                row={'sample':sample,'variation_source_sha256':p.variation_source_sha256,'status':classify(ik),'ik':ik}
                report['results'].append(row)
                report['status_counts']=dict(Counter(r['status'] for r in report['results']))
                save_report(args.output,report)
                print(f"{len(report['results'])}/{len(samples)} {sample['group']}: {row['status']}",flush=True)
        except KeyboardInterrupt:
            print('Interrupted; completed samples saved. No automatic retry.',flush=True)
            return 130
        report['complete']=True;save_report(args.output,report)
    print('Results:',report['status_counts'],'complete:',report['complete'],flush=True)
    if args.render_dir:render(sampler,report,args.render_dir)
    if args.view:view(sampler,report)
    return 0


if __name__=='__main__':raise SystemExit(main())
