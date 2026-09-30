#!/usr/bin/env python3
"""Explore fixed-target hand placements with local contact; no grasp acceptance."""

import argparse
import json
import os
from pathlib import Path
from queue import SimpleQueue
import struct
import sys
import time
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
# Offscreen rendering is explicit; an interactive viewer keeps its normal backend.
if '--render-dir' in sys.argv and '--view' not in sys.argv:
    os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np

from fuel_cap_task.grasp_search import GraspSearchScene, SearchSettings, pose_arrays, search
from fuel_cap_task.grasp_contact import ContactExplorer, ContactSettings, contact_search


def write_png(path, rgb):
    """Encode renderer RGB output using only the standard library."""
    height, width, _ = rgb.shape
    def chunk(kind, payload):
        return struct.pack('!I', len(payload)) + kind + payload + struct.pack('!I', zlib.crc32(kind + payload))
    rows = b''.join(b'\0' + row.tobytes() for row in rgb)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'
                     + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 2, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


def select(scene, candidate, fraction=1.):
    p, r = pose_arrays(candidate['cap_from_hand'])
    if 'contact_trial' in candidate:
        trial = candidate['contact_trial']
        mujoco.mj_resetData(scene.model, scene.data)
        scene.place(p, r, candidate.get('pregrasp_channels', scene.opened))
        if fraction > 0 and trial.get('trace'):
            row = trial['trace'][round(fraction * (len(trial['trace']) - 1))]
            scene.data.qpos[:] = row['qpos']
            scene.data.qvel[:] = row['qvel']
            scene.data.ctrl[:] = row['ctrl']
            scene.data.time = row['time_s']
            mujoco.mj_forward(scene.model, scene.data)
    else:
        channels = (1 - fraction) * np.array(scene.opened) + fraction * np.array(scene.closed)
        scene.place(p, r, channels)


def camera_settings(scene, cam, options):
    cam.lookat[:] = scene.data.site('cap_grip_center').xpos
    cam.distance = 0.42
    cam.azimuth = 90
    cam.elevation = -20
    options.geomgroup[0] = 0
    options.geomgroup[3] = 0
    options.frame = mujoco.mjtFrame.mjFRAME_NONE


def markers(scene, candidate, visual_scene):
    # Colored points are actual final contacts for contact trials; static
    # diagnostics use nearest grip points. Neither is a finger-pad calibration.
    cap_p, cap_r = scene.cap_frame()
    if 'contact_trial' in candidate:
        terminal = candidate['contact_trial'].get('final_observation', {}).get('terminal_contacts', [[], [], []])
        points = [[cap_p + cap_r @ np.array(c['position_cap_m']) for c in contacts] for contacts in terminal]
    else:
        points = [[np.array(c['grip_point_world_m'])] for c in candidate['contacts']]
    for positions, color in zip(points, ([0, 1, 0, 1], [1, 0.45, 0, 1])):
        for position in positions:
            if visual_scene.ngeom >= visual_scene.maxgeom:
                return
            geom = visual_scene.geoms[visual_scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.002] * 3),
                               position, np.eye(3).ravel(), np.array(color, dtype=np.float32))
            visual_scene.ngeom += 1


def render(scene, report, folder):
    folder.mkdir(parents=True, exist_ok=True)
    candidate = report['candidates'][0]
    select(scene, candidate)
    options, cam = mujoco.MjvOption(), mujoco.MjvCamera()
    camera_settings(scene, cam, options)
    scene.model.vis.global_.offwidth = 800
    scene.model.vis.global_.offheight = 640
    with mujoco.Renderer(scene.model, height=640, width=800) as renderer:
        for name, azimuth, elevation in [('front', 90, -15), ('side', 140, -15), ('top', 90, -60)]:
            cam.azimuth, cam.elevation = azimuth, elevation
            renderer.update_scene(scene.data, camera=cam, scene_option=options)
            markers(scene, candidate, renderer.scene)
            write_png(folder / f'{name}.png', renderer.render())
    status = candidate['status']
    reasons = ', '.join(candidate.get('rejection_reasons', candidate.get('proposal_shape_diagnostics', []))) or 'none'
    (folder / 'index.html').write_text(
        '<!doctype html><meta charset="utf-8"><title>Grasp geometry diagnostic</title>'
        '<style>body{font:16px sans-serif;margin:24px;background:#eee}img{max-width:48%;margin:4px}</style>'
        f'<h1>Candidate 0 — {status}</h1><p>Uncalibrated model exploration. No grasp acceptance or arm reachability validation.</p>'
        f'<p>Proposal shape diagnostics (not a physical verdict): {reasons}</p><p>Green: thumb. Orange: index. Contact mode shows recorded actual state and contacts.</p>'
        + ''.join(f'<img src="{name}.png" alt="{name} view">' for name in ('front', 'side', 'top')),
        encoding='utf-8')


def apply_extra_door_opening(scene, degrees):
    if not degrees:
        return
    body = scene.model.body("fuel_door").id
    angle = np.deg2rad(float(degrees))
    extra = np.array([np.cos(angle / 2), 0., np.sin(angle / 2), 0.])
    result = np.empty(4)
    mujoco.mju_mulQuat(result, scene.model.body_quat[body], extra)
    scene.model.body_quat[body] = result
    mujoco.mj_forward(scene.model, scene.data)

def view(scene, report):
    import mujoco.viewer
    keys = SimpleQueue()
    index, fraction = 0, 1.
    candidates = report['candidates']
    select(scene, candidates[index])
    print('N/P: next/previous candidate | O/G: pregrasp/end state | [/]: recorded sample | C: collision geometry', flush=True)
    def describe():
        candidate = candidates[index]
        print(f"candidate {index}: {candidate['status']}; closure fraction={fraction:.2f}; "
              f"mode={report['mode']}", flush=True)
    describe()
    with mujoco.viewer.launch_passive(scene.model, scene.data, key_callback=keys.put) as viewer:
        with viewer.lock():
            camera_settings(scene, viewer.cam, viewer.opt)
        while viewer.is_running():
            with viewer.lock():
                changed = False
                while not keys.empty():
                    key = keys.get()
                    if key == ord('N'):
                        index = (index + 1) % len(candidates)
                    elif key == ord('P'):
                        index = (index - 1) % len(candidates)
                    elif key == ord('O'):
                        fraction = 0.
                    elif key == ord('G'):
                        fraction = 1.
                    elif key in (ord('['), ord(']')):
                        fraction = float(np.clip(fraction + (1 if key == ord(']') else -1)
                                                 / (scene.settings.closure_samples - 1), 0, 1))
                    elif key == ord('C'):
                        viewer.opt.geomgroup[0] = 1 - viewer.opt.geomgroup[0]
                        viewer.opt.geomgroup[3] = 1 - viewer.opt.geomgroup[3]
                    changed = True
                select(scene, candidates[index], fraction)
                viewer.user_scn.ngeom = 0
                if fraction == 1:
                    markers(scene, candidates[index], viewer.user_scn)
                if changed:
                    describe()
            viewer.sync()
            time.sleep(1 / 60)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/grasp_search/report.json'))
    parser.add_argument('--load-report', type=Path, help='inspect a saved report without searching again')
    parser.add_argument('--keep', type=int, default=6)
    parser.add_argument('--view', action='store_true')
    parser.add_argument('--static-diagnostic', action='store_true', help='old target-shape diagnostic only; no grasp verdict')
    parser.add_argument('--trials', type=int, default=24)
    parser.add_argument('--render-dir', type=Path)
    parser.add_argument('--door-extra-open-deg', type=float, default=0., help='extra visual door opening angle')
    args = parser.parse_args()
    if args.keep < 1 or args.trials < 1:
        parser.error('--keep and --trials must be positive')
    if args.load_report:
        report = json.loads(args.load_report.read_text())
        if report.get('schema_version') not in (1, 2) or not report.get('candidates'):
            parser.error('unsupported or empty report')
        if report['schema_version'] == 2:
            scene = ContactExplorer(ContactSettings(**report['settings'])).scene
        else:
            scene = GraspSearchScene(settings=SearchSettings(**report['settings']))
        if (report['source_scene_sha256'] != scene.source_sha256
                or report['mujoco_version'] != mujoco.__version__
                or report['grasp_channels'] != list(scene.closed)):
            parser.error('report model/version/grasp no longer matches; rerun search')
    else:
        if args.static_diagnostic:
            scene = GraspSearchScene()
            report = search(scene, keep=args.keep, progress=lambda text: print(text, flush=True))
        else:
            explorer = ContactExplorer(ContactSettings(trials=args.trials))
            scene = explorer.scene
            report = contact_search(explorer, keep=args.keep, progress=lambda text: print(text, flush=True))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(f'report: {args.output.resolve()}', flush=True)
    apply_extra_door_opening(scene, args.door_extra_open_deg)
    print(f"{report['conclusion']}: {report['evaluated_placements']} placements; "
          f"contact counts={report.get('status_counts', 'static target-shape diagnostics only')}", flush=True)
    print('All poses refer to hand base in the cap task frame; no task parameters were updated.', flush=True)
    if not report['candidates']:
        print('No clear pregrasp placements selected; expand proposal search before contact trials.', flush=True)
        return
    if args.render_dir:
        render(scene, report, args.render_dir)
        print(f'images: {args.render_dir.resolve()}', flush=True)
    if args.view:
        view(scene, report)


if __name__ == '__main__':
    main()
