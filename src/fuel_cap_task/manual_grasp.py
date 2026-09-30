"""Manual fixed-fixture experiments; saved poses are candidates, not accepted grasps."""
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path

import mujoco
import numpy as np

from .grasp_contact import ContactExplorer
from .grasp_search import pose_arrays, quaternion


class ManualGrasp:
    def __init__(self, explorer=None):
        self.explorer = explorer or ContactExplorer()
        self.scene = self.explorer.scene
        self.run = None
        self.result = None
        self.channels = np.rint((np.array(self.scene.opened) + self.scene.closed) / 2).astype(int).tolist()
        s = self.scene
        s.place(np.zeros(3), np.eye(3), self.channels)
        mujoco.mj_forward(s.model, s.data)
        cp, cr = s.cap_frame()
        tips = np.array([cr.T @ (s.data.site(n).xpos - cp) for n in s.tips])
        mid = tips.mean(axis=0)
        y = tips[1] - tips[0]
        y /= np.linalg.norm(y)
        z = mid - y * (mid @ y)
        z /= np.linalg.norm(z)
        basis = np.column_stack((np.cross(y, z), y, z))
        r = np.diag([-1., 1., -1.]) @ basis.T
        self.pose = {'position_m': (-r @ mid + [0, 0, .06]).tolist(),
                     'orientation_xyzw': quaternion(r)}
        self.reset()

    def reset(self, pose=None, channels=None):
        new_pose = self.pose if pose is None else pose
        p, r = pose_arrays(new_pose)
        new_channels = self.channels if channels is None else channels
        self.scene.hand.targets(new_channels)  # Validate before touching the current experiment.
        if any(float(x) != int(x) for x in new_channels):
            raise ValueError('Pregrasp channels must be integers')
        self.stop()
        self.pose = {'position_m': p.tolist(), 'orientation_xyzw': quaternion(r)}
        self.channels = [int(x) for x in new_channels]
        s = self.scene
        mujoco.mj_resetData(s.model, s.data)
        s.place(p, r, self.channels)
        mujoco.mj_forward(s.model, s.data)
        self.result = None
        self.status = 'pregrasp'
        self.initial_penetrations = s.penetrations()

    def start(self):
        if self.run is not None:
            return
        self.reset()
        self.run = self.explorer.trial_steps(self.pose, self.channels)
        self.status = 'closing'

    def advance(self):
        if self.run is None:
            return
        try:
            next(self.run)
        except StopIteration as done:
            self.result = done.value
            self.status = self.result['status']
            self.run = None
        except Exception:
            self.stop()
            self.status = 'experiment_error'
            raise
        mujoco.mj_forward(self.scene.model, self.scene.data)

    def stop(self):
        if self.run is not None:
            self.run.close()
            self.run = None
            self.status = 'interrupted'
            self.result = None

    def report(self):
        s = self.scene
        return {'schema_version': 1, 'mode': 'manual_grasp_candidate',
                'created_utc': datetime.now(timezone.utc).isoformat(),
                'source_scene': str(s.path), 'source_scene_sha256': s.source_sha256,
                'mujoco_version': mujoco.__version__, 'grasp_channels': list(s.closed),
                'cap_from_hand': self.pose, 'pregrasp_channels': self.channels,
                'status': self.status, 'physical_grasp_verified': False,
                'initial_penetrations': self.initial_penetrations,
                'settings': asdict(self.explorer.settings), 'physics': self.explorer.physics_record(),
                'contact_trial': self.result,
                'current_state': {'time_s': float(s.data.time), 'qpos': s.data.qpos.tolist(),
                                  'qvel': s.data.qvel.tolist(), 'ctrl': s.data.ctrl.tolist()},
                'observation': self.explorer.observation()}

    def save(self, directory):
        if self.run is not None:
            raise ValueError('Pause or finish the experiment before saving')
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (datetime.now(timezone.utc).strftime('candidate_%Y%m%dT%H%M%S_%fZ') + '.json')
        payload = json.dumps(self.report(), indent=2, allow_nan=False) + '\n'
        with path.open('x') as f:
            f.write(payload)
        return path

    def load(self, path):
        report = json.loads(Path(path).read_text())
        if (report.get('schema_version') != 1 or report.get('mode') != 'manual_grasp_candidate'
                or report.get('source_scene_sha256') != self.scene.source_sha256
                or report.get('mujoco_version') != mujoco.__version__
                or report.get('grasp_channels') != list(self.scene.closed)):
            raise ValueError('Incompatible candidate format, model, MuJoCo version or final target')
        # Loading is a new pregrasp setup, never a successful-trial replay.
        self.reset(report['cap_from_hand'], report['pregrasp_channels'])
