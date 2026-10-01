"""Validated MTC geometric trajectory playback, strictly mj_forward only."""
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from .mujoco_scene import ARM_JOINTS, MujocoCapScene
from .arm_ik import body_transform, rotation_error
from .grasp_search import pose_arrays

ROOT = Path(__file__).resolve().parents[2]


class ApproachReplay:
    def __init__(self, path):
        self.path = Path(path)
        self.report = json.loads(self.path.read_text())
        report = self.report
        if (report.get('schema_version') != 1 or report.get('mode') != 'mtc_geometric_approach_not_execution'
                or report.get('dense_fcl_check_passed') is not True):
            raise ValueError('Not a validated MTC approach report')
        request = report['request']
        if request['arm_joints'] != list(ARM_JOINTS):
            raise ValueError('Arm joint order mismatch')
        for relative, expected in request['source_hashes'].items():
            path = (ROOT/relative).resolve()
            if not path.is_relative_to(ROOT) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError(f'Stale source: {relative}')
        # Check all source and generated geometry, not only manifest hashes.
        package = ROOT/'src/i10_rh56_moveit_config'
        provenance = json.loads((package/'config/provenance.json').read_text())
        if provenance['mujoco_version'] != mujoco.__version__:
            raise ValueError('MuJoCo version differs from planning model; use project .venv')
        for base, records in [(ROOT, provenance['sources']),
                              (package, json.loads((package/'config/generated_hashes.json').read_text()))]:
            for relative, expected in records.items():
                if hashlib.sha256((base/relative).read_bytes()).hexdigest() != expected:
                    raise ValueError(f'Stale planning geometry: {relative}')
        self.scene = MujocoCapScene()
        self.model, self.data = self.scene.model, self.scene.data
        self.scene.hand.apply(self.data, request['pregrasp_channels'])
        expected_hand = {self.model.joint(j).name: float(self.data.qpos[self.model.jnt_qposadr[j]])
                         for j in self.scene.hand.joint_ids}
        actual_hand = {name: value for name, value in request['initial_joint_positions'].items() if name not in ARM_JOINTS}
        if actual_hand.keys() != expected_hand.keys() or any(abs(actual_hand[name]-expected_hand[name]) > 1e-9 for name in expected_hand):
            raise ValueError('Hand preform mismatch')
        if len(report['stages']) != 2 or [s['name'] for s in report['stages']] != ['move to pregrasp', 'axial approach']:
            raise ValueError('Unexpected MTC stages')
        for name, q in request['initial_joint_positions'].items():
            self.data.joint(name).qpos[0] = q
        mujoco.mj_forward(self.model, self.data)
        self.initial = self.data.qpos.copy()
        self.addresses = np.array([self.model.joint(name).qposadr[0] for name in ARM_JOINTS])
        self.labels, points = [], []
        for stage in report['stages']:
            for q in stage['points']:
                points.append(q)
                self.labels.append(stage['name'])
        self.points = np.asarray(points, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 6 or len(points) < 2 or not np.isfinite(self.points).all():
            raise ValueError('Invalid trajectory points')
        if not np.allclose(self.points[0], self.initial[self.addresses], atol=1e-9, rtol=0):
            raise ValueError('Trajectory initial state differs from request')
        if np.max(np.abs(np.diff(self.points, axis=0))) > .02000001:
            raise ValueError('Trajectory discontinuity or insufficient sampling')
        # Playback display progress follows geometric joint distance, no physical times.
        length = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.progress = np.r_[0, np.cumsum(length)]
        if self.progress[-1] <= 0:
            raise ValueError('Empty motion')
        self.progress /= self.progress[-1]
        self.robot_bodies = set()
        root = self.model.body('base_link').id
        for b in range(self.model.nbody):
            parent = b
            while parent and parent != root:
                parent = int(self.model.body_parentid[parent])
            if parent == root:
                self.robot_bodies.add(b)
        self.set_progress(0)

    def set_progress(self, fraction):
        fraction = float(np.clip(fraction, 0, 1))
        right = min(len(self.points)-1, int(np.searchsorted(self.progress, fraction, side='right')))
        left = max(0, right-1)
        span = self.progress[right]-self.progress[left]
        weight = 0 if span == 0 else (fraction-self.progress[left])/span
        q = self.points[left]+weight*(self.points[right]-self.points[left])
        self.data.qpos[:] = self.initial
        self.data.qpos[self.addresses] = q
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        return self.labels[right]

    def validate(self):
        limits = np.array([self.model.joint(name).range for name in ARM_JOINTS])
        if np.any(self.points < limits[:, 0]-1e-8) or np.any(self.points > limits[:, 1]+1e-8):
            raise ValueError('MuJoCo joint limit violation')
        worst, pair = 0., None
        axis = np.asarray(self.report['request']['outward_axis_world'])
        origin = np.asarray(self.report['request']['pregrasp_position'])
        line_error, axial_positions = 0., []
        for index, q in enumerate(self.points):
            self.data.qpos[:] = self.initial
            self.data.qpos[self.addresses] = q
            mujoco.mj_forward(self.model, self.data)
            if self.labels[index] == 'axial approach':
                delta = body_transform(self.data, 'base')[:3, 3]-origin
                along = float(np.dot(delta, -axis))
                axial_positions.append(along)
                line_error = max(line_error, float(np.linalg.norm(delta+along*axis)))
            for contact in self.data.contact:
                bodies = self.model.geom_bodyid[contact.geom]
                if any(int(b) in self.robot_bodies for b in bodies) and -contact.dist > worst:
                    worst = float(-contact.dist)
                    pair = [self.model.geom(g).name for g in contact.geom]
        self.set_progress(1)
        actual = body_transform(self.data, 'base')
        request = self.report['request']
        error = float(np.linalg.norm(actual[:3, 3]-request['hand_position']))
        q = request['hand_orientation_xyzw']
        _, wanted_rotation = pose_arrays({'position_m': request['hand_position'], 'orientation_xyzw': q})
        orientation_error = float(np.linalg.norm(rotation_error(wanted_rotation, actual[:3, :3])))
        if error > 1e-4 or orientation_error > .002 or worst > 1e-4 or line_error > .0002:
            raise ValueError(f'MuJoCo validation failed: error={error}, penetration={worst}, pair={pair}')
        summary = {'mode': 'mujoco_kinematic_replay_not_dynamics', 'waypoints': len(self.points),
                   'stage_names': [s['name'] for s in self.report['stages']],
                   'maximum_robot_penetration_m': worst, 'worst_pair': pair,
                   'endpoint_position_error_m': error, 'endpoint_orientation_error_rad': orientation_error,
                   'hand_cap_door_unchanged': True,
                   'axial_max_lateral_error_m': line_error,
                   'axial_progress_start_end_m': [axial_positions[0], axial_positions[-1]],
                   'coverage_note': 'MuJoCo original enabled geometry; full planning geometry checked by FCL',
                   'trajectory_sha256': hashlib.sha256(self.path.read_bytes()).hexdigest()}
        fixed = np.ones(self.model.nq, dtype=bool);fixed[self.addresses] = False
        if not np.array_equal(self.data.qpos[fixed], self.initial[fixed]):
            raise ValueError('Non-arm joints changed')
        self.set_progress(0)
        return summary
