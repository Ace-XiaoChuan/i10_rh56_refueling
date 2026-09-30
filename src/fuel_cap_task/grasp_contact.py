"""Local fixed-object contact experiments with inherited, uncalibrated physics."""

from collections import Counter
from dataclasses import asdict, dataclass
import itertools
import math

import mujoco
import numpy as np

from .grasp_search import GraspSearchScene, distinct_candidates, pose_arrays, rotation


@dataclass(frozen=True)
class ContactSettings:
    timestep_s: float = 0.0005
    ramp_s: float = 1.0
    observe_s: float = 1.0
    tail_s: float = 0.2
    numerical_force_epsilon_n: float = 1e-6
    proposal_fractions: tuple = (0.4, 0.6, 0.8, 1.0)
    pregrasp_fractions: tuple = (0., 0.25, 0.5, 0.75)
    trials: int = 24
    outward_offsets_m: tuple = (0., 0.01, 0.02, 0.04, 0.06)

    def __post_init__(self):
        for name in ('timestep_s', 'ramp_s', 'observe_s', 'tail_s', 'numerical_force_epsilon_n'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.tail_s > self.observe_s or self.timestep_s > min(self.ramp_s, self.tail_s):
            raise ValueError('invalid integration/observation times')
        if type(self.trials) is not int or self.trials < 1:
            raise ValueError('trials must be positive integer')
        if not self.pregrasp_fractions or any(not 0 <= f < 1 for f in self.pregrasp_fractions):
            raise ValueError('pregrasp fractions must be in [0, 1)')
        if not self.outward_offsets_m or any(not math.isfinite(x) or x < 0 for x in self.outward_offsets_m):
            raise ValueError('outward offsets must be finite and nonnegative')
        if not self.proposal_fractions or any(not 0 < f <= 1 for f in self.proposal_fractions):
            raise ValueError('proposal fractions must be in (0, 1]')


class ContactExplorer:
    def __init__(self, settings=None):
        self.settings = settings or ContactSettings()
        self.scene = GraspSearchScene(fixture=True)
        self.scene.model.opt.timestep = self.settings.timestep_s
        self.scene.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    def observation(self):
        s = self.scene
        m, d = s.model, s.data
        cap_p, cap_r = s.cap_frame()
        forces = [0., 0., 0.]
        points = [[], [], []]
        others = []
        max_depth = 0.
        for index, contact in enumerate(d.contact):
            a, b = map(int, contact.geom)
            if not ({a, b} & s.hand_geoms):
                continue
            wrench = np.empty(6)
            mujoco.mj_contactForce(m, d, index, wrench)
            force = max(0., float(wrench[0]))
            max_depth = max(max_depth, -float(contact.dist))
            pair = {a, b}
            if s.grip in pair and any(f in pair for f in s.fingers):
                finger = next(i for i, g in enumerate(s.fingers) if g in pair)
                forces[finger] += force
                points[finger].append({'position_cap_m': (cap_r.T @ (contact.pos - cap_p)).tolist(),
                                       'normal_force_n': force, 'distance_m': float(contact.dist)})
            elif force > self.settings.numerical_force_epsilon_n:
                others.append({'geoms': [s.geom_label(a), s.geom_label(b)], 'normal_force_n': force,
                               'distance_m': float(contact.dist)})
        return {'terminal_normal_forces_n': forces, 'terminal_contacts': points,
                'other_loaded_contacts': others, 'max_actual_contact_depth_m': max_depth,
                'max_joint_speed_rad_s': float(np.max(np.abs(d.qvel)))}

    def trial(self, pose, pregrasp_channels=None):
        run = self.trial_steps(pose, pregrasp_channels)
        while True:
            try:
                next(run)
            except StopIteration as done:
                return done.value

    def trial_steps(self, pose, pregrasp_channels=None):
        """Yield periodically so a manual viewer can render and accept stop requests."""
        s, cfg = self.scene, self.settings
        m, d = s.model, s.data
        mujoco.mj_resetData(m, d)
        p, r = pose_arrays(pose)
        pregrasp = s.opened if pregrasp_channels is None else pregrasp_channels
        s.place(p, r, pregrasp)  # Initialization only. No qpos writes after this.
        initial = s.penetrations()
        if initial:
            return {'status': 'initial_interference', 'initial_penetrations': initial,
                    'physical_grasp_verified': False}
        start = s.hand.targets(pregrasp)
        target = s.hand.targets(s.closed)
        actuators = [c.actuator_id for c in s.hand.channels]
        addresses = [int(m.jnt_qposadr[c.joint_id]) for c in s.hand.channels]
        total = math.ceil((cfg.ramp_s + cfg.observe_s) / cfg.timestep_s)
        tail_start = cfg.ramp_s + cfg.observe_s - cfg.tail_s
        tail, trace = [], []
        initial_warnings = d.warning.number.copy()
        for step in range(total):
            alpha = min(1., (step + 1) * cfg.timestep_s / cfg.ramp_s)
            d.ctrl[actuators] = (1 - alpha) * start + alpha * target
            mujoco.mj_step(m, d)
            if (not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all()
                    or np.any(d.warning.number > initial_warnings)):
                return {'status': 'numerical_failure', 'time_s': float(d.time),
                        'warning_counts': d.warning.number.tolist(), 'physical_grasp_verified': False}
            if d.time >= tail_start or step % 100 == 0 or step == total - 1:
                # mj_step advances qpos after computing contacts; update derived
                # quantities before reading the current-state contact forces.
                mujoco.mj_forward(m, d)
                obs = self.observation()
                if d.time >= tail_start:
                    tail.append(obs)
                if step % 100 == 0 or step == total - 1:
                    trace.append({'time_s': float(d.time), 'qpos': d.qpos.tolist(), 'qvel': d.qvel.tolist(),
                                  'ctrl': d.ctrl.tolist(), 'terminal_normal_forces_n': obs['terminal_normal_forces_n'],
                                  'max_actual_contact_depth_m': obs['max_actual_contact_depth_m'],
                                  'max_joint_speed_rad_s': obs['max_joint_speed_rad_s']})
            if step % 20 == 0:
                yield None
        forces = np.array([row['terminal_normal_forces_n'] for row in tail])
        triple_contact = np.all(forces > cfg.numerical_force_epsilon_n, axis=1)
        final = tail[-1]
        # Observation only: even sustained contact is not force closure or grasp stability.
        return {'status': 'triple_contact_observed' if np.all(triple_contact) else 'insufficient_terminal_contact',
                'physical_grasp_verified': False, 'time_s': float(d.time),
                'pregrasp_channels': list(pregrasp),
                'initial_penetrations': [], 'tail_triple_contact_fraction': float(np.mean(triple_contact)),
                'tail_force_mean_n': forces.mean(axis=0).tolist(),
                'tail_force_min_n': forces.min(axis=0).tolist(),
                'tail_force_std_n': forces.std(axis=0).tolist(),
                'tail_max_joint_speed_rad_s': max(row['max_joint_speed_rad_s'] for row in tail),
                'tail_max_contact_depth_m': max(row['max_actual_contact_depth_m'] for row in tail),
                'driver_targets_rad': target.tolist(), 'actual_driver_positions_rad': d.qpos[addresses].tolist(),
                'driver_errors_rad': (target - d.qpos[addresses]).tolist(),
                'final_qpos': d.qpos.tolist(), 'final_qvel': d.qvel.tolist(), 'final_ctrl': d.ctrl.tolist(),
                'final_observation': final, 'trace': trace}

    def restore(self, candidate):
        """Restore the recorded actual state, not the empty-hand target shape."""
        s = self.scene
        mujoco.mj_resetData(s.model, s.data)
        s.place(*pose_arrays(candidate['cap_from_hand']), candidate.get('pregrasp_channels', s.opened))
        trial = candidate['contact_trial']
        if 'final_qpos' in trial:
            s.data.qpos[:] = trial['final_qpos']
            s.data.qvel[:] = trial['final_qvel']
            s.data.ctrl[:] = trial['final_ctrl']
            s.data.time = trial['time_s']
            mujoco.mj_forward(s.model, s.data)

    def physics_record(self):
        m = self.scene.model
        return {'source': 'inherited_model_parameters_uncalibrated',
                'actuator_gainprm': m.actuator_gainprm.tolist(), 'actuator_biasprm': m.actuator_biasprm.tolist(),
                'joint_actuator_force_limits': m.jnt_actfrcrange.tolist(),
                'joint_actuator_force_limited': m.jnt_actfrclimited.tolist(),
                'gravity_m_s2': m.opt.gravity.tolist(), 'body_mass_kg': m.body_mass.tolist(),
                'body_inertia_kg_m2': m.body_inertia.tolist(),
                'geom_friction': m.geom_friction.tolist(), 'geom_solref': m.geom_solref.tolist(),
                'geom_solimp': m.geom_solimp.tolist(), 'equality_solref': m.eq_solref.tolist(),
                'equality_solimp': m.eq_solimp.tolist(), 'joint_damping': m.dof_damping.tolist(),
                'joint_frictionloss': m.dof_frictionloss.tolist(), 'joint_armature': m.dof_armature.tolist(),
                'fixture': 'fixed_hand_root_cap_and_door_no_arm_no_unlock'}


def contact_search(explorer, *, keep=6, progress=None):
    s, cfg = explorer.scene, explorer.settings
    proposals = []
    evaluations = 0
    for fraction in cfg.proposal_fractions:
        channels = (1 - fraction) * np.array(s.opened) + fraction * np.array(s.closed)
        s.place(np.zeros(3), np.eye(3), channels)
        cap_p, cap_r = s.cap_frame()
        tips = np.array([cap_r.T @ (s.data.site(name).xpos - cap_p) for name in s.tips])
        mid = tips.mean(axis=0)
        y = tips[1] - tips[0]
        y /= np.linalg.norm(y)
        z = mid - y * (mid @ y)
        z /= np.linalg.norm(z)
        basis = np.column_stack((np.cross(y, z), y, z))
        pool = []
        def evaluate(p, r):
            nonlocal evaluations
            c = s.evaluate(p, r, channels)
            evaluations += 1
            # Empty-hand shape penetrations are proposal diagnostics, not grasp verdicts.
            c['status'] = 'contact_trial_pending'
            c['proposal_fraction'] = fraction
            pool.append(c)
            return c
        for side, roll, tilt, along, height in itertools.product(
                (-1, 1), np.linspace(0, 2 * math.pi, 12, endpoint=False),
                (-math.pi / 9, 0, math.pi / 9), (-0.6, 0, 0.6), (-0.6, 0, 0.6)):
            r = rotation(0, tilt) @ rotation(1, roll) @ np.diag([-side, side, -1]) @ basis.T
            target = np.array([along * s.half[0], 0, height * s.half[2]])
            evaluate(target - r @ mid, r)
        seeds = distinct_candidates(pool, 4)
        for seed in seeds:
            best = seed
            for step_m, step_deg in ((0.004, 10), (0.002, 5), (0.001, 2)):
                for axis in range(6):
                    for sign in (-1, 1):
                        p, r = pose_arrays(best['cap_from_hand'])
                        if axis < 3:
                            p[axis] += sign * step_m
                        else:
                            pivot = p + r @ mid
                            r = rotation(axis - 3, math.radians(sign * step_deg)) @ r
                            p = pivot - r @ mid
                        c = evaluate(p, r)
                        if c['score_m'] < best['score_m']:
                            best = c
        # Check starting geometry before choosing dynamics trials; closing target
        # penetration is allowed, starting in an obstacle is not teleported away.
        clear = []
        for c in sorted(pool, key=lambda c: c['score_m']):
            p, r = pose_arrays(c['cap_from_hand'])
            for outward in cfg.outward_offsets_m:
                adjusted = p + np.array([0., 0., outward])
                found = False
                for start_fraction in cfg.pregrasp_fractions:
                    if start_fraction > fraction:
                        continue
                    pregrasp = np.rint((1 - start_fraction) * np.array(s.opened)
                                       + start_fraction * np.array(s.closed)).astype(int)
                    s.place(adjusted, r, pregrasp)
                    if not s.penetrations():
                        proposal = s.evaluate(adjusted, r, channels)
                        proposal['status'] = 'contact_trial_pending'
                        proposal['proposal_fraction'] = fraction
                        proposal['outward_offset_m'] = outward
                        proposal['pregrasp_channels'] = pregrasp.tolist()
                        proposal['pregrasp_fraction'] = start_fraction
                        evaluations += 1
                        clear.append(proposal)
                        found = True
                        break
                if found:
                    break
        selected = distinct_candidates(clear, max(1, math.ceil(cfg.trials / len(cfg.proposal_fractions))))
        proposals.extend(selected)
        if progress:
            progress(f'proposal fraction {fraction:.1f}: {len(pool)} evaluated, {len(clear)} clear pregrasp starts, {len(selected)} selected')
    trials = []
    for index, proposal in enumerate(proposals[:cfg.trials]):
        trial = explorer.trial(proposal['cap_from_hand'], proposal['pregrasp_channels'])
        candidate = {'cap_from_hand': proposal['cap_from_hand'], 'score_m': proposal['score_m'],
                     'proposal_fraction': proposal['proposal_fraction'],
                     'pregrasp_channels': proposal['pregrasp_channels'],
                     'pregrasp_fraction': proposal['pregrasp_fraction'],
                     'proposal_shape_diagnostics': proposal['rejection_reasons'],
                     'status': trial['status'], 'contact_trial': trial}
        trials.append(candidate)
        if progress:
            progress(f"contact trial {index+1}/{min(len(proposals),cfg.trials)}: {trial['status']}")
    trials.sort(key=lambda c: (-c['contact_trial'].get('tail_triple_contact_fraction', 0),
                              c['contact_trial'].get('tail_max_joint_speed_rad_s', float('inf')), c['score_m']))
    for index, candidate in enumerate(trials):
        candidate['id'] = index
    return {'schema_version': 2, 'mode': 'local_contact_exploration_uncalibrated',
            'source_scene': str(s.path), 'source_scene_sha256': s.source_sha256, 'mujoco_version': mujoco.__version__,
            'grasp_channels': list(s.closed), 'exploration_open_channels': list(s.opened),
            'settings': asdict(cfg), 'proposal_settings': asdict(s.settings), 'physics': explorer.physics_record(),
            'evaluated_placements': evaluations, 'contact_trials': len(trials),
            'status_counts': dict(Counter(c['status'] for c in trials)), 'candidates': trials[:keep],
            'all_trial_summaries': [{'cap_from_hand': c['cap_from_hand'], 'status': c['status'],
                                     'proposal_fraction': c['proposal_fraction'],
                                     'pregrasp_channels': c['pregrasp_channels']} for c in trials],
            'conclusion': 'local_contact_observations_only', 'physical_grasp_verified': False,
            'unchecked': ['arm_endpoint_and_path', 'force_and_stability_acceptance', 'sensor_adapter_collision',
                          'hardware_calibration', 'free_cap_holding_slip_unlock_and_load']}
