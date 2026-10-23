"""Geometric grasp exploration, never an Executor or physical grasp verdict."""

from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .interfaces import TaskParameters
from .mujoco_scene import DEFAULT_SCENE, HandPreviewMapping


@dataclass(frozen=True)
class SearchSettings:
    """Software exploration assumptions, not calibrated tolerances."""
    contact_gap_m: float = 0.001
    penetration_m: float = 0.0001
    tip_region_m: float = 0.012
    edge_margin_m: float = 0.001
    roll_samples: int = 12
    refinement_seeds: int = 8
    closure_samples: int = 21

    def __post_init__(self):
        for name in ('contact_gap_m', 'penetration_m', 'tip_region_m', 'edge_margin_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('roll_samples', 'refinement_seeds', 'closure_samples'):
            value = getattr(self, name)
            if type(value) is not int or value < 2:
                raise ValueError(f'{name} must be an integer >= 2')


def rotation(axis, angle):
    vector = np.zeros(3)
    vector[axis] = 1
    cross = np.array([[0, -vector[2], vector[1]], [vector[2], 0, -vector[0]],
                      [-vector[1], vector[0], 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def quaternion(matrix):
    wxyz = np.empty(4)
    mujoco.mju_mat2Quat(wxyz, np.ascontiguousarray(matrix).ravel())
    return wxyz[[1, 2, 3, 0]].tolist()


def pose_arrays(pose):
    position = np.asarray(pose['position_m'], dtype=float)
    xyzw = np.asarray(pose['orientation_xyzw'], dtype=float)
    if (position.shape != (3,) or xyzw.shape != (4,) or not np.isfinite(position).all()
            or not np.isfinite(xyzw).all() or abs(np.linalg.norm(xyzw) - 1) > 1e-6):
        raise ValueError('pose requires finite position and a unit XYZW quaternion')
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, xyzw[[3, 0, 1, 2]])
    return position, matrix.reshape(3, 3)


class GraspSearchScene:
    """In-memory detached assembly, preserving original geometry and joints.

    Only this derived scene disables environment/environment contacts to reduce
    redundant cap/tank contacts. Hand/environment and hand self contacts remain.
    """

    def __init__(self, path: Path = DEFAULT_SCENE, settings: SearchSettings | None = None, *, fixture=False):
        self.settings = settings or SearchSettings()
        self.path = Path(path).resolve()
        self.source_sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        root = ET.parse(self.path).getroot()
        compiler = root.find('compiler')
        meshdir = compiler.get('meshdir', '')
        for mesh in root.findall('./asset/mesh'):
            mesh.set('file', str((self.path.parent / meshdir / mesh.get('file')).resolve()))
        compiler.attrib.pop('meshdir', None)
        if fixture:
            for name in ('cap_body', 'fuel_door'):
                body = root.find(f".//body[@name='{name}']")
                joint = body.find('joint')
                # The door body already encodes its initial opening in euler.
                body.remove(joint)
        world = root.find('worldbody')
        arm = world.find("body[@name='inspection_arm_mount']")
        sensor = arm.find(".//body[@name='sensor_link']")
        parent = next(node for node in arm.iter() if sensor in list(node))
        parent.remove(sensor)
        world.remove(arm)
        removed_bodies = {body.get('name') for body in arm.iter('body')}
        for contact in root.findall('contact'):
            for exclude in list(contact.findall('exclude')):
                if {exclude.get('body1'), exclude.get('body2')} & removed_bodies:
                    contact.remove(exclude)
        carrier = ET.SubElement(world, 'body', name='grasp_search_carrier', mocap='true')
        carrier.append(sensor)
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        self.hand = HandPreviewMapping(self.model)
        self.closed = TaskParameters().grasp_channels
        self.opened = (1000,) * 6  # Explicitly authorized exploration preset.
        self.hand.apply(self.data, self.closed)
        self.hand_bodies = set()
        base = self.model.body('base').id
        for body in range(self.model.nbody):
            ancestor = body
            while ancestor and ancestor != base:
                ancestor = int(self.model.body_parentid[ancestor])
            if ancestor == base:
                self.hand_bodies.add(body)
        self.hand_geoms = {g for g in range(self.model.ngeom)
                           if self.model.geom_bodyid[g] in self.hand_bodies
                           and (self.model.geom_contype[g] or self.model.geom_conaffinity[g])}
        for geom in range(self.model.ngeom):
            if self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]:
                self.model.geom_contype[geom] = 1 if geom in self.hand_geoms else 2
                self.model.geom_conaffinity[geom] = 3 if geom in self.hand_geoms else 1
        self.grip = self.model.geom('cap_body_collision_2').id
        # The frozen grasp uses thumb, index, and middle finger contact.
        self.fingers = [self._collision_geom(name) for name in ('thumb_distal', 'index_intermediate', 'middle_intermediate')]
        self.tips = ['right_thumb_tip', 'right_index_tip', 'right_middle_tip']
        self.cap_site = self.model.site('cap_grip_center').id
        # 盒体几何的长边尺寸存于第二项，按盖体系的长、宽、高排列。
        if self.model.geom_type[self.grip] != mujoco.mjtGeom.mjGEOM_BOX:
            raise ValueError('search requires the current box grip geometry')
        self.half = self.model.geom_size[self.grip][[1, 0, 2]].copy()
        if self.settings.edge_margin_m >= min(self.half[0], self.half[2]):
            raise ValueError('edge margin leaves no usable grip face')
        mujoco.mj_forward(self.model, self.data)
        carrier_data = self.data.body('grasp_search_carrier')
        carrier_r = carrier_data.xmat.reshape(3, 3)
        self.carrier_from_hand_p = carrier_r.T @ (self.data.body('base').xpos - carrier_data.xpos)
        self.carrier_from_hand_r = carrier_r.T @ self.data.body('base').xmat.reshape(3, 3)
        hand_p = self.data.body('base').xpos
        hand_r = self.data.body('base').xmat.reshape(3, 3)
        self.tip_local = np.array([hand_r.T @ (self.data.site(site).xpos - hand_p) for site in self.tips])
        self.mocap = int(self.model.body('grasp_search_carrier').mocapid[0])

    def _collision_geom(self, body_name):
        body = self.model.body(body_name).id
        geoms = [g for g in self.hand_geoms if self.model.geom_bodyid[g] == body]
        if len(geoms) != 1:
            raise ValueError(f'{body_name}: expected one terminal collision geom')
        return geoms[0]

    def cap_frame(self):
        """直接读取统一盖体系在世界中的位姿；X 沿握梁，Z 朝外。"""
        return (self.data.site_xpos[self.cap_site].copy(),
                self.data.site_xmat[self.cap_site].reshape(3, 3).copy())

    def place(self, position, matrix, channels=None):
        # mj_resetData clears derived site poses; refresh before using cap axes.
        mujoco.mj_kinematics(self.model, self.data)
        cap_p, cap_r = self.cap_frame()
        hand_p, hand_r = cap_p + cap_r @ position, cap_r @ matrix
        carrier_r = hand_r @ self.carrier_from_hand_r.T
        carrier_p = hand_p - carrier_r @ self.carrier_from_hand_p
        xyzw = quaternion(carrier_r)
        self.data.mocap_pos[self.mocap] = carrier_p
        self.data.mocap_quat[self.mocap] = np.array(xyzw)[[3, 0, 1, 2]]
        self.hand.apply(self.data, self.closed if channels is None else channels)

    def geom_label(self, geom):
        return self.model.geom(geom).name or f'{self.model.body(self.model.geom_bodyid[geom]).name}/geom{geom}'

    def penetrations(self):
        pairs = {}
        for contact in self.data.contact:
            a, b = map(int, contact.geom)
            if not ({a, b} & self.hand_geoms) or contact.dist >= -self.settings.penetration_m:
                continue
            key = tuple(sorted((a, b)))
            pairs[key] = min(pairs.get(key, 0), float(contact.dist))
        return [{'geoms': [self.geom_label(a), self.geom_label(b)], 'distance_m': distance,
                 'kind': 'hand_self' if a in self.hand_geoms and b in self.hand_geoms else 'hand_environment'}
                for (a, b), distance in sorted(pairs.items(), key=lambda item: item[1])]

    def measure(self):
        cap_p, cap_r = self.cap_frame()
        contacts = []
        for geom, site in zip(self.fingers, self.tips):
            segment = np.zeros(6)
            distance = float(mujoco.mj_geomDistance(self.model, self.data, geom, self.grip, 1.0, segment))
            if distance >= 1:
                raise ValueError('candidate outside distance query range')
            hand_point, cap_point = segment[:3], segment[3:]
            local = cap_r.T @ (cap_point - cap_p)
            side = 1 if local[1] >= 0 else -1
            face_error = abs(abs(local[1]) - self.half[1])
            edge_overrun = max(0., abs(local[0]) - (self.half[0] - self.settings.edge_margin_m),
                               abs(local[2]) - (self.half[2] - self.settings.edge_margin_m))
            tip_distance = float(np.linalg.norm(hand_point - self.data.site(site).xpos))
            contacts.append({'finger': site, 'distance_m': distance,
                             'hand_point_world_m': hand_point.tolist(),
                             'grip_point_world_m': cap_point.tolist(),
                             'grip_point_cap_m': local.tolist(), 'side': side,
                             'side_face_error_m': float(face_error), 'edge_overrun_m': float(edge_overrun),
                             'tip_region_distance_m': tip_distance})
        return contacts

    def evaluate(self, position, matrix, channels=None):
        self.place(position, matrix, channels)
        contacts = self.measure()
        collisions = self.penetrations()
        s = self.settings
        reasons = []
        if any(c['distance_m'] > s.contact_gap_m for c in contacts):
            reasons.append('fingers_not_both_near_grip')
        if any(c['distance_m'] < -s.penetration_m for c in contacts):
            reasons.append('finger_grip_penetration')
        if contacts[0]['side'] == contacts[1]['side'] or contacts[0]['side'] == contacts[2]['side']:
            reasons.append('contacts_not_on_opposite_sides')
        if any(c['side_face_error_m'] > s.penetration_m or c['edge_overrun_m'] > s.penetration_m
               for c in contacts):
            reasons.append('contact_outside_usable_side_faces')
        if any(c['tip_region_distance_m'] > s.tip_region_m for c in contacts):
            reasons.append('contact_outside_tip_proxy_region')
        if any(c['kind'] == 'hand_self' for c in collisions):
            reasons.append('hand_self_penetration')
        if any(c['kind'] == 'hand_environment' for c in collisions):
            reasons.append('hand_environment_penetration')
        # Metre-scale diagnostic objective; self intersections are invariant
        # under rigid placement and remain explicit rejection reasons.
        score = sum(abs(c['distance_m']) + 2 * c['side_face_error_m'] + 2 * c['edge_overrun_m']
                    + max(0, c['tip_region_distance_m'] - s.tip_region_m) for c in contacts)
        score += 0.02 * (contacts[0]['side'] == contacts[1]['side'])
        score += 0.02 * (contacts[0]['side'] == contacts[2]['side'])
        score += 3 * sum(-c['distance_m'] for c in collisions if c['kind'] == 'hand_environment')
        return {'cap_from_hand': {'parent_frame': 'cap_grip_task', 'child_frame': 'base',
                                 'position_m': np.asarray(position).tolist(), 'orientation_xyzw': quaternion(matrix)},
                'score_m': float(score), 'status': 'rejected' if reasons else 'geometric_candidate',
                'rejection_reasons': reasons, 'contacts': contacts, 'penetrations': collisions}

    def closure(self, pose):
        position, matrix = pose_arrays(pose)
        samples = []
        try:
            for alpha in np.linspace(0, 1, self.settings.closure_samples):
                channels = (1 - alpha) * np.array(self.opened) + alpha * np.array(self.closed)
                result = self.evaluate(position, matrix, channels)
                samples.append({'fraction': float(alpha), 'channels': channels.tolist(),
                                'distances_m': [c['distance_m'] for c in result['contacts']],
                                'rejection_reasons': result['rejection_reasons'],
                                'penetrations': result['penetrations']})
        finally:
            self.place(position, matrix)
        first_contact = next((row['fraction'] for row in samples if min(row['distances_m']) <= 0), None)
        first_penetration = next((row['fraction'] for row in samples if row['penetrations']), None)
        both_near = next((row['fraction'] for row in samples
                         if all(-self.settings.penetration_m <= d <= self.settings.contact_gap_m
                                for d in row['distances_m'])), None)
        return {'mode': 'sampled_kinematics_no_contact_response', 'first_terminal_contact_fraction': first_contact,
                'first_forbidden_penetration_fraction': first_penetration,
                'first_all_terminals_near_fraction': both_near, 'samples': samples,
                'physical_closure_verified': False}


def distinct_candidates(candidates, limit):
    result = []
    for candidate in sorted(candidates, key=lambda item: (item['status'] != 'geometric_candidate', item['score_m'])):
        p, r = pose_arrays(candidate['cap_from_hand'])
        if any(np.linalg.norm(p - pose_arrays(other['cap_from_hand'])[0]) < 0.003
               and np.trace(r.T @ pose_arrays(other['cap_from_hand'])[1]) > 1 + 2 * math.cos(math.radians(8))
               for other in result):
            continue
        result.append(candidate)
        if len(result) == limit:
            break
    return result


def search(scene, *, keep=6, progress=None):
    if type(keep) is not int or keep < 1:
        raise ValueError('keep must be a positive integer')
    s = scene.settings
    mid = scene.tip_local.mean(axis=0)
    y = scene.tip_local[1] - scene.tip_local[0]
    y /= np.linalg.norm(y)
    z = mid - y * (mid @ y)
    z /= np.linalg.norm(z)
    basis = np.column_stack((np.cross(y, z), y, z))
    pool = []
    counts = Counter()
    evaluated = 0
    feasible = 0

    def evaluate(p, r):
        nonlocal evaluated, feasible
        candidate = scene.evaluate(p, r)
        evaluated += 1
        feasible += candidate['status'] == 'geometric_candidate'
        counts.update(candidate['rejection_reasons'])
        pool.append(candidate)
        return candidate

    for side, roll, tilt, along, height in itertools.product(
            (-1, 1), np.linspace(0, 2 * math.pi, s.roll_samples, endpoint=False),
            (-math.pi / 9, 0, math.pi / 9), (-0.6, 0, 0.6), (-0.6, 0, 0.6)):
        target_basis = np.diag([-side, side, -1])
        r = rotation(0, tilt) @ rotation(1, roll) @ target_basis @ basis.T
        target = np.array([along * scene.half[0], 0, height * scene.half[2]])
        evaluate(target - r @ mid, r)
    seeds = distinct_candidates(pool, s.refinement_seeds)
    if progress:
        progress(f'coarse placements: {evaluated}; refining {len(seeds)} seeds')
    for seed in seeds:
        best = seed
        for translation_step, angle_step in ((0.004, 10), (0.002, 5), (0.001, 2), (0.0005, 1)):
            for _ in range(2):
                improved = False
                for axis in range(6):
                    for sign in (-1, 1):
                        p, r = pose_arrays(best['cap_from_hand'])
                        if axis < 3:
                            p[axis] += sign * translation_step
                        else:
                            # Rotate about the pinch midpoint, not the remote wrist.
                            pivot = p + r @ mid
                            r = rotation(axis - 3, math.radians(sign * angle_step)) @ r
                            p = pivot - r @ mid
                        trial = evaluate(p, r)
                        if trial['score_m'] < best['score_m']:
                            best = trial
                            improved = True
                if not improved:
                    break
    candidates = distinct_candidates(pool, keep)
    for index, candidate in enumerate(candidates):
        candidate['id'] = index
        candidate['closure'] = scene.closure(candidate['cap_from_hand'])
    # Rigid placement cannot repair self penetration of the frozen empty-hand pose.
    scene.place(np.array([0, 0, 0.5]), np.eye(3))
    baseline = [c for c in scene.penetrations() if c['kind'] == 'hand_self']
    return {'schema_version': 1, 'mode': 'geometric_search_no_physics',
            'interpretation': 'static_target_shape_only_not_a_grasp_feasibility_verdict',
            'source_scene': str(scene.path), 'source_scene_sha256': scene.source_sha256,
            'mujoco_version': mujoco.__version__, 'mapping_source': 'model_ranges_uncalibrated',
            'grasp_channels': list(scene.closed), 'exploration_open_channels': list(scene.opened),
            'settings': asdict(s), 'grip_half_extents_task_m': scene.half.tolist(),
            'frozen_tip_site_separation_m': [float(np.linalg.norm(scene.tip_local[i] - scene.tip_local[0])) for i in (1, 2)],
            'frozen_hand_self_penetrations': baseline,
            'evaluated_placements': evaluated, 'geometrically_feasible_evaluations': feasible,
            'rejection_counts': dict(counts), 'candidates': candidates,
            'conclusion': 'candidates_require_validation' if feasible else 'no_geometric_candidate_found',
            'unchecked': ['anatomical_finger_pad_calibration', 'sensor_adapter_collision',
                          'continuous_approach_and_closure', 'arm_endpoint_ik_path_and_full_unlock_rotation',
                          'contact_response_grip_stability_and_loads', 'hardware_calibration']}
