#!/usr/bin/env python3
"""Derive the P0 MoveIt model from the current compiled scene; no dynamics.

Run with the project .venv. JSON configuration files are also valid YAML and
can be consumed by yaml-cpp / ROS launch without adding PyYAML to this venv.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.arm_ik import ArmIKPreview, body_transform
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP
from fuel_cap_task.mujoco_scene import ARM_JOINTS

PACKAGE = ROOT / 'src/i10_rh56_moveit_config'
CANDIDATE = DEFAULT_FIXED_GRASP
IK_REPORT = ROOT / 'config/arm_ik_current.json'


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def numbers(values):
    return ' '.join(format(float(x), '.17g') for x in values)


def matrix(quat):
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, np.asarray(quat, dtype=float))
    return result.reshape(3, 3)


def rpy(quat):
    transform = np.eye(4)
    transform[:3, :3] = matrix(quat)
    return trimesh.transformations.euler_from_matrix(transform, axes='sxyz')


def origin(element, pos, quat):
    ET.SubElement(element, 'origin', xyz=numbers(pos), rpy=numbers(rpy(quat)))


def write_xml(path, tree):
    ET.indent(tree)
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)


def generate(output):
    p = ArmIKPreview(CANDIDATE)
    m, d = p.model, p.data
    config, meshes = output / 'config', output / 'meshes'
    config.mkdir(parents=True, exist_ok=True)
    meshes.mkdir(parents=True, exist_ok=True)
    source_arm = ET.parse(ROOT / 'assets/robots/aubo_i10/urdf/aubo_i10_JY.urdf').getroot()
    arm_limits = {j.get('name'): j.find('limit').attrib for j in source_arm.findall('joint') if j.find('limit') is not None}
    robot_bodies = sorted(p.robot_bodies)
    source_differences = []
    for name in ARM_JOINTS:
        j = m.joint(name).id
        b = int(m.jnt_bodyid[j])
        source = source_arm.find(f"joint[@name='{name}']")
        pos = np.fromstring(source.find('origin').get('xyz'), sep=' ')
        angles = np.fromstring(source.find('origin').get('rpy'), sep=' ')
        source_rot = trimesh.transformations.euler_matrix(*angles)[:3, :3]
        axis = np.fromstring(source.find('axis').get('xyz'), sep=' ')
        errors = {'joint': name, 'translation_max_abs_m': float(np.max(np.abs(pos-m.body_pos[b]))),
                  'rotation_matrix_max_abs': float(np.max(np.abs(source_rot-matrix(m.body_quat[b])))),
                  'axis_max_abs': float(np.max(np.abs(axis-m.jnt_axis[j]))),
                  'limit_max_abs_rad': float(np.max(np.abs(np.array([float(arm_limits[name][k]) for k in ('lower','upper')])-m.jnt_range[j])))}
        if max(errors[k] for k in errors if k != 'joint') > 1e-5:
            raise ValueError(f'URDF/MJCF disagreement beyond serialization tolerance: {errors}')
        source_differences.append(errors)

    # Use the existing system SciPy in a separate process, not mixed Python ABIs.
    collision_mesh_ids = sorted({int(m.geom_dataid[g]) for g in range(m.ngeom)
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and (m.geom_contype[g] or m.geom_conaffinity[g])})
    vertices = {str(mid): m.mesh_vert[int(m.mesh_vertadr[mid]):int(m.mesh_vertadr[mid]+m.mesh_vertnum[mid])].tolist()
                for mid in collision_mesh_ids}
    hull_code = """
import json, sys
import numpy as np
from scipy.spatial import ConvexHull
result = {}
for key, vertices in json.load(sys.stdin).items():
    points = np.asarray(vertices)
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    normals = np.cross(points[faces[:,1]]-points[faces[:,0]], points[faces[:,2]]-points[faces[:,0]])
    flip = np.sum(normals*hull.equations[:,:3], axis=1) < 0
    faces[flip] = faces[flip][:,[0,2,1]]
    result[key] = faces.tolist()
json.dump(result, sys.stdout)
"""
    hull_faces = json.loads(subprocess.run(['/usr/bin/python3','-c',hull_code],
        input=json.dumps(vertices),text=True,capture_output=True,check=True).stdout)
    mesh_cache = {}
    mesh_audit = []
    def mesh_uri(g, hull):
        mid = int(m.geom_dataid[g])
        key = (mid, hull)
        if key not in mesh_cache:
            va, vn = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
            fa, fn = int(m.mesh_faceadr[mid]), int(m.mesh_facenum[mid])
            mesh = trimesh.Trimesh(m.mesh_vert[va:va+vn].copy(), m.mesh_face[fa:fa+fn].copy(), process=False)
            if hull:
                mesh = trimesh.Trimesh(mesh.vertices.copy(), np.asarray(hull_faces[str(mid)]), process=False)
            filename = f'mesh_{mid:03d}_{"hull" if hull else "surface"}.stl'
            mesh.export(meshes / filename)
            mesh_cache[key] = f'package://i10_rh56_moveit_config/meshes/{filename}'
            path = m.paths[int(m.mesh_pathadr[mid]):].split(b'\0')[0].decode() if m.mesh_pathadr[mid] >= 0 else None
            mesh_audit.append({'file': filename, 'compiled_mesh': m.mesh(mid).name,
                               'source_path': path, 'convex_hull': hull, 'triangles': len(mesh.faces)})
        return mesh_cache[key]

    def shape(g, hull=True):
        t, size = int(m.geom_type[g]), m.geom_size[g]
        if t == mujoco.mjtGeom.mjGEOM_MESH:
            return {'type':'mesh','uri':mesh_uri(g,hull)}
        if t == mujoco.mjtGeom.mjGEOM_BOX:
            return {'type':'box','size':(2*size).tolist()}
        if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
            return {'type':'cylinder','radius':float(size[0]),'length':float(2*size[1])}
        if t == mujoco.mjtGeom.mjGEOM_SPHERE:
            return {'type':'sphere','radius':float(size[0])}
        if t == mujoco.mjtGeom.mjGEOM_PLANE:
            return {'type':'plane','coefficients':[0.,0.,1.,0.]}
        raise ValueError(f'Unsupported shape: {t}; do not silently approximate')

    def add_geom(link, g, role, added=False):
        element = ET.SubElement(link, role, name=f'geom_{g}_{role}')
        origin(element, m.geom_pos[g], m.geom_quat[g])
        geom = ET.SubElement(element, 'geometry')
        desc = shape(g, hull=role=='collision' and not added)
        if desc['type']=='mesh': ET.SubElement(geom,'mesh',filename=desc['uri'])
        elif desc['type']=='box': ET.SubElement(geom,'box',size=numbers(desc['size']))
        elif desc['type']=='cylinder': ET.SubElement(geom,'cylinder',radius=str(desc['radius']),length=str(desc['length']))
        elif desc['type']=='sphere': ET.SubElement(geom,'sphere',radius=str(desc['radius']))
        else: raise ValueError('Plane is world-only')
        if role=='visual':
            color=ET.SubElement(element,'material',name=f'color_{g}')
            ET.SubElement(color,'color',rgba=numbers(m.geom_rgba[g]))

    root = ET.Element('robot',name='i10_rh56')
    root.append(ET.Comment(' GENERATED by tools/build_moveit_model.py; planning only; no execution limits approved. '))
    ET.SubElement(root,'link',name='world')
    coupling = {c.follower_id:c for c in p.scene.hand.couplings}
    driver_limits = {c.joint_id:(c.q_open,c.q_closed) for c in p.scene.hand.channels}
    added_names = {'sensor_visual','adapter_visual'}
    geometry_records = []
    for b in robot_bodies:
        name=m.body(b).name
        link=ET.SubElement(root,'link',name=name)
        for g in range(int(m.body_geomadr[b]),int(m.body_geomadr[b]+m.body_geomnum[b])):
            enabled=bool(m.geom_contype[g] or m.geom_conaffinity[g])
            added=m.geom(g).name in added_names
            if not enabled: add_geom(link,g,'visual')
            if enabled or added: add_geom(link,g,'collision',added=added)
            geometry_records.append({'geom':g,'link':name,'name':m.geom(g).name,'source_collision':enabled,
                                     'added_collision':added,'shape':shape(g,hull=enabled)})
        parent=int(m.body_parentid[b])
        if m.body_jntnum[b] > 1: raise ValueError('Multi-joint body needs explicit frame conversion')
        if m.body_jntnum[b]:
            j=int(m.body_jntadr[b]);jname=m.joint(j).name
            if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE or np.any(m.jnt_pos[j]) or m.qpos0[m.jnt_qposadr[j]] != 0:
                raise ValueError(f'Unsupported joint frame/reference: {jname}')
            joint=ET.SubElement(root,'joint',name=jname,type='revolute')
            ET.SubElement(joint,'axis',xyz=numbers(m.jnt_axis[j]))
            bounds=driver_limits.get(j,m.jnt_range[j])
            src=arm_limits.get(jname,{'velocity':'0','effort':'0'})
            ET.SubElement(joint,'limit',lower=str(bounds[0]),upper=str(bounds[1]),velocity=src['velocity'],effort=src['effort'])
            if j in coupling:
                c=coupling[j]
                ET.SubElement(joint,'mimic',joint=m.joint(c.driver_id).name,multiplier=str(c.slope),offset=str(c.offset))
        else:
            joint=ET.SubElement(root,'joint',name=f'{name}_fixed',type='fixed')
        ET.SubElement(joint,'child',link=name)
        if parent in p.robot_bodies:
            ET.SubElement(joint,'parent',link=m.body(parent).name)
            origin(joint,m.body_pos[b],m.body_quat[b])
        else:
            ET.SubElement(joint,'parent',link='world')
            origin(joint,d.xpos[b],d.xquat[b])
    write_xml(config/'robot.urdf',root)
    # Source coverage is a diagnostic model, never the default planning model.
    source_root=ET.fromstring(ET.tostring(root))
    for link in source_root.findall('link'):
        if link.get('name') in {'sensor_link','adapter'}:
            for geom in link.findall('collision'): link.remove(geom)
    write_xml(config/'robot_source_coverage.urdf',source_root)

    srdf=ET.Element('robot',name='i10_rh56')
    arm=ET.SubElement(srdf,'group',name='arm')
    ET.SubElement(arm,'chain',base_link='base_link',tip_link='base')
    hand=ET.SubElement(srdf,'group',name='hand')
    for c in p.scene.hand.channels: ET.SubElement(hand,'joint',name=c.joint)
    ET.SubElement(srdf,'end_effector',name='rh56',parent_link='base',group='hand',parent_group='arm')
    state=ET.SubElement(srdf,'group_state',name='candidate_pregrasp',group='hand')
    for c in p.scene.hand.channels:
        ET.SubElement(state,'joint',name=c.joint,value=str(d.qpos[m.jnt_qposadr[c.joint_id]]))
    exclusions={}
    def exclude(a,b,reason):
        key=tuple(sorted((a,b)))
        if a!=b: exclusions.setdefault(key,reason)
    for sig in m.exclude_signature:
        a,b=int(sig)>>16,int(sig)&65535
        if a in p.robot_bodies and b in p.robot_bodies:
            exclude(m.body(a).name,m.body(b).name,'Source explicit body exclusion')
    # Match MuJoCo's welded-body/parent collision filter for source geometry only.
    collision_bodies={int(m.geom_bodyid[g]) for g in range(m.ngeom) if int(m.geom_bodyid[g]) in p.robot_bodies and (m.geom_contype[g] or m.geom_conaffinity[g])}
    for a in sorted(collision_bodies):
        for b in sorted(collision_bodies):
            if a>=b: continue
            wa,wb=int(m.body_weldid[a]),int(m.body_weldid[b])
            if wa==wb:
                exclude(m.body(a).name,m.body(b).name,'Source welded-body filter')
            elif wa and wb and (int(m.body_weldid[m.body_parentid[wa]])==wb or int(m.body_weldid[m.body_parentid[wb]])==wa):
                exclude(m.body(a).name,m.body(b).name,'Source parent-body filter')
    source_exclusions=dict(exclusions)
    for a,b in [('wrist3_Link','sensor_link'),('sensor_link','adapter'),('adapter','base')]:
        exclude(a,b,'Fixed mounting interface; internal fit unverified')
    for (a,b),reason in sorted(exclusions.items()): ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason=reason)
    write_xml(config/'robot.srdf',srdf)
    source_srdf=ET.fromstring(ET.tostring(srdf))
    for el in list(source_srdf.findall('disable_collisions')):
        if tuple(sorted((el.get('link1'),el.get('link2')))) not in source_exclusions: source_srdf.remove(el)
    write_xml(config/'robot_source_coverage.srdf',source_srdf)

    world=[]
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) in p.robot_bodies: continue
        floor=m.geom(g).name=='floor'
        if not floor and not (m.geom_contype[g] or m.geom_conaffinity[g]): continue
        world.append({'id':f'scene_geom_{g}','source_geom':g,'source_body':m.body(int(m.geom_bodyid[g])).name,
                      'added_collision':floor,'position':d.geom_xpos[g].tolist(),
                      'rotation_matrix':d.geom_xmat[g].reshape(3,3).tolist(),'shape':shape(g)})
    dump(config/'scene.json',{'frame_id':'world','objects':world,'allowed_world_contacts':[
        {'object':next(x['id'] for x in world if x['added_collision']),'link':'base_link','reason':'Fixed base support only'}]})
    dump(config/'kinematics.json',{'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution':0.005,'kinematics_solver_timeout':0.05}})

    report=json.loads(IK_REPORT.read_text())
    if report['source_scene_sha256'] != p.scene_hash or report['candidate_sha256'] != hashlib.sha256(CANDIDATE.read_bytes()).hexdigest():
        raise ValueError('Stale IK report')
    samples=[('initial_pregrasp',p.initial.copy(),p.candidate['pregrasp_channels'])]
    samples += [(f'ik_{i:02d}',np.array(sol['arm_qpos_rad']),p.candidate['pregrasp_channels']) for i,sol in enumerate(report['solutions'])]
    rng=np.random.default_rng(7)
    for i in range(6): samples.append((f'kinematic_sample_{i}',rng.uniform(-1,1,6),rng.integers(0,1001,6).tolist()))
    refs=[]
    for name,q,channels in samples:
        p.scene.hand.apply(d,channels);d.qpos[p.addresses]=q;mujoco.mj_forward(m,d)
        refs.append({'name':name,'joint_positions':{m.joint(j).name:float(d.qpos[m.jnt_qposadr[j]]) for j in range(m.njnt) if int(m.jnt_bodyid[j]) in p.robot_bodies and j not in coupling},
                     'links':{m.body(b).name:body_transform(d,m.body(b).name).tolist() for b in robot_bodies},
                     'mujoco_contacts':[{'pair':sorted([m.body(int(m.geom_bodyid[int(c.geom[0])])).name,m.body(int(m.geom_bodyid[int(c.geom[1])])).name]),'depth_m':float(-c.dist)} for c in d.contact if c.dist < 0 and any(int(m.geom_bodyid[int(g)]) in p.robot_bodies for g in c.geom)]})
    dump(config/'audit_reference.json',{'mode':'kinematic_model_reference_not_execution','arm_joints':list(ARM_JOINTS),
        'mimic_count':len(coupling),'fk_tolerance':1e-8,'penetration_diagnostic_m':p.settings.penetration_diagnostic_m,
        'joint_bounds':{m.joint(j).name:list(map(float,driver_limits.get(j,m.jnt_range[j]))) for j in range(m.njnt) if int(m.jnt_bodyid[j]) in p.robot_bodies},
        'cases':refs,'target_world_from_hand':p.world_from_hand.tolist()})
    source_paths={p.scene_path,CANDIDATE,IK_REPORT,Path(__file__).resolve(),ROOT/'assets/robots/aubo_i10/urdf/aubo_i10_JY.urdf'}
    source_paths.update(ROOT/'src/fuel_cap_task'/name for name in (
        'arm_ik.py', 'mujoco_scene.py', 'grasp_search.py', 'interfaces.py', 'fixed_grasp.py',
        'grasp_targets.py', 'grasp_preparation.py',
    ))
    for mesh in mesh_audit:
        if mesh['source_path']: source_paths.add((p.scene_path.parent/mesh['source_path']).resolve())
    versions=subprocess.run(['dpkg-query','-W','-f=${Package}=${Version}\n','ros-humble-moveit-core','ros-humble-moveit-kinematics','ros-humble-moveit-task-constructor-core'],text=True,capture_output=True,check=True).stdout.splitlines()
    dump(config/'provenance.json',{'generator':'tools/build_moveit_model.py','mujoco_version':mujoco.__version__,
        'python_environment':'project .venv (candidate-compatible)','numpy_version':np.__version__,
        'trimesh_version':trimesh.__version__,'system_scipy_version':subprocess.check_output(['/usr/bin/python3','-c','import scipy; print(scipy.__version__)'],text=True).strip(),'ros_packages':versions,
        'sources':{str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(source_paths)},
        'source_arm_differences':source_differences,'meshes':mesh_audit,'geometries':geometry_records,
        'collision_exclusions':[{'pair':list(pair),'reason':reason} for pair,reason in sorted(exclusions.items())],
        'limitations':['planning_only','uncalibrated_mount','hand_mimic_is_rigid_kinematic_approximation',
                       'source_arm_effort_velocity_not_execution_approval','hand_velocity_effort_zero_blocks_motion',
                       'no_acceleration_or_time_parameterization','fixed_mount_internal_fit_unverified',
                       'sensor_adapter_surface_mesh_not_added_to_mujoco','no_path_or_physics_validation']})
    generated=[f for folder in (config,meshes) for f in folder.iterdir() if f.is_file() and f.name!='generated_hashes.json']
    dump(config/'generated_hashes.json',{str(f.relative_to(output)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(generated)})
    print(f'Generated {len(robot_bodies)} robot links, {len(samples)} FK cases, {len(world)} world shapes in {output}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=PACKAGE)
    args=parser.parse_args()
    generate(args.output.resolve())
