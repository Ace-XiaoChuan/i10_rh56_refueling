"""Check the raised opening, grounded body and preserved component relationships."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from fuel_cap_task.arm_ik import body_transform, inverse
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / 'artifacts/scene_history/20261001_height_040/assets/scenes/i10_rh56_fuel_cap_preview.xml'


class TankLayoutTests(unittest.TestCase):
    def test_opening_components_shift_rigidly_without_moving_arm(self):
        source = ET.parse(OLD).getroot()
        for mesh in source.findall('./asset/mesh'):
            mesh.set('file', str((DEFAULT_SCENE.parent / mesh.get('file')).resolve()))
        old_model = mujoco.MjModel.from_xml_string(ET.tostring(source, encoding='unicode'))
        old_data = mujoco.MjData(old_model)
        model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
        data = mujoco.MjData(model)
        mujoco.mj_forward(old_model, old_data)
        mujoco.mj_forward(model, data)
        for name in ['tank_opening_center', 'tank_recess_floor', 'tank_filler_top', 'cap_body', 'cap_grip_center', 'fuel_door']:
            before = body_transform(old_data, name)
            expected = before.copy()
            expected[:3, 3] += [0, 0, .2]
            np.testing.assert_allclose(body_transform(data, name), expected, atol=1e-12)
            np.testing.assert_allclose(
                inverse(body_transform(data, 'tank_opening_center')) @ body_transform(data, name),
                inverse(body_transform(old_data, 'tank_opening_center')) @ before, atol=1e-12)
        for name in ['base_link', 'wrist3_Link']:
            np.testing.assert_allclose(body_transform(data, name), body_transform(old_data, name), atol=1e-12)

    def test_actual_geometry_reaches_ground_and_new_top(self):
        model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        body = model.body('fuel_tank_body').id
        points = []
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != body:
                continue
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = model.geom_dataid[g]
                start, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
                vertices = model.mesh_vert[start:start+count]
            elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                vertices = np.array([[x, y, z] for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]]) * model.geom_size[g]
            else:
                self.fail('Unexpected tank geometry')
            points.extend(vertices @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g])
        bounds = np.array([np.min(points, axis=0), np.max(points, axis=0)])
        np.testing.assert_allclose(bounds, [[-.4, 1., 0.], [.4, 1.4, 1.]], atol=1e-7)
        np.testing.assert_allclose(data.body('tank_opening_center').xpos, [0, 1, .6], atol=1e-12)
        np.testing.assert_allclose(data.body('tank_body_center').xpos, [0, 1.2, .5], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
