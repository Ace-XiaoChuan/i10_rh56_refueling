"""Check the current opening height and grounded tank geometry."""
import unittest

import mujoco
import numpy as np

from fuel_cap_task.mujoco_scene import DEFAULT_SCENE

class TankLayoutTests(unittest.TestCase):
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
