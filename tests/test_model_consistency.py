"""Guard the frame/parallel-axis conversions with two independent dynamics engines."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from franka_7link_dynamics_id.estimation import build_pinocchio_model, load_config
from franka_7link_dynamics_id.inertial import cad_prior_vector
from franka_7link_dynamics_id.model_export import read_urdf_rigid_parameters, write_identified_urdf
from franka_7link_dynamics_id.paths import model_config_path, package_root
from franka_7link_dynamics_id.simulation import write_mjcf


class ModelConsistencyTests(unittest.TestCase):
    def test_urdf_and_mujoco_match_pinocchio_inverse_dynamics(self):
        import mujoco
        import pinocchio as pin

        config = load_config(model_config_path())
        parameters = cad_prior_vector(config)[:70]
        nominal = package_root() / 'models/fr3_nominal.urdf'
        np.testing.assert_allclose(read_urdf_rigid_parameters(nominal), parameters, atol=1e-12)
        _, model = build_pinocchio_model(config)
        for joint in range(7):
            model.inertias[joint+1] = pin.Inertia.FromDynamicParameters(parameters[10*joint:10*(joint+1)])
        q = np.array([0.2, -0.7, 0.3, -2.2, 0.1, 1.4, 0.6])
        dq = np.array([0.3, -0.2, 0.4, -0.1, 0.5, 0.2, -0.3])
        qdd = np.array([0.4, -0.7, 0.1, 0.8, -0.2, 0.3, 0.5])
        expected = pin.rnea(model, model.createData(), q, dq, qdd)

        with tempfile.TemporaryDirectory(prefix='franka id ') as temporary:
            directory = Path(temporary)
            urdf = write_identified_urdf(nominal, directory/'identified.urdf', parameters)
            round_trip_model = pin.buildModelFromUrdf(str(urdf))
            actual = pin.rnea(round_trip_model, round_trip_model.createData(), q, dq, qdd)
            np.testing.assert_allclose(actual, expected, atol=1e-9)

            xml = write_mjcf(directory/'identified.xml', parameters, config, 'fr3_check')
            mj_model = mujoco.MjModel.from_xml_path(str(xml))
            data = mujoco.MjData(mj_model)
            data.qpos[:] = q
            data.qvel[:] = dq
            data.qacc[:] = qdd
            mujoco.mj_inverse(mj_model, data)
            expected_with_armature = expected + np.asarray(config['simulation_joint_armature']) * qdd
            np.testing.assert_allclose(data.qfrc_inverse, expected_with_armature, atol=1e-8)


if __name__ == '__main__':
    unittest.main()
