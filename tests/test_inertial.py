import unittest

import numpy as np

from franka_7link_dynamics_id.inertial import (
    inertial_com_to_origin,
    inertial_origin_to_com,
    parameters_from_pseudo_inertia,
    pseudo_inertia,
)


class InertialFrameTests(unittest.TestCase):
    def test_parallel_axis_round_trip(self):
        mass = 2.4
        com = np.array([0.03, -0.02, 0.07])
        inertia_com = np.diag([0.02, 0.03, 0.04])
        inertia_origin = inertial_com_to_origin(
            mass, com, [inertia_com[0, 0], 0, inertia_com[1, 1], 0, 0, inertia_com[2, 2]]
        )
        params = np.r_[mass, mass * com, inertia_origin[0, 0], inertia_origin[0, 1],
                       inertia_origin[1, 1], inertia_origin[0, 2], inertia_origin[1, 2],
                       inertia_origin[2, 2]]
        recovered_mass, recovered_com, recovered_inertia = inertial_origin_to_com(params)
        self.assertAlmostEqual(recovered_mass, mass)
        np.testing.assert_allclose(recovered_com, com)
        np.testing.assert_allclose(recovered_inertia, inertia_com)

    def test_pseudo_inertia_parameter_round_trip(self):
        params = np.array([2.0, 0.1, -0.2, 0.3, 0.8, 0.01, 0.9, -0.02, 0.03, 0.7])
        matrix = pseudo_inertia(params)
        np.testing.assert_allclose(parameters_from_pseudo_inertia(matrix), params)
        self.assertTrue(np.all(np.isfinite(np.linalg.eigvalsh(matrix))))


if __name__ == "__main__":
    unittest.main()
