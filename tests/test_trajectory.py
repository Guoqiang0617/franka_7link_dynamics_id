import unittest

import numpy as np

from franka_7link_dynamics_id.trajectory import FFSTrajectory


class TrajectoryTests(unittest.TestCase):
    def test_ffs_endpoints_and_shapes(self):
        initial = np.arange(7, dtype=float)
        ak = np.zeros((7, 5))
        bk = np.zeros((7, 5))
        trajectory = FFSTrajectory(
            {
                "initial_q": initial,
                "ak": ak.tolist(),
                "bk": bk.tolist(),
                "omega_f": 0.6283185307,
                "duration": 10.0,
                "num_harmonics": 5,
            }
        )
        q, qd, qdd = trajectory.evaluate([0.0, 1.0, 10.0])
        self.assertEqual(q.shape, (3, 7))
        self.assertEqual(qd.shape, (3, 7))
        self.assertEqual(qdd.shape, (3, 7))
        np.testing.assert_allclose(q, np.tile(initial, (3, 1)))
        np.testing.assert_allclose(qd, 0.0)
        np.testing.assert_allclose(qdd, 0.0)


if __name__ == "__main__":
    unittest.main()
