import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from franka_7link_dynamics_id.trajectory import FFSTrajectory
from franka_7link_dynamics_id.trajectory_generation import generate_ffs_dataset, load_generation_config, _valid_trajectory


class TrajectoryGenerationTests(unittest.TestCase):
    def test_generated_files_have_zero_endpoint_derivatives_and_obey_limits(self):
        with tempfile.TemporaryDirectory() as temporary:
            summary = generate_ffs_dataset(Path(temporary), 1, 1, seed=42)
            info = json.loads(summary.read_text())
            settings, robot = load_generation_config()
            self.assertEqual(info['num_train_trajectories'], 1)
            self.assertEqual(info['num_test_trajectories'], 1)
            for split in ('train', 'test'):
                files = list((summary.parent/split).glob('trajectory_*.json'))
                self.assertEqual(len(files), 1)
                trajectory = FFSTrajectory(files[0])
                q, dq, qdd = trajectory.evaluate([0, trajectory.duration])
                np.testing.assert_allclose(q, np.tile(trajectory.initial_q, (2, 1)), atol=1e-12)
                np.testing.assert_allclose(dq, 0, atol=1e-12)
                np.testing.assert_allclose(qdd, 0, atol=1e-12)
                self.assertTrue(_valid_trajectory(trajectory.ak, trajectory.bk, settings, robot))


if __name__ == '__main__':
    unittest.main()
