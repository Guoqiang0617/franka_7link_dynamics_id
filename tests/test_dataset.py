import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

from franka_7link_dynamics_id.dataset import prepare_split


class DatasetPreparationTests(unittest.TestCase):
    def test_csv_becomes_finite_seven_axis_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "raw"
            source.mkdir()
            time_s = np.arange(301) * 0.002
            values = {"ros_time_ns": (time_s * 1e9).astype(np.int64), "period_ns": 2_000_000}
            for joint in range(7):
                values[f"q_{joint}"] = np.sin(time_s * (joint + 1))
                values[f"dq_{joint}"] = (joint + 1) * np.cos(time_s * (joint + 1))
                values[f"tau_J_{joint}"] = np.full_like(time_s, joint + 0.5)
                values[f"gravity_{joint}"] = np.full_like(time_s, joint + 1.5)
            pd.DataFrame(values).to_csv(source / "trajectory_001.csv", index=False)

            output = root / "processed" / "train.npz"
            report = prepare_split(source, output, stride=5, window=21, polyorder=3)
            with np.load(output) as data:
                self.assertEqual(data["q"].shape, (61, 7))
                self.assertEqual(data["qdd"].shape, (61, 7))
                self.assertEqual(data["source_files"].tolist(), ["trajectory_001.csv"])
                self.assertTrue(np.isfinite(data["qdd"]).all())
                np.testing.assert_allclose(data["tau"][0], np.arange(7) + 0.5)
                np.testing.assert_allclose(data["gravity"][0], np.arange(7) + 1.5)
            self.assertEqual(report["target_torque_column"], "tau_J")

    def test_empty_recording_is_reported_and_does_not_break_source_indices(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            values = {"ros_time_ns": [1, 1_000_001, 2_000_001], "period_ns": [1_000_000]*3}
            for name in ('q', 'dq', 'tau_J', 'gravity'):
                for joint in range(7):
                    values[f'{name}_{joint}'] = [0., 0., 0.]
            frame = pd.DataFrame(values)
            frame.iloc[:0].to_csv(root/'trajectory_001.csv', index=False)
            frame.to_csv(root/'trajectory_002.csv', index=False)
            report = prepare_split(root, root/'prepared.npz')
            self.assertEqual(report['input_file_count'], 2)
            self.assertEqual(report['raw_file_count'], 1)
            self.assertEqual(report['skipped_files'][0]['file'], 'trajectory_001.csv')
            with np.load(root/'prepared.npz') as prepared:
                self.assertEqual(prepared['source_files'].tolist(), ['trajectory_002.csv'])
                self.assertEqual(prepared['source_id'].tolist(), [0])


if __name__ == "__main__":
    unittest.main()
