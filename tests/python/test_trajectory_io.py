"""Regression tests for trajectory format handling and timestamp association."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

from trajectory_io import (
    align_trajectory,
    associate_trajectories,
    load_trajectory_full,
    save_trajectory,
)


class TrajectoryAssociationTest(unittest.TestCase):
    def test_timestamp_association_skips_unmatched_poses(self):
        estimated = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        groundtruth = np.array([[10.0, 0.0, 0.0], [20.0, 0.0, 0.0]])

        est, gt = associate_trajectories(
            estimated,
            np.array([0.0, 1.0, 2.0]),
            groundtruth,
            np.array([0.01, 2.01]),
            max_time_difference=0.02,
        )

        np.testing.assert_array_equal(est, estimated[[0, 2]])
        np.testing.assert_array_equal(gt, groundtruth)

    def test_index_association_for_timestamp_free_format(self):
        estimated = np.arange(12, dtype=float).reshape(4, 3)
        groundtruth = np.arange(6, dtype=float).reshape(2, 3)

        est, gt = associate_trajectories(estimated, None, groundtruth, None)

        np.testing.assert_array_equal(est, estimated[:2])
        np.testing.assert_array_equal(gt, groundtruth)

    def test_alignment_rejects_stationary_trajectory(self):
        stationary = np.zeros((3, 3))
        groundtruth = np.eye(3)

        with self.assertRaises(ValueError):
            align_trajectory(stationary, groundtruth)

    def test_alignment_handles_rotation_scale_and_translation(self):
        estimated = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [2.0, 1.0, 0.0]]
        )
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        groundtruth = (2.5 * rotation @ estimated.T).T + np.array([3.0, -1.0, 4.0])

        np.testing.assert_allclose(align_trajectory(estimated, groundtruth), groundtruth)

    def test_euroc_round_trip_uses_native_timestamp_and_quaternion_order(self):
        positions = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        timestamps = np.array([1.4e9, 1.4e9 + 0.05])
        quaternions = np.array([[0.1, 0.2, 0.3, 0.9], [0.0, 0.0, 0.0, 1.0]])

        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state_groundtruth_estimate0.csv"
            save_trajectory(str(path), positions, timestamps, quaternions, format="euroc")
            loaded_positions, loaded_timestamps, loaded_quaternions = load_trajectory_full(
                str(path), "euroc"
            )

        np.testing.assert_allclose(loaded_positions, positions)
        np.testing.assert_allclose(loaded_timestamps, timestamps)
        np.testing.assert_allclose(loaded_quaternions, quaternions)


if __name__ == "__main__":
    unittest.main()
