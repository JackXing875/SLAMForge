#!/usr/bin/env python3
"""Absolute Trajectory Error (ATE) evaluation tool.

Computes the RMSE of absolute trajectory error after SE(3) alignment
(Horn's method) between estimated and ground-truth trajectories.

Usage:
    python tools/evaluate_ate.py estimated.txt groundtruth.txt [--plot]
"""

import argparse
import sys
from pathlib import Path

import numpy as np


def load_tum_trajectory(path: str) -> np.ndarray:
    """Load a TUM-format trajectory: timestamp tx ty tz qx qy qz qw."""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    # Return [x, y, z] columns
    return data[:, 1:4]


def load_kitti_trajectory(path: str) -> np.ndarray:
    """Load a KITTI-format trajectory: 3x4 pose matrices per row."""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    # Each row is 12 values (3x4 pose matrix), extract translation (cols 3, 7, 11)
    translations = data[:, [3, 7, 11]]
    return translations


def align_trajectory(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """Align estimated trajectory to ground truth using Umeyama (similarity)."""
    assert est.shape == gt.shape, f"Shape mismatch: {est.shape} vs {gt.shape}"

    # Compute centroids
    est_centroid = est.mean(axis=0)
    gt_centroid = gt.mean(axis=0)

    est_centered = est - est_centroid
    gt_centered = gt - gt_centroid

    # Cross-covariance
    H = est_centered.T @ gt_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure proper rotation (det = +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Scale
    scale = np.trace(gt_centered.T @ est_centered @ R) / np.trace(
        est_centered.T @ est_centered
    )
    scale = max(scale, 1e-10)

    t = gt_centroid - scale * R @ est_centroid

    aligned = (scale * R @ est.T).T + t
    return aligned


def compute_ate(est: np.ndarray, gt: np.ndarray) -> dict:
    """Compute ATE statistics."""
    diff = est - gt
    squared_errors = np.sum(diff**2, axis=1)
    rmse = np.sqrt(np.mean(squared_errors))

    return {
        "rmse": rmse,
        "mean": np.mean(np.sqrt(squared_errors)),
        "median": np.median(np.sqrt(squared_errors)),
        "std": np.std(np.sqrt(squared_errors)),
        "min": np.min(np.sqrt(squared_errors)),
        "max": np.max(np.sqrt(squared_errors)),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate Absolute Trajectory Error (ATE)"
    )
    parser.add_argument("estimated", help="Path to estimated trajectory file")
    parser.add_argument("groundtruth", help="Path to ground truth trajectory file")
    parser.add_argument(
        "--format",
        choices=["tum", "kitti"],
        default="tum",
        help="Trajectory file format",
    )
    parser.add_argument("--plot", action="store_true", help="Plot trajectories")
    args = parser.parse_args()

    # Load
    loader = load_tum_trajectory if args.format == "tum" else load_kitti_trajectory
    est = loader(args.estimated)
    gt = loader(args.groundtruth)

    # Align temporal dimension by truncating to min length
    min_len = min(len(est), len(gt))
    est = est[:min_len]
    gt = gt[:min_len]

    if min_len < 2:
        print("Error: need at least 2 poses to compute ATE")
        sys.exit(1)

    # Align
    est_aligned = align_trajectory(est, gt)

    # Compute
    stats = compute_ate(est_aligned, gt)

    print("═══════════════════════════════════════════")
    print("  Absolute Trajectory Error (ATE)")
    print("═══════════════════════════════════════════")
    print(f"  RMSE:   {stats['rmse']:.4f} m")
    print(f"  Mean:   {stats['mean']:.4f} m")
    print(f"  Median: {stats['median']:.4f} m")
    print(f"  Std:    {stats['std']:.4f} m")
    print(f"  Min:    {stats['min']:.4f} m")
    print(f"  Max:    {stats['max']:.4f} m")
    print("═══════════════════════════════════════════")

    if args.plot:
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(12, 5))

        ax1 = fig.add_subplot(121)
        ax1.plot(gt[:, 0], gt[:, 1], "b-", linewidth=1, label="Ground Truth")
        ax1.plot(est_aligned[:, 0], est_aligned[:, 1], "r--", linewidth=1, label="Estimated")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_title("Trajectory (top-down)")
        ax1.legend()
        ax1.axis("equal")

        ax2 = fig.add_subplot(122, projection="3d")
        ax2.plot(gt[:, 0], gt[:, 1], gt[:, 2], "b-", linewidth=1, label="GT")
        ax2.plot(
            est_aligned[:, 0],
            est_aligned[:, 1],
            est_aligned[:, 2],
            "r--",
            linewidth=1,
            label="Est",
        )
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")
        ax2.set_zlabel("Z")
        ax2.set_title("Trajectory (3D)")
        ax2.legend()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
