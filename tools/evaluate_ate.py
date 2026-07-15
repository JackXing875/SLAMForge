#!/usr/bin/env python3
"""Absolute Trajectory Error (ATE) evaluation tool.

Computes the RMSE of absolute trajectory error after SE(3) alignment
(Umeyama's method) between estimated and ground-truth trajectories.

Usage:
    python tools/evaluate_ate.py estimated.txt groundtruth.txt [--format tum] [--plot]
"""

import argparse
import sys

from trajectory_io import (
    FORMAT_REGISTRY,
    align_trajectory,
    associate_trajectories,
    compute_ate,
    load_trajectory_full,
)


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate Absolute Trajectory Error (ATE)"
    )
    parser.add_argument("estimated", help="Path to estimated trajectory file")
    parser.add_argument("groundtruth", help="Path to ground truth trajectory file")
    parser.add_argument(
        "--format",
        choices=list(FORMAT_REGISTRY.keys()),
        default=None,
        help="Default format for both trajectory files",
    )
    parser.add_argument(
        "--estimated-format",
        choices=list(FORMAT_REGISTRY.keys()),
        help="Format of the estimated trajectory (overrides --format)",
    )
    parser.add_argument(
        "--groundtruth-format",
        choices=list(FORMAT_REGISTRY.keys()),
        help="Format of the ground-truth trajectory (overrides --format)",
    )
    parser.add_argument(
        "--max-time-difference",
        type=float,
        default=0.02,
        help="Maximum timestamp difference in seconds for TUM/EuRoC association",
    )
    parser.add_argument("--plot", action="store_true", help="Plot trajectories")
    args = parser.parse_args()

    default_format = args.format or "tum"
    estimated_format = args.estimated_format or default_format
    groundtruth_format = args.groundtruth_format or default_format

    est, est_timestamps, _ = load_trajectory_full(args.estimated, estimated_format)
    gt, gt_timestamps, _ = load_trajectory_full(args.groundtruth, groundtruth_format)
    est, gt = associate_trajectories(
        est, est_timestamps, gt, gt_timestamps, args.max_time_difference
    )
    min_len = len(est)

    if min_len < 2:
        print("Error: need at least 2 poses to compute ATE")
        sys.exit(1)

    try:
        est_aligned = align_trajectory(est, gt)
    except ValueError as error:
        print(f"Error: {error}")
        sys.exit(1)

    # Compute
    stats = compute_ate(est_aligned, gt)

    print("═══════════════════════════════════════════")
    print("  Absolute Trajectory Error (ATE)")
    print("═══════════════════════════════════════════")
    print(f"  Associated poses: {min_len}")
    print(f"  Formats: est={estimated_format}, gt={groundtruth_format}")
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
