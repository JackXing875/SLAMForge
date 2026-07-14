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
    compute_ate,
    load_trajectory,
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
        default="tum",
        help="Trajectory file format",
    )
    parser.add_argument("--plot", action="store_true", help="Plot trajectories")
    args = parser.parse_args()

    # Load
    est = load_trajectory(args.estimated, args.format)
    gt = load_trajectory(args.groundtruth, args.format)

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
