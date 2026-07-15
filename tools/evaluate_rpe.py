#!/usr/bin/env python3
"""Relative Pose Error (RPE) evaluation tool.

Computes the RMSE of relative pose error over fixed time intervals
between estimated and ground-truth trajectories.

Usage:
    python tools/evaluate_rpe.py estimated.txt groundtruth.txt [--delta 1] [--format tum]
"""

import argparse
import sys

from trajectory_io import (
    FORMAT_REGISTRY,
    align_trajectory,
    associate_trajectories,
    compute_rpe,
    load_trajectory_full,
)


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate Relative Pose Error (RPE)"
    )
    parser.add_argument("estimated", help="Path to estimated trajectory file")
    parser.add_argument("groundtruth", help="Path to ground truth trajectory file")
    parser.add_argument(
        "--delta",
        type=int,
        default=1,
        help="Frame interval for relative comparison (default: 1)",
    )
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
        print("Error: need at least 2 poses to compute RPE")
        sys.exit(1)

    try:
        est_aligned = align_trajectory(est, gt)
    except ValueError as error:
        print(f"Error: {error}")
        sys.exit(1)

    # Compute RPE
    stats = compute_rpe(est_aligned, gt, delta=args.delta)

    print("═══════════════════════════════════════════")
    print("  Relative Pose Error (RPE)")
    print(f"  Frame delta: {args.delta}")
    print(f"  Associated poses: {min_len}")
    print(f"  Formats: est={estimated_format}, gt={groundtruth_format}")
    print("═══════════════════════════════════════════")
    print(f"  RMSE:   {stats['rmse']:.4f} m")
    print(f"  Mean:   {stats['mean']:.4f} m")
    print(f"  Median: {stats['median']:.4f} m")
    print(f"  Std:    {stats['std']:.4f} m")
    print(f"  Min:    {stats['min']:.4f} m")
    print(f"  Max:    {stats['max']:.4f} m")
    print("═══════════════════════════════════════════")

    # Typical RPE interpretation
    if stats["rmse"] < 0.01:
        quality = "Excellent"
    elif stats["rmse"] < 0.05:
        quality = "Good"
    elif stats["rmse"] < 0.10:
        quality = "Fair"
    else:
        quality = "Poor"
    print(f"  Quality: {quality}")
    print()


if __name__ == "__main__":
    main()
