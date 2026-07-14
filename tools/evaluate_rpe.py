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
    compute_rpe,
    load_trajectory,
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
        default="tum",
        help="Trajectory file format",
    )
    args = parser.parse_args()

    # Load
    est = load_trajectory(args.estimated, args.format)
    gt = load_trajectory(args.groundtruth, args.format)

    # Truncate to matching length
    min_len = min(len(est), len(gt))
    est = est[:min_len]
    gt = gt[:min_len]

    if min_len < 2:
        print("Error: need at least 2 poses to compute RPE")
        sys.exit(1)

    # Align
    est_aligned = align_trajectory(est, gt)

    # Compute RPE
    stats = compute_rpe(est_aligned, gt, delta=args.delta)

    print("═══════════════════════════════════════════")
    print("  Relative Pose Error (RPE)")
    print(f"  Frame delta: {args.delta}")
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
