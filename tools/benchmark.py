#!/usr/bin/env python3
"""Batch benchmarking tool for SLAMForge.

Runs SLAMForge on multiple dataset sequences, computes ATE and RPE for each,
and produces a summary table and CSV report.

Usage:
    python tools/benchmark.py --dataset-dir /data/kitti --config config/kitti.yaml \\
        --sequences 00 01 02 03 04 05 06 07 08 09 10
    python tools/benchmark.py --dataset-dir /data/tum --config config/tum.yaml \\
        --format tum --pattern "rgbd_dataset_freiburg*"
"""

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

from trajectory_io import (
    FORMAT_REGISTRY,
    align_trajectory,
    associate_trajectories,
    compute_ate,
    compute_rpe,
    load_trajectory_full,
)


def find_sequences(dataset_dir: Path, pattern: str = "*") -> list[Path]:
    """Find dataset sequences matching a glob pattern."""
    matches = sorted(dataset_dir.glob(pattern))
    return [m for m in matches if m.is_dir()]


def run_slam(
    slamforge_bin: str,
    config_path: str,
    input_dir: str,
    output_path: str,
    extra_args: Optional[list[str]] = None,
) -> tuple[bool, str]:
    """Run slamforge_cli on a sequence. Returns (success, stdout)."""
    cmd = [
        slamforge_bin, "run",
        "--config", config_path,
        "--input", input_dir,
        "--output", output_path,
    ]
    if extra_args:
        cmd.extend(extra_args)

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=3600,  # 1 hour max per sequence
        )
        return result.returncode == 0, result.stdout
    except subprocess.TimeoutExpired:
        return False, "TIMEOUT"
    except FileNotFoundError:
        return False, f"Binary not found: {slamforge_bin}"


def evaluate_sequence(
    est_path: str,
    gt_path: str,
    estimated_format: str = "tum",
    groundtruth_format: str = "tum",
    max_time_difference: float = 0.02,
) -> dict:
    """Evaluate a single sequence: load, align, compute ATE + RPE."""
    try:
        est, est_timestamps, _ = load_trajectory_full(est_path, estimated_format)
        gt, gt_timestamps, _ = load_trajectory_full(gt_path, groundtruth_format)
        est, gt = associate_trajectories(
            est, est_timestamps, gt, gt_timestamps, max_time_difference
        )
    except Exception as e:
        return {"error": str(e)}

    min_len = len(est)

    if min_len < 2:
        return {"error": "Too few poses"}

    try:
        est_aligned = align_trajectory(est, gt)
        ate = compute_ate(est_aligned, gt)
        rpe = compute_rpe(est_aligned, gt, delta=1)
        rpe_10 = compute_rpe(est_aligned, gt, delta=10)
    except ValueError as error:
        return {"error": str(error)}

    return {
        "frames": min_len,
        "ate_rmse": ate["rmse"],
        "ate_mean": ate["mean"],
        "ate_median": ate["median"],
        "rpe_rmse": rpe["rmse"],
        "rpe_rmse_10f": rpe_10["rmse"],
    }


def main():
    parser = argparse.ArgumentParser(
        description="Batch benchmark SLAMForge across multiple dataset sequences"
    )
    parser.add_argument(
        "--dataset-dir", required=True, help="Root directory of the dataset"
    )
    parser.add_argument(
        "--config", required=True, help="Path to YAML configuration file"
    )
    parser.add_argument(
        "--sequences",
        nargs="+",
        help="Specific sequence names/subdirs to run (space-separated)",
    )
    parser.add_argument(
        "--pattern", default="*", help="Glob pattern to discover sequences"
    )
    parser.add_argument(
        "--format",
        choices=list(FORMAT_REGISTRY.keys()),
        default="tum",
        help="Default trajectory format for both estimate and ground truth",
    )
    parser.add_argument(
        "--estimated-format",
        choices=list(FORMAT_REGISTRY.keys()),
        help="Estimated trajectory format (overrides --format)",
    )
    parser.add_argument(
        "--groundtruth-format",
        choices=list(FORMAT_REGISTRY.keys()),
        help="Ground-truth trajectory format (overrides --format)",
    )
    parser.add_argument(
        "--max-time-difference",
        type=float,
        default=0.02,
        help="Maximum timestamp difference in seconds for TUM/EuRoC association",
    )
    parser.add_argument(
        "--slamforge-bin",
        default="build/apps/slamforge_cli",
        help="Path to slamforge_cli binary",
    )
    parser.add_argument(
        "--output-dir",
        default="benchmark_results",
        help="Directory for output trajectories and results",
    )
    parser.add_argument(
        "--gt-subdir",
        default="",
        help="Subdirectory within each sequence containing groundtruth.txt",
    )
    parser.add_argument(
        "--gt-filename",
        default="groundtruth.txt",
        help="Ground truth filename",
    )
    parser.add_argument(
        "--image-subdir",
        default="image_0",
        help="Subdirectory containing input images",
    )
    parser.add_argument(
        "--csv", default="benchmark_results.csv", help="Output CSV summary file"
    )
    args = parser.parse_args()
    estimated_format = args.estimated_format or args.format
    groundtruth_format = args.groundtruth_format or args.format

    dataset_dir = Path(args.dataset_dir)
    if not dataset_dir.is_dir():
        print(f"Error: dataset directory not found: {dataset_dir}")
        sys.exit(1)

    # Discover sequences
    if args.sequences:
        sequences = [dataset_dir / s for s in args.sequences]
    else:
        sequences = find_sequences(dataset_dir, args.pattern)

    if not sequences:
        print(f"No sequences found in {dataset_dir}")
        sys.exit(1)

    print(f"Found {len(sequences)} sequences to benchmark")
    print(f"SLAMForge binary: {args.slamforge_bin}")
    print(f"Config: {args.config}")
    print()

    # Prepare output
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    results = []
    total_start = time.time()

    for seq_path in sequences:
        seq_name = seq_path.name
        print(f"[{seq_name}] Running...", end=" ", flush=True)

        # Input images
        image_dir = seq_path / args.image_subdir
        if not image_dir.is_dir():
            print(f"SKIP (no image dir: {image_dir})")
            continue

        # Output trajectory
        traj_out = output_dir / f"{seq_name}.txt"

        # Run
        ok, stdout = run_slam(
            args.slamforge_bin,
            args.config,
            str(image_dir),
            str(traj_out),
            ["--output-format", estimated_format],
        )

        if not ok:
            print(f"FAILED")
            results.append({"sequence": seq_name, "status": "FAILED", "error": stdout.strip().split("\n")[-1] if stdout else "Unknown"})
            continue

        # Evaluate against ground truth
        gt_path = (seq_path / args.gt_subdir / args.gt_filename) if args.gt_subdir else (seq_path / args.gt_filename)
        if gt_path.exists():
            eval_result = evaluate_sequence(
                str(traj_out),
                str(gt_path),
                estimated_format,
                groundtruth_format,
                args.max_time_difference,
            )
            if "error" in eval_result:
                print(f"DONE (eval error: {eval_result['error']})")
                results.append({"sequence": seq_name, "status": "EVAL_ERROR", **eval_result})
            else:
                print(f"DONE (ATE={eval_result['ate_rmse']:.3f}m, RPE={eval_result['rpe_rmse']:.3f}m)")
                results.append({"sequence": seq_name, "status": "OK", **eval_result})
        else:
            print(f"DONE (no GT at {gt_path})")
            results.append({"sequence": seq_name, "status": "NO_GT"})

    total_elapsed = time.time() - total_start

    # ── Summary table ──────────────────────────────────────────────────────
    print("\n" + "═" * 80)
    print("  SLAMForge Benchmark Summary")
    print("═" * 80)
    header = f"  {'Sequence':<20s} {'Status':<10s} {'Frames':>8s} {'ATE':>8s} {'RPE':>8s} {'RPE10':>8s}"
    print(header)
    print("  " + "-" * 70)

    ok_results = [r for r in results if r.get("status") == "OK"]
    for r in results:
        status = r.get("status", "???")
        if status == "OK":
            print(f"  {r['sequence']:<20s} {status:<10s} {r['frames']:>8d} "
                  f"{r['ate_rmse']:>7.3f}m {r['rpe_rmse']:>7.3f}m {r['rpe_rmse_10f']:>7.3f}m")
        else:
            print(f"  {r['sequence']:<20s} {status:<10s} {'—':>8s} {'—':>8s} {'—':>8s} {'—':>8s}")

    print("═" * 80)

    if ok_results:
        avg_ate = sum(r["ate_rmse"] for r in ok_results) / len(ok_results)
        avg_rpe = sum(r["rpe_rmse"] for r in ok_results) / len(ok_results)
        print(f"  Average ATE:  {avg_ate:.3f} m  (n={len(ok_results)})")
        print(f"  Average RPE:  {avg_rpe:.3f} m")
        print(f"  Total time:   {total_elapsed:.0f} s")
    print("═" * 80)

    # ── Save CSV ───────────────────────────────────────────────────────────
    csv_path = output_dir / args.csv
    if results:
        with open(csv_path, "w", newline="") as f:
            fieldnames = ["sequence", "status", "frames", "ate_rmse", "ate_mean",
                          "ate_median", "rpe_rmse", "rpe_rmse_10f", "error"]
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
        print(f"\nResults saved to: {csv_path}")

    # ── Save JSON ──────────────────────────────────────────────────────────
    json_path = output_dir / "benchmark_results.json"
    with open(json_path, "w") as f:
        json.dump({
            "config": args.config,
            "dataset_dir": str(dataset_dir),
            "estimated_format": estimated_format,
            "groundtruth_format": groundtruth_format,
            "elapsed_s": total_elapsed,
            "results": results,
        }, f, indent=2)
    print(f"JSON report: {json_path}")


if __name__ == "__main__":
    main()
