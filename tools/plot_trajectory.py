#!/usr/bin/env python3
"""3D trajectory visualization tool for LiteVO output.

Usage:
    python tools/plot_trajectory.py trajectory.txt [--groundtruth gt.txt] [--save plot.png]
"""

import argparse
from pathlib import Path

import numpy as np


def load_trajectory(path: str, format: str = "tum") -> np.ndarray:
    """Load trajectory from file."""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    if format == "tum":
        # timestamp tx ty tz qx qy qz qw
        return data[:, 1:4]
    elif format == "kitti":
        # 3x4 matrix per row → extract translation cols 3, 7, 11
        return data[:, [3, 7, 11]]
    else:
        raise ValueError(f"Unknown format: {format}")


def main():
    parser = argparse.ArgumentParser(description="Plot 3D trajectory")
    parser.add_argument("trajectory", help="Path to trajectory file")
    parser.add_argument("--groundtruth", "-g", help="Optional ground truth trajectory")
    parser.add_argument("--format", choices=["tum", "kitti"], default="tum")
    parser.add_argument("--save", "-s", help="Save plot to file instead of showing")
    args = parser.parse_args()

    import matplotlib.pyplot as plt

    traj = load_trajectory(args.trajectory, args.format)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], "b-", linewidth=2, label="Estimated")
    ax.scatter(traj[0, 0], traj[0, 1], traj[0, 2], color="green", s=80, label="Start")
    ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], color="red", s=80, label="End")

    if args.groundtruth:
        gt = load_trajectory(args.groundtruth, args.format)
        ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], "k--", linewidth=1, alpha=0.7, label="GT")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("LiteVO Trajectory")
    ax.legend()
    ax.view_init(elev=30, azim=-45)

    if args.save:
        plt.savefig(args.save, dpi=300, bbox_inches="tight")
        print(f"Plot saved to: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
