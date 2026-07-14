#!/usr/bin/env python3
"""Trajectory I/O utilities for LiteVO evaluation tools.

Provides format-agnostic trajectory loading and conversion for:
  - TUM:   timestamp tx ty tz qx qy qz qw
  - KITTI: 3x4 pose matrix per row (12 columns)
  - EuRoC: timestamp tx ty tz qx qy qz qw (same as TUM)

The core data structure is a Nx3 numpy array of [x, y, z] translations.
Timestamps and orientations are preserved as optional secondary arrays.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

import numpy as np


# ── Format registry ────────────────────────────────────────────────────────────

def _load_tum(path: str) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Load TUM-format: timestamp tx ty tz qx qy qz qw.

    Returns (positions_nx3, timestamps_n, quaternions_nx4).
    """
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quats = data[:, 4:8]
    return positions, timestamps, quats


def _load_kitti(path: str) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Load KITTI-format: 3x4 pose matrix per row (12 columns).

    Returns (positions_nx3, None, None).
    The 3x4 matrix [R|t] has translation at columns 3, 7, 11.
    """
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    # Columns: R11 R12 R13 tx R21 R22 R23 ty R31 R32 R33 tz  → translation at cols 3, 7, 11
    positions = data[:, [3, 7, 11]]
    return positions, None, None


def _load_euroc(path: str) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Load EuRoC-format: same as TUM (timestamp tx ty tz qx qy qz qw)."""
    return _load_tum(path)


# Format name → loader function
FORMAT_REGISTRY: dict[str, Callable] = {
    "tum": _load_tum,
    "kitti": _load_kitti,
    "euroc": _load_euroc,
}


# ── Public API ─────────────────────────────────────────────────────────────────

def load_trajectory(
    path: str,
    format: str = "tum",
) -> np.ndarray:
    """Load trajectory positions from a file.

    Args:
        path: Path to trajectory file.
        format: One of "tum", "kitti", "euroc".

    Returns:
        Nx3 numpy array of [x, y, z] positions.
    """
    if format not in FORMAT_REGISTRY:
        raise ValueError(f"Unknown format '{format}'. Supported: {list(FORMAT_REGISTRY)}")

    positions, _, _ = FORMAT_REGISTRY[format](path)
    return positions


def load_trajectory_full(
    path: str,
    format: str = "tum",
) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Load trajectory with timestamps and orientations.

    Args:
        path: Path to trajectory file.
        format: One of "tum", "kitti", "euroc".

    Returns:
        (positions_nx3, timestamps_n | None, quaternions_nx4 | None).
        KITTI format returns None for timestamps and quaternions.
    """
    if format not in FORMAT_REGISTRY:
        raise ValueError(f"Unknown format '{format}'. Supported: {list(FORMAT_REGISTRY)}")

    return FORMAT_REGISTRY[format](path)


def save_trajectory(
    path: str,
    positions: np.ndarray,
    timestamps: Optional[np.ndarray] = None,
    quaternions: Optional[np.ndarray] = None,
    format: str = "tum",
) -> None:
    """Save a trajectory to a file.

    Args:
        path: Output file path.
        positions: Nx3 array of [x, y, z].
        timestamps: Optional N-element array of timestamps.
        quaternions: Optional Nx4 array of [qx, qy, qz, qw].
        format: Output format ("tum" or "kitti").
    """
    n = len(positions)
    if format in ("tum", "euroc"):
        if timestamps is None:
            timestamps = np.arange(n, dtype=np.float64)
        if quaternions is None:
            quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
        out = np.column_stack([timestamps, positions, quaternions])
        header = "# timestamp tx ty tz qx qy qz qw"
    elif format == "kitti":
        # Build 3x4 identity rotations with given translations
        out = np.zeros((n, 12))
        out[:, 0] = 1.0  # R11
        out[:, 4] = 1.0  # R22
        out[:, 8] = 1.0  # R33
        out[:, 3] = positions[:, 0]  # tx
        out[:, 7] = positions[:, 1]  # ty
        out[:, 11] = positions[:, 2]  # tz
        header = "# R11 R12 R13 tx R21 R22 R23 ty R31 R32 R33 tz"
    else:
        raise ValueError(f"Unknown format '{format}'")

    np.savetxt(path, out, header=header, comments="")


def align_trajectory(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """Align estimated trajectory to ground truth using Umeyama's method.

    Finds the similarity transform (scale * rotation + translation) that
    minimizes the least-squares error between the estimated and ground truth
    point clouds.

    Args:
        est: Nx3 estimated positions.
        gt:  Nx3 ground truth positions.

    Returns:
        Nx3 aligned estimated positions.
    """
    assert est.shape == gt.shape, f"Shape mismatch: {est.shape} vs {gt.shape}"

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
    """Compute Absolute Trajectory Error statistics.

    Args:
        est: Nx3 estimated positions (already aligned to GT).
        gt:  Nx3 ground truth positions.

    Returns:
        Dict with keys: rmse, mean, median, std, min, max (all in meters).
    """
    diff = est - gt
    errors = np.sqrt(np.sum(diff**2, axis=1))

    return {
        "rmse": float(np.sqrt(np.mean(errors**2))),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "std": float(np.std(errors)),
        "min": float(np.min(errors)),
        "max": float(np.max(errors)),
    }


def compute_rpe(
    est: np.ndarray,
    gt: np.ndarray,
    delta: int = 1,
) -> dict:
    """Compute Relative Pose Error statistics.

    RPE measures the local drift over a fixed time interval (delta frames).
    For translation-only RPE (the default), we compare the relative motion
    between corresponding frame pairs in the estimated and ground truth
    trajectories.

    Args:
        est:   Nx3 estimated positions.
        gt:    Nx3 ground truth positions (aligned to est).
        delta: Frame interval for relative comparisons (default=1).

    Returns:
        Dict with keys: rmse, mean, median, std, min, max (all in meters).
    """
    n = len(est)
    if n <= delta:
        return {"rmse": 0.0, "mean": 0.0, "median": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}

    errors = []
    for i in range(n - delta):
        # Relative motion in estimated
        est_rel = est[i + delta] - est[i]
        # Relative motion in ground truth
        gt_rel = gt[i + delta] - gt[i]
        # Translational error
        error = np.linalg.norm(est_rel - gt_rel)
        errors.append(error)

    errors = np.array(errors)

    return {
        "rmse": float(np.sqrt(np.mean(errors**2))),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "std": float(np.std(errors)),
        "min": float(np.min(errors)),
        "max": float(np.max(errors)),
    }
