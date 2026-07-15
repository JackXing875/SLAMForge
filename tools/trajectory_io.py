#!/usr/bin/env python3
"""Trajectory I/O utilities for LiteVO evaluation tools.

Provides format-agnostic trajectory loading and conversion for:
  - TUM:   timestamp[s] tx ty tz qx qy qz qw
  - KITTI: 3x4 pose matrix per row (12 columns)
  - EuRoC: timestamp[ns], p_RS_R, q_RS_w q_RS_x q_RS_y q_RS_z

The core data structure is a Nx3 numpy array of [x, y, z] translations.
Timestamps and orientations are preserved as optional secondary arrays.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

import numpy as np


# ── Format registry ────────────────────────────────────────────────────────────

def _as_2d(data: np.ndarray, path: str, min_columns: int) -> np.ndarray:
    """Normalize a loaded numeric table and validate its column count."""
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.ndim != 2 or data.shape[1] < min_columns:
        raise ValueError(
            f"Trajectory '{path}' must contain at least {min_columns} numeric columns"
        )
    return data


def _normalise_timestamp_seconds(timestamps: np.ndarray) -> np.ndarray:
    """Convert native nanosecond timestamps to seconds while keeping TUM timestamps."""
    timestamps = np.asarray(timestamps, dtype=np.float64)
    # Unix epoch timestamps in seconds are roughly 1e9, whereas EuRoC records
    # nanoseconds at roughly 1e18.  This keeps association tolerances in seconds.
    if timestamps.size and np.nanmedian(np.abs(timestamps)) > 1e12:
        return timestamps * 1e-9
    return timestamps

def _load_tum(path: str) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Load TUM-format: timestamp tx ty tz qx qy qz qw.

    Returns (positions_nx3, timestamps_n, quaternions_nx4).
    """
    data = _as_2d(np.loadtxt(path), path, 8)
    timestamps = _normalise_timestamp_seconds(data[:, 0])
    positions = data[:, 1:4]
    quats = data[:, 4:8]
    return positions, timestamps, quats


def _load_kitti(path: str) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Load KITTI-format: 3x4 pose matrix per row (12 columns).

    Returns (positions_nx3, None, None).
    The 3x4 matrix [R|t] has translation at columns 3, 7, 11.
    """
    data = _as_2d(np.loadtxt(path), path, 12)
    # Columns: R11 R12 R13 tx R21 R22 R23 ty R31 R32 R33 tz  → translation at cols 3, 7, 11
    positions = data[:, [3, 7, 11]]
    return positions, None, None


def _load_euroc(path: str) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Load native EuRoC pose CSV data.

    EuRoC stores timestamps in nanoseconds and uses scalar-first quaternions:
    ``timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,...``.  The public return value
    exposes timestamps in seconds and quaternions in ``qx,qy,qz,qw`` order.
    Whitespace-delimited early LiteVO output is accepted as a compatibility
    fallback.
    """
    try:
        data = np.loadtxt(path, delimiter=",")
    except ValueError:
        data = np.loadtxt(path)
    data = _as_2d(data, path, 8)

    timestamps = _normalise_timestamp_seconds(data[:, 0])
    positions = data[:, 1:4]
    # Early LiteVO output advertised EuRoC but serialized a TUM-style
    # qx,qy,qz,qw line. Preserve that explicit legacy header on read.
    header = ""
    with Path(path).open(encoding="utf-8") as trajectory_file:
        for line in trajectory_file:
            if line.lstrip().startswith("#"):
                header = line.lower()
                break
            if line.strip():
                break
    quats = data[:, 4:8] if "qx qy qz qw" in header else data[:, [5, 6, 7, 4]]
    return positions, timestamps, quats


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
        (positions_nx3, timestamps_n | None, quaternions_nx4 | None), with
        timestamps expressed in seconds and quaternions ordered qx,qy,qz,qw.
        KITTI format returns None for timestamps and quaternions.
    """
    if format not in FORMAT_REGISTRY:
        raise ValueError(f"Unknown format '{format}'. Supported: {list(FORMAT_REGISTRY)}")

    return FORMAT_REGISTRY[format](path)


def associate_trajectories(
    est_positions: np.ndarray,
    est_timestamps: Optional[np.ndarray],
    gt_positions: np.ndarray,
    gt_timestamps: Optional[np.ndarray],
    max_time_difference: float = 0.02,
) -> tuple[np.ndarray, np.ndarray]:
    """Return temporally corresponding estimated and ground-truth poses.

    Timestamped formats (TUM/EuRoC) are paired one-to-one in chronological
    order.  KITTI has no timestamps, so its trajectories are paired by index.
    This avoids silently comparing a pose after tracking loss with an unrelated
    ground-truth pose, which was the previous behavior.
    """
    if max_time_difference < 0.0:
        raise ValueError("max_time_difference must be non-negative")

    if est_timestamps is None or gt_timestamps is None:
        count = min(len(est_positions), len(gt_positions))
        return est_positions[:count], gt_positions[:count]

    est_order = np.argsort(est_timestamps)
    gt_order = np.argsort(gt_timestamps)
    est_timestamps = est_timestamps[est_order]
    gt_timestamps = gt_timestamps[gt_order]
    est_positions = est_positions[est_order]
    gt_positions = gt_positions[gt_order]

    est_indices: list[int] = []
    gt_indices: list[int] = []
    est_idx = 0
    gt_idx = 0
    while est_idx < len(est_timestamps) and gt_idx < len(gt_timestamps):
        delta = est_timestamps[est_idx] - gt_timestamps[gt_idx]
        if abs(delta) <= max_time_difference:
            est_indices.append(est_idx)
            gt_indices.append(gt_idx)
            est_idx += 1
            gt_idx += 1
        elif delta < 0.0:
            est_idx += 1
        else:
            gt_idx += 1

    return est_positions[est_indices], gt_positions[gt_indices]


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
        format: Output format ("tum", "kitti", or "euroc").
    """
    n = len(positions)
    if format == "tum":
        if timestamps is None:
            timestamps = np.arange(n, dtype=np.float64)
        if quaternions is None:
            quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
        out = np.column_stack([timestamps, positions, quaternions])
        header = "# timestamp tx ty tz qx qy qz qw"
    elif format == "euroc":
        if timestamps is None:
            timestamps = np.arange(n, dtype=np.float64)
        if quaternions is None:
            quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
        timestamp_ns = np.rint(np.asarray(timestamps) * 1e9).astype(np.int64)
        # EuRoC serializes scalar-first quaternions.
        out = np.column_stack(
            [timestamp_ns, positions, quaternions[:, 3], quaternions[:, :3]]
        )
        header = (
            "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],"
            "q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []"
        )
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

    if format == "euroc":
        np.savetxt(path, out, delimiter=",", header=header, comments="", fmt="%.9f")
    else:
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
    est_variance = np.trace(est_centered.T @ est_centered)
    if est_variance <= 1e-12:
        raise ValueError("Cannot align a trajectory with zero spatial variance")
    # The transformed row-vector trajectory is ``est @ R.T``.  The scale
    # numerator is therefore tr(R @ (est.T @ gt)), not tr(gt.T @ est @ R).
    # The latter happens to work for identity rotations but is wrong generally.
    scale = np.trace(R @ H) / est_variance
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
