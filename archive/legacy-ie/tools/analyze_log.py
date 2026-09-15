#!/usr/bin/env python3
"""
analyze_log.py — Parse UWB-IMU FGO debug logs and generate visualizations.

Usage:
    python3 analyze_log.py [log_dir]

If log_dir is not specified, uses logs/latest (via symlink).
Generates plots (.png) and a summary report (.html) in the same directory.

Requirements:
    pip install numpy matplotlib
"""

import sys
import os
import json
import csv
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ===========================================================================
# Helpers
# ===========================================================================

def resolve_log_dir(argv):
    """Resolve target log directory from CLI or symlink."""
    if len(argv) > 1:
        d = Path(argv[1])
    else:
        # Default: logs/latest (symlink to most recent run)
        script_dir = Path(__file__).resolve().parent
        d = script_dir.parent / "logs" / "latest"
    if not d.exists():
        print(f"ERROR: log directory not found: {d}", file=sys.stderr)
        sys.exit(1)
    if d.is_symlink():
        d = d.resolve()
    return d


def load_csv(path, columns):
    """Load a CSV file into a dict of numpy arrays. Returns None on failure."""
    if not path.exists():
        return None
    data = {c: [] for c in columns}
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            for c in columns:
                try:
                    data[c].append(float(row[c]))
                except (ValueError, KeyError):
                    data[c].append(np.nan)
    for c in columns:
        data[c] = np.array(data[c])
    return data


def load_optimization_csv(log_dir):
    """Load optimization.csv → list of dict rows."""
    path = log_dir / "optimization.csv"
    if not path.exists():
        return None
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def load_summary(log_dir):
    """Load summary.json → dict."""
    path = log_dir / "summary.json"
    if not path.exists():
        return None
    with open(path) as f:
        return json.load(f)


def load_gt_comparison(log_dir):
    """Load gt_comparison.csv → dict."""
    path = log_dir / "gt_comparison.csv"
    if not path.exists():
        return None
    data = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                data[row["metric"]] = float(row["value"])
            except (ValueError, KeyError):
                pass
    return data


def load_tum_trajectory(path):
    """Load a TUM-format trajectory file.
    
    Format: timestamp tx ty tz qx qy qz qw
    Returns: dict with keys 't', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
             each as numpy array, or None on failure.
    """
    if not path.exists():
        return None
    cols = ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    data = {c: [] for c in cols}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                for i, c in enumerate(cols):
                    data[c].append(float(parts[i]))
            except (ValueError, IndexError):
                continue
    if len(data['t']) == 0:
        return None
    for c in cols:
        data[c] = np.array(data[c])
    return data


def se3_transform(pts, R, t):
    """Apply SE(3) transformation: pts @ R.T + t (row-wise)."""
    return pts @ R.T + t


def umeyama_alignment(est_pts, gt_pts):
    """Align estimated points to ground-truth using Umeyama (SE3).

    Minimises ||s * R * X + t - Y||^2  where X=est_pts, Y=gt_pts.
    Follows: S. Umeyama, "Least-squares estimation of transformation
    parameters between two point patterns", IEEE TPAMI, 1991.

    Args:
        est_pts: Nx3 numpy array of estimated positions
        gt_pts:  Nx3 numpy array of ground-truth positions

    Returns:
        R:        3x3 rotation matrix
        t:        3x1 translation vector
        s:        scale factor
        aligned:  Nx3 aligned estimated positions
    """
    n = est_pts.shape[0]
    mu_est = est_pts.mean(axis=0)
    mu_gt  = gt_pts.mean(axis=0)

    X = est_pts - mu_est
    Y = gt_pts  - mu_gt

    var_x = np.sum(X ** 2) / n

    # Cross-covariance: cov = X^T @ Y / n
    cov = (X.T @ Y) / n   # 3x3

    U, D, Vt = np.linalg.svd(cov)  # cov = U @ diag(D) @ Vt

    # Reflection correction
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    # Rotation
    R = U @ S @ Vt   # 3x3

    # Scale
    if var_x < 1e-15:
        s = 1.0
    else:
        s = np.trace(np.diag(D) @ S) / var_x

    # Translation
    t = mu_gt - s * (R @ mu_est)

    # Apply transformation to all points
    aligned = s * (est_pts @ R.T) + t
    return R, t, s, aligned


def match_trajectories(est, gt, max_dt=0.5):
    """Match estimated and ground-truth poses by nearest timestamp.

    Returns:
        est_raw:     Nx3 array of raw estimated positions (no alignment)
        gt_matched:  Nx3 array of GT positions
        est_t:       N array of timestamps
    """
    matched_est = []
    matched_gt = []
    matched_t = []
    gi = 0
    for ei in range(len(est['t'])):
        et = est['t'][ei]
        best_dt = float('inf')
        best_idx = -1
        while gi < len(gt['t']):
            dt = abs(gt['t'][gi] - et)
            if dt < best_dt:
                best_dt = dt
                best_idx = gi
            if gt['t'][gi] > et + max_dt:
                break
            gi += 1
        if best_idx >= 0 and best_dt < max_dt:
            matched_est.append([est['x'][ei], est['y'][ei], est['z'][ei]])
            matched_gt.append([gt['x'][best_idx], gt['y'][best_idx], gt['z'][best_idx]])
            matched_t.append(et)
    if len(matched_est) < 3:
        return None, None, None
    return np.array(matched_est), np.array(matched_gt), np.array(matched_t)


def compute_ape(est_aligned, gt_pts):
    """Absolute Position Error: ||aligned_i - gt_i||."""
    return np.linalg.norm(est_aligned - gt_pts, axis=1)


def compute_rpe(est, gt, delta_frames=1):
    """Relative Pose Error on RAW (unaligned) trajectories.
    
    Matches evo's default behaviour: RPE w.r.t. translation part,
    delta=N frames using consecutive pairs, NOT aligned.

    rpe[i] = || (est[i+δ] - est[i]) - (gt[i+δ] - gt[i]) ||
    """
    if len(est) <= delta_frames:
        return np.array([]), np.array([])
    
    # Relative motion in estimate and GT
    est_rel = est[delta_frames:] - est[:-delta_frames]
    gt_rel  = gt[delta_frames:]  - gt[:-delta_frames]
    
    # Difference in relative motion
    rpe = np.linalg.norm(est_rel - gt_rel, axis=1)
    rpe_t = np.arange(delta_frames, len(est))  # frame indices
    return rpe, rpe_t


def call_evo(log_dir, est_path, gt_path):
    """Run evo_ape and evo_rpe via Python API for definitive metrics + per-frame data.
    
    Returns dict with 'ate', 'rpe', 'ate_per_frame', 'est_aligned', or None on failure.
    """
    try:
        from evo.core import metrics, sync
        from evo.core.trajectory import PoseTrajectory3D
        from evo.tools import file_interface
    except ImportError:
        return None

    try:
        traj_est = file_interface.read_tum_trajectory_file(str(est_path))
        traj_gt  = file_interface.read_tum_trajectory_file(str(gt_path))
    except Exception as e:
        print(f"  [evo] Failed to load trajectories: {e}")
        return None

    if traj_est.num_poses < 3 or traj_gt.num_poses < 3:
        return None

    # --- Associate trajectories by timestamp ---
    traj_est_sync, traj_gt_sync = sync.associate_trajectories(traj_est, traj_gt)
    if traj_est_sync.num_poses < 3:
        return None

    # --- ATE with SE(3) Umeyama alignment ---
    try:
        # align() transforms traj_est_sync IN-PLACE (positions + orientations).
        # Returns (R, t, s) for reference; traj_est_sync is already aligned.
        _R, _t, _s = traj_est_sync.align(
            traj_gt_sync, correct_scale=False, correct_only_scale=False)

        # Compute APE directly on the already-aligned trajectory
        ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
        ape_metric.process_data((traj_est_sync, traj_gt_sync))

        all_ape_stats = ape_metric.get_all_statistics()
        ate_stats = {
            'mean':   float(all_ape_stats.get('mean', -1)),
            'rmse':   float(all_ape_stats.get('rmse', -1)),
            'max':    float(all_ape_stats.get('max', -1)),
            'min':    float(all_ape_stats.get('min', -1)),
            'median': float(all_ape_stats.get('median', -1)),
            'std':    float(all_ape_stats.get('std', -1)),
            'sse':    float(all_ape_stats.get('sse', -1)),
        }

        # Per-frame ATE errors and aligned positions
        ate_per_frame = ape_metric.error.copy()
        est_aligned_xyz = traj_est_sync.positions_xyz.copy()   # already aligned
        gt_sync_pos     = traj_gt_sync.positions_xyz.copy()
        est_aligned_ts  = traj_est_sync.timestamps.copy()

        print(f"  [evo] ATE: mean={ate_stats['mean']:.4f}, "
              f"rmse={ate_stats['rmse']:.4f}, median={ate_stats['median']:.4f}")

    except Exception as e:
        print(f"  [evo] ATE computation failed: {e}")
        import traceback; traceback.print_exc()
        return None

    # --- RPE (unaligned, 1-frame delta) ---
    rpe_stats = {}
    rpe_per_frame = None
    try:
        rpe_metric = metrics.RPE(metrics.PoseRelation.translation_part, delta=1,
                                 delta_unit=metrics.Unit.frames,
                                 all_pairs=False)
        rpe_metric.process_data((traj_est_sync, traj_gt_sync))

        all_rpe_stats = rpe_metric.get_all_statistics()
        rpe_stats = {
            'mean':   float(all_rpe_stats.get('mean', -1)),
            'rmse':   float(all_rpe_stats.get('rmse', -1)),
            'max':    float(all_rpe_stats.get('max', -1)),
            'min':    float(all_rpe_stats.get('min', -1)),
            'median': float(all_rpe_stats.get('median', -1)),
            'std':    float(all_rpe_stats.get('std', -1)),
        }
        rpe_per_frame = rpe_metric.error.copy()
        print(f"  [evo] RPE (unaligned, Δ=1 frame): "
              f"mean={rpe_stats['mean']:.4f}, rmse={rpe_stats['rmse']:.4f}")
    except Exception as e:
        print(f"  [evo] RPE computation failed: {e}")

    # --- RPE ~1s delta ---
    rpe_1s_stats = {}
    rpe_1s_per_frame = None
    delta_1s = 2
    try:
        if len(est_aligned_ts) > 2:
            mean_dt = np.mean(np.diff(est_aligned_ts))
            delta_1s = max(1, int(1.0 / mean_dt)) if mean_dt > 0 else 2
        else:
            delta_1s = 2

        rpe_1s_metric = metrics.RPE(metrics.PoseRelation.translation_part,
                                    delta=delta_1s, delta_unit=metrics.Unit.frames,
                                    all_pairs=False)
        rpe_1s_metric.process_data((traj_est_sync, traj_gt_sync))

        all_rpe_1s = rpe_1s_metric.get_all_statistics()
        rpe_1s_stats = {
            'mean':   float(all_rpe_1s.get('mean', -1)),
            'rmse':   float(all_rpe_1s.get('rmse', -1)),
            'max':    float(all_rpe_1s.get('max', -1)),
            'min':    float(all_rpe_1s.get('min', -1)),
            'median': float(all_rpe_1s.get('median', -1)),
            'std':    float(all_rpe_1s.get('std', -1)),
        }
        rpe_1s_per_frame = rpe_1s_metric.error.copy()
    except Exception:
        pass

    return {
        'ate': ate_stats,
        'rpe': rpe_stats,
        'ate_per_frame': ate_per_frame,
        'rpe_per_frame': rpe_per_frame,
        'rpe_1s_per_frame': rpe_1s_per_frame,
        'rpe_1s': rpe_1s_stats,
        'rpe_1s_delta': delta_1s,
        'est_aligned': est_aligned_xyz,
        'gt_synced': gt_sync_pos,
        'est_aligned_ts': est_aligned_ts,
        'n_matched': len(est_aligned_xyz),
    }


def stats_str(values, unit="m"):
    """Format statistics of an array as a readable string."""
    if len(values) == 0:
        return "N/A"
    return (f"mean={np.mean(values):.4f} {unit}, "
            f"rmse={np.sqrt(np.mean(values**2)):.4f} {unit}, "
            f"max={np.max(values):.4f} {unit}, "
            f"min={np.min(values):.4f} {unit}, "
            f"median={np.median(values):.4f} {unit}, "
            f"p95={np.percentile(values, 95):.4f} {unit}, "
            f"std={np.std(values):.4f} {unit}")


def stats_dict(values):
    """Return dict of statistics for an array."""
    if len(values) == 0:
        return {}
    return {
        'mean':   float(np.mean(values)),
        'rmse':   float(np.sqrt(np.mean(values**2))),
        'max':    float(np.max(values)),
        'min':    float(np.min(values)),
        'median': float(np.median(values)),
        'p95':    float(np.percentile(values, 95)),
        'std':    float(np.std(values)),
        'count':  len(values),
    }


# ===========================================================================
# Plotting Functions
# ===========================================================================

def plot_trajectory_xy(state, log_dir, gt=None):
    """Top-down (XY) trajectory view with optional GT overlay."""
    fig, ax = plt.subplots(figsize=(8, 6))
    # Ground truth
    if gt is not None and gt['x'] is not None and len(gt['x']) > 0:
        ax.plot(gt['x'], gt['y'], 'b--', linewidth=1.2, alpha=0.6,
                label='Ground Truth')
    # Estimate
    ax.plot(state["x"], state["y"], 'r-', linewidth=1.0, alpha=0.9,
            label='Estimate')
    ax.plot(state["x"][0], state["y"][0], 'go', markersize=8, label='Start')
    ax.plot(state["x"][-1], state["y"][-1], 'ro', markersize=8, label='End')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory — Top-down View (XY)")
    ax.axis("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = log_dir / "trajectory_xy.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_trajectory_3d(state, log_dir, gt=None):
    """3D trajectory view with optional GT overlay."""
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    # Ground truth
    if gt is not None and gt['x'] is not None and len(gt['x']) > 0:
        ax.plot(gt['x'], gt['y'], gt['z'], 'b--', linewidth=0.8, alpha=0.5,
                label='Ground Truth')
    # Estimate
    ax.plot(state["x"], state["y"], state["z"], 'r-', linewidth=0.8,
            label='Estimate')
    ax.plot([state["x"][0]], [state["y"][0]], [state["z"][0]],
            'go', markersize=6, label='Start')
    ax.plot([state["x"][-1]], [state["y"][-1]], [state["z"][-1]],
            'ro', markersize=6, label='End')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Trajectory — 3D View")
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = log_dir / "trajectory_3d.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_position_over_time(state, log_dir, gt=None):
    """X, Y, Z position vs time with optional GT overlay."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    colors_est = ['#e74c3c', '#2ecc71', '#3498db']
    for ax, key, label, ce in zip(
        axes, ["x", "y", "z"], ["X", "Y", "Z"], colors_est
    ):
        ax.plot(state["t"], state[key], color=ce, linewidth=1.0, alpha=0.95,
                label=f'Est. {label}')
        if gt is not None and gt[key] is not None and len(gt[key]) > 0:
            ax.plot(gt['t'], gt[key], color='black', linewidth=0.6, alpha=0.5,
                    linestyle='--', label=f'GT {label}')
        ax.set_ylabel(f"{label} (m)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc='upper right')
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Position vs Time")
    fig.tight_layout()
    path = log_dir / "position_time.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_velocity_over_time(state, log_dir):
    """Vx, Vy, Vz, and speed vs time (estimate only — GT velocity not in TUM)."""
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    speed = np.sqrt(state["vx"]**2 + state["vy"]**2 + state["vz"]**2)
    for ax, key, label, color in zip(
        axes[:3], ["vx", "vy", "vz"], ["Vx", "Vy", "Vz"],
        ['#e74c3c', '#2ecc71', '#3498db']
    ):
        ax.plot(state["t"], state[key], color=color, linewidth=0.6)
        ax.set_ylabel(f"{label} (m/s)")
        ax.grid(True, alpha=0.3)
    axes[3].plot(state["t"], speed, color='#9b59b6', linewidth=0.8)
    axes[3].set_ylabel("Speed (m/s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].set_xlabel("Time (s)")
    fig.suptitle("Velocity vs Time (Estimate)")
    fig.tight_layout()
    path = log_dir / "velocity_time.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_convergence(opt_rows, log_dir):
    """Bar chart: final error and chi² per pass."""
    if not opt_rows:
        return
    n = len(opt_rows)
    names = [r["pass"].replace('"', '') for r in opt_rows]
    errors = [float(r["final_error"]) for r in opt_rows]
    chi2s = [float(r["reduced_chi2"]) for r in opt_rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    colors_e = plt.cm.Blues(np.linspace(0.4, 0.9, n))
    ax1.bar(range(n), errors, color=colors_e, edgecolor='#333', linewidth=0.5)
    ax1.set_xticks(range(n))
    ax1.set_xticklabels(names, rotation=25, ha='right', fontsize=7)
    ax1.set_ylabel("Final Error")
    ax1.set_title("Error per Optimization Pass")
    ax1.grid(True, alpha=0.3, axis='y')

    colors_c = plt.cm.Oranges(np.linspace(0.4, 0.9, n))
    ax2.bar(range(n), chi2s, color=colors_c, edgecolor='#333', linewidth=0.5)
    ax2.set_xticks(range(n))
    ax2.set_xticklabels(names, rotation=25, ha='right', fontsize=7)
    ax2.set_ylabel("Reduced χ²")
    ax2.set_title("Chi² per Optimization Pass (lower = better)")
    ax2.grid(True, alpha=0.3, axis='y')

    fig.suptitle("Optimization Convergence", fontsize=13, fontweight='bold')
    fig.tight_layout()
    path = log_dir / "convergence.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_covariance_diag(state, log_dir):
    """Plot covariance diagonal from covariance_diag.csv."""
    path = log_dir / "covariance_diag.csv"
    if not path.exists():
        return
    cov_data = load_csv(path, ["sigma_x", "sigma_y", "sigma_z"])
    if cov_data is None or len(cov_data["sigma_x"]) == 0:
        return
    # Check if all values are -1 (placeholder)
    if np.all(cov_data["sigma_x"] < 0):
        print("  [skip] covariance_diag: all placeholders (no marginal data)")
        return

    st = load_csv(log_dir / "state_trace.csv", ["t", "x", "y", "z"])
    if st is None:
        return
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for ax, key, color in zip(axes, ["sigma_x", "sigma_y", "sigma_z"],
                              ['#e74c3c', '#2ecc71', '#3498db']):
        ax.plot(st["t"][:len(cov_data[key])], cov_data[key],
                color=color, linewidth=0.8)
        ax.set_ylabel(f"σ_{key[-1]} (m)")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Position Uncertainty (σ) vs Time")
    fig.tight_layout()
    path_out = log_dir / "covariance_diag.png"
    fig.savefig(path_out, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path_out.name}")


def plot_residual_heatmap(log_dir):
    """Plot per-anchor residual heatmap from residuals.csv."""
    path = log_dir / "residuals.csv"
    if not path.exists():
        return
    # Parse column names to find anchor columns
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)
    if len(rows) < 2:
        return
    # Extract anchor residual columns
    resid_cols = [i for i, h in enumerate(header) if h.startswith("resid_a")]
    if not resid_cols:
        return
    data = np.array([[float(r[i]) if r[i] != '-1' else np.nan
                       for i in resid_cols] for r in rows])
    # Transpose: rows=anchors, cols=keyframes
    data = data.T
    fig, ax = plt.subplots(figsize=(12, 4))
    im = ax.imshow(data, aspect='auto', cmap='RdYlGn_r',
                   vmin=0, vmax=np.nanmax(data) * 0.8,
                   interpolation='nearest')
    ax.set_xlabel("Keyframe Index")
    ax.set_ylabel("Anchor")
    anchor_labels = [h.replace("resid_", "") for h in header if h.startswith("resid_a")]
    ax.set_yticks(range(len(anchor_labels)))
    ax.set_yticklabels(anchor_labels)
    ax.set_title("Per-Anchor Residual RMSE Over Keyframes")
    plt.colorbar(im, ax=ax, label="RMSE (m)")
    fig.tight_layout()
    path_out = log_dir / "residual_heatmap.png"
    fig.savefig(path_out, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path_out.name}")


# ===========================================================================
# Trajectory Comparison Plots (GT vs Estimate)
# ===========================================================================

def plot_trajectory_compare_xy(gt_pts, est_aligned, errors, log_dir):
    """Top-down XY comparison with error color-coding."""
    fig, ax = plt.subplots(figsize=(10, 8))
    # GT trajectory (dashed blue)
    ax.plot(gt_pts[:, 0], gt_pts[:, 1], 'b--', linewidth=1.2,
            alpha=0.7, label='Ground Truth')
    # Estimate trajectory with error color-coding
    norm = plt.Normalize(vmin=0, vmax=max(np.percentile(errors, 95), 0.01))
    points = np.column_stack([est_aligned[:, 0], est_aligned[:, 1]])
    for i in range(len(points) - 1):
        seg_err = (errors[i] + errors[i+1]) / 2.0
        ax.plot(points[i:i+2, 0], points[i:i+2, 1],
                color=plt.cm.hot(norm(seg_err)), linewidth=1.0, alpha=0.8)
    sm = plt.cm.ScalarMappable(cmap='hot', norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, label='ATE (m)', shrink=0.8)
    ax.plot(gt_pts[0, 0], gt_pts[0, 1], 'go', markersize=8, label='Start')
    ax.plot(gt_pts[-1, 0], gt_pts[-1, 1], 'ro', markersize=8, label='End')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory Comparison — Top-down (XY)\nColor = Absolute Trajectory Error")
    ax.axis("equal")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = log_dir / "trajectory_compare_xy.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_trajectory_compare_3d(gt_pts, est_aligned, errors, log_dir):
    """3D trajectory comparison with error color-coding."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    norm = plt.Normalize(vmin=0, vmax=max(np.percentile(errors, 95), 0.01))
    for i in range(len(est_aligned) - 1):
        seg_err = (errors[i] + errors[i+1]) / 2.0
        ax.plot(est_aligned[i:i+2, 0], est_aligned[i:i+2, 1],
                est_aligned[i:i+2, 2],
                color=plt.cm.hot(norm(seg_err)), linewidth=0.8)
    ax.plot(gt_pts[:, 0], gt_pts[:, 1], gt_pts[:, 2], 'b--',
            linewidth=0.8, alpha=0.5, label='GT')
    ax.plot([gt_pts[0, 0]], [gt_pts[0, 1]], [gt_pts[0, 2]],
            'go', markersize=6, label='Start')
    ax.plot([gt_pts[-1, 0]], [gt_pts[-1, 1]], [gt_pts[-1, 2]],
            'ro', markersize=6, label='End')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Trajectory Comparison — 3D\nColor = ATE")
    ax.legend()
    fig.tight_layout()
    path = log_dir / "trajectory_compare_3d.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_ate_timeline(est_t, errors, log_dir, target_p95=0.10):
    """ATE evolution over time with RMSE/mean/median/P95 annotations."""
    rmse = np.sqrt(np.mean(errors**2))
    mean_v = np.mean(errors)
    median_v = np.median(errors)
    p95_v = np.percentile(errors, 95)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(est_t, errors, 'r-', linewidth=0.8, alpha=0.8, label='ATE')
    ax.axhline(rmse, color='#e74c3c', linestyle='-', linewidth=1.5,
               label=f'RMSE = {rmse:.4f} m')
    ax.axhline(mean_v, color='#f39c12', linestyle='--', linewidth=1.0,
               label=f'Mean = {mean_v:.4f} m')
    ax.axhline(median_v, color='#2ecc71', linestyle='--', linewidth=1.0,
               label=f'Median = {median_v:.4f} m')
    ax.axhline(p95_v, color='#9b59b6', linestyle=':', linewidth=1.5,
               label=f'P95 = {p95_v:.4f} m')
    # Target line
    ax.axhline(target_p95, color='#e74c3c', linestyle='-.', linewidth=1.0,
               alpha=0.6, label=f'Target P95 = {target_p95:.2f} m')
    if p95_v > target_p95:
        ax.fill_between(est_t, target_p95, max(errors), alpha=0.08, color='red')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("ATE (m)")
    ax.set_title(f"Absolute Trajectory Error vs Time\n"
                 f"RMSE={rmse:.4f} | Mean={mean_v:.4f} | Median={median_v:.4f} | "
                 f"P95={p95_v:.4f} m {'✓' if p95_v <= target_p95 else '✗ TARGET NOT MET'}")
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = log_dir / "ate_timeline.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_rpe_timeline(rpe_errors, rpe_t, delta, log_dir):
    """RPE evolution over timeline with RMSE/mean/median annotations."""
    if len(rpe_errors) == 0:
        return
    rmse = np.sqrt(np.mean(rpe_errors**2))
    mean_v = np.mean(rpe_errors)
    median_v = np.median(rpe_errors)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(rpe_t, rpe_errors, 'b-', linewidth=0.6, alpha=0.8, label='RPE')
    ax.axhline(rmse, color='#e74c3c', linestyle='-', linewidth=1.5,
               label=f'RMSE = {rmse:.4f} m')
    ax.axhline(mean_v, color='#f39c12', linestyle='--', linewidth=1.0,
               label=f'Mean = {mean_v:.4f} m')
    ax.axhline(median_v, color='#2ecc71', linestyle='--', linewidth=1.0,
               label=f'Median = {median_v:.4f} m')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(f"RPE (m) — Δ = {delta} frames")
    ax.set_title(f"Relative Pose Error vs Time\n"
                 f"RMSE={rmse:.4f} | Mean={mean_v:.4f} | Median={median_v:.4f} m")
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = log_dir / "rpe_timeline.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_error_distributions(ate_errors, rpe_errors, delta, log_dir,
                             target_p95=0.10):
    """Histogram of ATE and RPE errors with statistics overlay + P95 target."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # ATE histogram
    ax1.hist(ate_errors, bins=40, color='#e74c3c', alpha=0.7, edgecolor='#333',
             linewidth=0.3)
    s = stats_dict(ate_errors)
    ax1.axvline(s['rmse'], color='#e74c3c', linestyle='-', linewidth=2.0,
                label=f"RMSE={s['rmse']:.4f}")
    ax1.axvline(s['mean'], color='#f39c12', linestyle='-', linewidth=1.5,
                label=f"Mean={s['mean']:.4f}")
    ax1.axvline(s['median'], color='#2ecc71', linestyle='--', linewidth=1.5,
                label=f"Median={s['median']:.4f}")
    ax1.axvline(s['p95'], color='#9b59b6', linestyle=':', linewidth=2.0,
                label=f"P95={s['p95']:.4f}")
    ax1.axvline(target_p95, color='#e74c3c', linestyle='-.', linewidth=1.0,
                alpha=0.6, label=f"Target={target_p95:.2f}")
    p95_pass = "✓ PASS" if s['p95'] <= target_p95 else "✗ FAIL"
    ax1.set_xlabel("ATE (m)")
    ax1.set_ylabel("Frequency")
    ax1.set_title(f"ATE Distribution  [{p95_pass}]\n"
                  f"RMSE={s['rmse']:.4f} | Mean={s['mean']:.4f} | "
                  f"Median={s['median']:.4f} | P95={s['p95']:.4f} m")
    ax1.legend(fontsize=6, loc='upper right')
    ax1.grid(True, alpha=0.3, axis='y')

    # RPE histogram
    if len(rpe_errors) > 0:
        ax2.hist(rpe_errors, bins=40, color='#3498db', alpha=0.7,
                 edgecolor='#333', linewidth=0.3)
        s2 = stats_dict(rpe_errors)
        ax2.axvline(s2['rmse'], color='#e74c3c', linestyle='-', linewidth=2.0,
                    label=f"RMSE={s2['rmse']:.4f}")
        ax2.axvline(s2['mean'], color='#f39c12', linestyle='-', linewidth=1.5,
                    label=f"Mean={s2['mean']:.4f}")
        ax2.axvline(s2['median'], color='#2ecc71', linestyle='--', linewidth=1.5,
                    label=f"Median={s2['median']:.4f}")
        ax2.axvline(s2['p95'], color='#9b59b6', linestyle=':', linewidth=2.0,
                    label=f"P95={s2['p95']:.4f}")
        ax2.set_xlabel(f"RPE (m) — Δ={delta} frames")
        ax2.set_title(f"RPE Distribution\n"
                      f"RMSE={s2['rmse']:.4f} | Mean={s2['mean']:.4f} | "
                      f"Median={s2['median']:.4f} | P95={s2['p95']:.4f} m")
        ax2.legend(fontsize=6, loc='upper right')
    else:
        ax2.text(0.5, 0.5, "No RPE data", ha='center', va='center',
                 transform=ax2.transAxes, fontsize=14)
    ax2.grid(True, alpha=0.3, axis='y')

    fig.tight_layout()
    path = log_dir / "error_distributions.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def run_trajectory_comparison(log_dir, target_p95=0.10):
    """Load GT and estimate trajectories, compute ATE/RPE via evo API, generate plots.
    
    Uses evo Python API for alignment + ATE/RPE computation. All stats, per-frame
    errors, and aligned trajectories come from evo for consistency with evo CLI.
    Falls back to our own Umeyama implementation if evo is unavailable.
    
    Args:
        log_dir: path to log directory
        target_p95: target P95 ATE in meters (default 0.10 = 10 cm)
    
    Returns dict of statistics or None if trajectories unavailable.
    """
    est_path = log_dir / "trajectory.txt"
    gt_path  = log_dir / "groundtruth.txt"

    est = load_tum_trajectory(est_path)
    gt  = load_tum_trajectory(gt_path)

    if est is None or gt is None:
        print("  [skip] GT/est trajectory comparison: missing files")
        return None

    print(f"  Est poses: {len(est['t'])}, GT poses: {len(gt['t'])}")

    # --- Try evo first ---
    evo_data = call_evo(log_dir, est_path, gt_path)

    if evo_data:
        print("  [evo] Using evo Python API for all ATE/RPE computations")
        ate_stats    = evo_data['ate']
        rpe_stats    = evo_data['rpe']
        rpe_1s_stats = evo_data.get('rpe_1s', {})
        delta_1sec   = evo_data.get('rpe_1s_delta', 2)
        n_matched    = evo_data['n_matched']

        # evo-aligned trajectory + per-frame ATE
        est_aligned = evo_data['est_aligned']
        gt_synced   = evo_data['gt_synced']
        ate_errors  = evo_data['ate_per_frame'].ravel()  # 1D array
        est_t       = evo_data['est_aligned_ts']

        # evo RPE per-frame (unaligned)
        rpe_1f_per_frame = evo_data.get('rpe_per_frame')
        if rpe_1f_per_frame is not None:
            rpe_1f_per_frame = rpe_1f_per_frame.ravel()
        rpe_1s_per_frame = evo_data.get('rpe_1s_per_frame')
        if rpe_1s_per_frame is not None:
            rpe_1s_per_frame = rpe_1s_per_frame.ravel()

        # Compute P95 from evo's ATE per-frame data
        p95_ate = float(np.percentile(ate_errors, 95))
        ate_stats['p95'] = p95_ate
        ate_stats['count'] = len(ate_errors)

        # Also set P95 for RPE
        if rpe_1f_per_frame is not None and len(rpe_1f_per_frame) > 0:
            rpe_stats['p95'] = float(np.percentile(rpe_1f_per_frame, 95))
        if rpe_1s_per_frame is not None and len(rpe_1s_per_frame) > 0:
            rpe_1s_stats['p95'] = float(np.percentile(rpe_1s_per_frame, 95))

        p95_pass = p95_ate <= target_p95
        print(f"  ATE (evo): mean={ate_stats['mean']:.4f}, "
              f"rmse={ate_stats['rmse']:.4f}, median={ate_stats['median']:.4f}, "
              f"P95={p95_ate:.4f}")
        print(f"  RPE (evo, unaligned, Δ=1): mean={rpe_stats.get('mean',-1):.4f}, "
              f"rmse={rpe_stats.get('rmse',-1):.4f}")
        print(f"  ATE P95: {p95_ate:.4f} m "
              f"({'✓ PASS' if p95_pass else '✗ FAIL'} target={target_p95:.2f} m)")

    else:
        # --- Fallback: our own Umeyama + RPE ---
        print("  [ours] evo unavailable, using own Umeyama implementation")
        est_raw, gt_matched, est_t = match_trajectories(est, gt)
        if est_raw is None or len(est_raw) < 3:
            print("  [skip] GT/est comparison: too few matched poses")
            return None

        R, t, s, est_aligned = umeyama_alignment(est_raw, gt_matched)
        gt_synced = gt_matched
        ate_errors = compute_ape(est_aligned, gt_matched)
        ate_stats = stats_dict(ate_errors)
        n_matched = len(est_raw)

        # RPE on raw (unaligned) data
        rpe_1f_per_frame, rpe_1f_t = compute_rpe(est_raw, gt_matched, delta_frames=1)
        rpe_stats = stats_dict(rpe_1f_per_frame) if len(rpe_1f_per_frame) > 0 else {}

        if len(est_t) > 2:
            mean_dt = np.mean(np.diff(est_t))
        else:
            mean_dt = 0.5
        delta_1sec = max(1, int(1.0 / mean_dt)) if mean_dt > 0 else 2
        rpe_1s_per_frame, rpe_1s_t = compute_rpe(est_raw, gt_matched, delta_frames=delta_1sec)
        rpe_1s_stats = stats_dict(rpe_1s_per_frame) if len(rpe_1s_per_frame) > 0 else {}

        p95_pass = ate_stats['p95'] <= target_p95
        print(f"  Alignment: scale={s:.6f}")
        print(f"  ATE (ours): {stats_str(ate_errors)}")
        if len(rpe_1f_per_frame) > 0:
            print(f"  RPE (ours, unaligned, Δ=1): {stats_str(rpe_1f_per_frame)}")

    # --- Shared: generate all plots with the computed data ---
    plot_trajectory_compare_xy(gt_synced, est_aligned, ate_errors, log_dir)
    plot_trajectory_compare_3d(gt_synced, est_aligned, ate_errors, log_dir)
    plot_ate_timeline(est_t, ate_errors, log_dir, target_p95)

    # RPE timeline: use 1-frame RPE data
    if rpe_1f_per_frame is not None and len(rpe_1f_per_frame) > 0:
        rpe_t_1f = est_t[1:] if evo_data else est_t[1:len(rpe_1f_per_frame)+1]
        plot_rpe_timeline(rpe_1f_per_frame, rpe_t_1f[:len(rpe_1f_per_frame)],
                          delta=1, log_dir=log_dir)

    # Error distributions: ATE + RPE ~1s (fallback to 1-frame if no ~1s data)
    rpe_for_hist = rpe_1s_per_frame
    if rpe_for_hist is None or len(rpe_for_hist) == 0:
        rpe_for_hist = rpe_1f_per_frame
    if rpe_for_hist is None:
        rpe_for_hist = np.array([])
    plot_error_distributions(ate_errors, rpe_for_hist, delta_1sec, log_dir, target_p95)

    return {
        'ate': ate_stats,
        'rpe_1frame': rpe_stats,
        'rpe_1sec': rpe_1s_stats,
        'rpe_1sec_delta_frames': delta_1sec,
        'n_matched': n_matched,
        'alignment_scale': 1.0,  # evo's Umeyama keeps scale=1
        'target_p95': target_p95,
        'p95_pass': p95_pass,
        'using_evo': evo_data is not None,
    }


# ===========================================================================
# Markdown Report
# ===========================================================================

def generate_markdown_report(log_dir, summary, opt_rows, gt_data, traj_stats):
    """Generate a Markdown summary report with embedded images."""
    path = log_dir / "report.md"
    run_name = log_dir.name

    lines = []
    lines.append(f"# UWB-IMU FGO — Run Report")
    lines.append("")
    lines.append(f"**Run:** `{run_name}`  ")
    lines.append(f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")

    # --- Summary ---
    if summary:
        lines.append("## Summary")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("|--------|-------|")

        items = [
            ("total_elapsed_sec",     "Total Elapsed",   "{:.2f}", "s"),
            ("n_keyframes",           "Keyframes",       "{}",     ""),
            ("n_factors",             "Total Factors",   "{}",     ""),
            ("n_uwb_inliers",         "UWB Inliers",     "{}",     ""),
            ("n_uwb_total",           "UWB Total",       "{}",     ""),
            ("inlier_pct",            "Inlier Ratio",    "{:.1f}", "%"),
            ("ate_rmse_m",            "ATE RMSE (UWB)",  "{:.4f}", "m"),
            ("p95_error_m",           "P95 Error",       "{:.4f}", "m"),
        ]
        for key, label, fmt_str, unit in items:
            if key in summary:
                val = summary[key]
                val_str = fmt_str.format(val)
                lines.append(f"| {label} | {val_str} {unit} |")

        lines.append("")

        # Per-Anchor RMSE
        if "anchor_rmse_m" in summary:
            lines.append("### Per-Anchor RMSE")
            lines.append("")
            lines.append("| Anchor | RMSE (m) |")
            lines.append("|--------|----------|")
            for a_id, rmse in summary["anchor_rmse_m"].items():
                lines.append(f"| Anchor {a_id} | {rmse:.4f} |")
            lines.append("")

    # --- Trajectory Comparison (GT vs Estimate) ---
    if traj_stats:
        lines.append("## Trajectory Comparison (GT vs Estimate)")
        lines.append("")
        lines.append(f"**Matched poses:** {traj_stats['n_matched']}  ")
        if traj_stats.get('using_evo', False):
            lines.append(f"**Metrics source:** evo Python API (SE(3) Umeyama alignment)  ")
        lines.append(f"**Alignment scale:** {traj_stats['alignment_scale']:.6f}")
        lines.append("")

        ate = traj_stats['ate']
        target = traj_stats.get('target_p95', 0.10)
        p95_pass = traj_stats.get('p95_pass', False)
        p95_icon = "✅" if p95_pass else "❌"

        lines.append(f"### {p95_icon} ATE (Absolute Trajectory Error) — P95 Target: ≤ {target:.2f} m")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("|--------|-------|")
        lines.append(f"| RMSE | {ate['rmse']:.4f} m |")
        lines.append(f"| Mean | {ate['mean']:.4f} m |")
        lines.append(f"| Median | {ate['median']:.4f} m |")
        lines.append(f"| **P95** | **{ate['p95']:.4f} m** {'✅' if p95_pass else '❌ NOT MET'} |")
        lines.append(f"| Max | {ate['max']:.4f} m |")
        lines.append(f"| Min | {ate['min']:.4f} m |")
        lines.append(f"| Std | {ate['std']:.4f} m |")
        lines.append(f"| Samples | {ate['count']} |")
        lines.append("")

        if p95_pass:
            lines.append(f"> ✅ **P95 target ({target:.2f} m) MET** — P95 ATE = {ate['p95']:.4f} m ≤ {target:.2f} m")
        else:
            margin = ate['p95'] - target
            lines.append(f"> ❌ **P95 target ({target:.2f} m) NOT MET** — P95 ATE = {ate['p95']:.4f} m, exceeds target by {margin:.4f} m")
        lines.append("")

        rpe1 = traj_stats['rpe_1frame']
        if rpe1:
            lines.append("### RPE (Relative Pose Error, Δ=1 frame)")
            lines.append("")
            lines.append("| Metric | Value |")
            lines.append("|--------|-------|")
            lines.append(f"| RMSE | {rpe1['rmse']:.4f} m |")
            lines.append(f"| Mean | {rpe1['mean']:.4f} m |")
            lines.append(f"| Median | {rpe1['median']:.4f} m |")
            lines.append(f"| P95 | {rpe1['p95']:.4f} m |")
            lines.append(f"| Max | {rpe1['max']:.4f} m |")
            lines.append(f"| Min | {rpe1['min']:.4f} m |")
            lines.append(f"| Std | {rpe1['std']:.4f} m |")
            lines.append("")

        rpe_s = traj_stats['rpe_1sec']
        if rpe_s:
            df = traj_stats.get('rpe_1sec_delta_frames', '?')
            lines.append(f"### RPE (Relative Pose Error, Δ~1s = {df} frames)")
            lines.append("")
            lines.append("| Metric | Value |")
            lines.append("|--------|-------|")
            lines.append(f"| RMSE | {rpe_s['rmse']:.4f} m |")
            lines.append(f"| Mean | {rpe_s['mean']:.4f} m |")
            lines.append(f"| Median | {rpe_s['median']:.4f} m |")
            lines.append(f"| P95 | {rpe_s['p95']:.4f} m |")
            lines.append(f"| Max | {rpe_s['max']:.4f} m |")
            lines.append(f"| Min | {rpe_s['min']:.4f} m |")
            lines.append(f"| Std | {rpe_s['std']:.4f} m |")
            lines.append("")

    # --- GT Comparison (legacy gt_comparison.csv) ---
    if gt_data:
        lines.append("## Ground Truth Comparison (Legacy)")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("|--------|-------|")
        for k, v in gt_data.items():
            unit = "m" if k not in ("MatchedFrames", "GT_Total") else ""
            if isinstance(v, float) and v == int(v):
                lines.append(f"| {k} | {int(v)} {unit} |")
            else:
                lines.append(f"| {k} | {v:.4f} {unit} |")
        lines.append("")

    # --- Optimization Passes ---
    if opt_rows:
        lines.append("## Optimization Passes")
        lines.append("")
        lines.append("| Pass | Initial Error | Final Error | χ²/dof | Inliers | Outliers | Reject Rounds | Time (s) |")
        lines.append("|------|---------------|-------------|--------|---------|----------|---------------|----------|")
        for r in opt_rows:
            pass_name = r['pass'].strip('"')
            lines.append(
                f"| {pass_name} "
                f"| {float(r['initial_error']):.2f} "
                f"| {float(r['final_error']):.2f} "
                f"| {float(r['reduced_chi2']):.3f} "
                f"| {r['inliers']} "
                f"| {r['outliers']} "
                f"| {r.get('num_reject_rounds', '-')} "
                f"| {float(r.get('elapsed_sec', 0)):.2f} |")
        lines.append("")

    # --- Plots ---
    # Basic plots now include GT overlay when groundtruth.txt is available
    base_plots = [
        ("trajectory_xy.png",           "Trajectory — Top-down (XY, GT+Estimate)"),
        ("trajectory_3d.png",           "Trajectory — 3D View (GT+Estimate)"),
        ("position_time.png",           "Position vs Time (GT+Estimate)"),
        ("velocity_time.png",           "Velocity vs Time (Estimate)"),
        ("convergence.png",             "Optimization Convergence"),
        ("covariance_diag.png",         "Position Uncertainty (σ)"),
        ("residual_heatmap.png",        "Per-Anchor Residual Heatmap"),
    ]
    # Comparison plots (only if GT exists)
    comp_plots = [
        ("trajectory_compare_xy.png",   "GT vs Estimate — Top-down (XY, error-colored)"),
        ("trajectory_compare_3d.png",   "GT vs Estimate — 3D (error-colored)"),
        ("ate_timeline.png",            "ATE — Error vs Time"),
        ("rpe_timeline.png",            "RPE — Relative Error vs Time"),
        ("error_distributions.png",     "ATE & RPE Distribution (histogram)"),
    ]

    has_comp = (log_dir / "trajectory_compare_xy.png").exists()
    plot_list = base_plots + (comp_plots if has_comp else [])

    lines.append("## Plots")
    lines.append("")
    for pf, title in plot_list:
        full = log_dir / pf
        if full.exists():
            lines.append(f"### {title}")
            lines.append("")
            lines.append(f"![]({pf})")
            lines.append("")

    # --- Footer ---
    lines.append("---")
    lines.append(f"*Generated by `tools/analyze_log.py` — {run_name}*")
    lines.append("")

    with open(path, 'w') as f:
        f.write("\n".join(lines))
    print(f"  [report] {path.name}")


# ===========================================================================
# Main
# ===========================================================================

def main():
    log_dir = resolve_log_dir(sys.argv)
    print(f"Analyzing: {log_dir}")

    summary = load_summary(log_dir)
    opt_rows = load_optimization_csv(log_dir)
    gt_data = load_gt_comparison(log_dir)
    state = load_csv(log_dir / "state_trace.csv",
                     ["t", "x", "y", "z", "vx", "vy", "vz"])

    # -- GT vs Estimate trajectory comparison --
    print("Running trajectory comparison (GT vs Estimate)...")
    traj_stats = run_trajectory_comparison(log_dir)

    # -- Load GT trajectory for overlay on basic plots --
    gt = load_tum_trajectory(log_dir / "groundtruth.txt")

    # -- Plots --
    if state is not None and len(state["t"]) > 1:
        print("Generating basic plots...")
        plot_trajectory_xy(state, log_dir, gt)
        plot_trajectory_3d(state, log_dir, gt)
        plot_position_over_time(state, log_dir, gt)
        plot_velocity_over_time(state, log_dir)
    else:
        print("WARNING: No valid state trace data.")

    if opt_rows:
        plot_convergence(opt_rows, log_dir)

    plot_covariance_diag(state, log_dir)
    plot_residual_heatmap(log_dir)

    # -- Report --
    print("Generating report...")
    generate_markdown_report(log_dir, summary, opt_rows, gt_data, traj_stats)

    print(f"\nDone! Open in VS Code or browser:")
    print(f"   {log_dir / 'report.md'}")


if __name__ == "__main__":
    main()
