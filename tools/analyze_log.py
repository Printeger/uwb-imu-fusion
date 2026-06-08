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


# ===========================================================================
# Plotting Functions
# ===========================================================================

def plot_trajectory_xy(state, log_dir):
    """Top-down (XY) trajectory view."""
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(state["x"], state["y"], 'r-', linewidth=1.0, alpha=0.9)
    ax.plot(state["x"][0], state["y"][0], 'go', markersize=6, label='Start')
    ax.plot(state["x"][-1], state["y"][-1], 'ro', markersize=6, label='End')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory — Top-down View (XY)")
    ax.axis("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = log_dir / "trajectory_xy.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_trajectory_3d(state, log_dir):
    """3D trajectory view."""
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(state["x"], state["y"], state["z"], 'r-', linewidth=0.8)
    ax.plot([state["x"][0]], [state["y"][0]], [state["z"][0]],
            'go', markersize=5, label='Start')
    ax.plot([state["x"][-1]], [state["y"][-1]], [state["z"][-1]],
            'ro', markersize=5, label='End')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Trajectory — 3D View")
    ax.legend()
    fig.tight_layout()
    path = log_dir / "trajectory_3d.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_position_over_time(state, log_dir):
    """X, Y, Z position vs time."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for ax, key, label, color in zip(
        axes, ["x", "y", "z"], ["X", "Y", "Z"], ['#e74c3c', '#2ecc71', '#3498db']
    ):
        ax.plot(state["t"], state[key], color=color, linewidth=0.6)
        ax.set_ylabel(f"{label} (m)")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Position vs Time")
    fig.tight_layout()
    path = log_dir / "position_time.png"
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    print(f"  [plot] {path.name}")


def plot_velocity_over_time(state, log_dir):
    """Vx, Vy, Vz, and speed vs time."""
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
    fig.suptitle("Velocity vs Time")
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
# Markdown Report
# ===========================================================================

def generate_markdown_report(log_dir, summary, opt_rows, gt_data):
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

        def s(key, fmt_str, unit=""):
            if key in summary:
                val = summary[key]
                val_str = fmt_str.format(val)
                return f"| {val_str} {unit} |"
            return ""

        items = [
            ("total_elapsed_sec",     "Total Elapsed",   "{:.2f}", "s"),
            ("n_keyframes",           "Keyframes",       "{}",     ""),
            ("n_factors",             "Total Factors",   "{}",     ""),
            ("n_uwb_inliers",         "UWB Inliers",     "{}",     ""),
            ("n_uwb_total",           "UWB Total",       "{}",     ""),
            ("inlier_pct",            "Inlier Ratio",    "{:.1f}", "%"),
            ("ate_rmse_m",            "ATE RMSE",        "{:.4f}", "m"),
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

    # --- GT Comparison ---
    if gt_data:
        lines.append("## Ground Truth Comparison")
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
    plot_files = [
        ("trajectory_xy.png",      "Trajectory — Top-down (XY)"),
        ("trajectory_3d.png",      "Trajectory — 3D View"),
        ("position_time.png",      "Position vs Time"),
        ("velocity_time.png",      "Velocity vs Time"),
        ("convergence.png",        "Optimization Convergence"),
        ("covariance_diag.png",    "Position Uncertainty (σ)"),
        ("residual_heatmap.png",   "Per-Anchor Residual Heatmap"),
    ]
    lines.append("## Plots")
    lines.append("")
    for pf, title in plot_files:
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

    # -- Plots --
    if state is not None and len(state["t"]) > 1:
        print("Generating plots...")
        plot_trajectory_xy(state, log_dir)
        plot_trajectory_3d(state, log_dir)
        plot_position_over_time(state, log_dir)
        plot_velocity_over_time(state, log_dir)
    else:
        print("WARNING: No valid state trace data.")

    if opt_rows:
        plot_convergence(opt_rows, log_dir)

    plot_covariance_diag(state, log_dir)
    plot_residual_heatmap(log_dir)

    # -- Report --
    print("Generating report...")
    generate_markdown_report(log_dir, summary, opt_rows, gt_data)

    print(f"\nDone! Open in VS Code or browser:")
    print(f"   {log_dir / 'report.md'}")


if __name__ == "__main__":
    main()
