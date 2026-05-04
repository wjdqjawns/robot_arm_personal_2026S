#!/usr/bin/env python3
"""3D end-effector trajectory visualization for Piper arm simulation logs."""

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="3D trajectory analysis for piper_arm.csv")
    parser.add_argument(
        "--input",
        default="data/logs/piper_arm.csv",
        help="Path to the simulation CSV log",
    )
    parser.add_argument(
        "--output-dir",
        default="data/fig",
        help="Directory for plots",
    )
    return parser.parse_args()


def load_csv(path: Path) -> Tuple[List[Dict[str, str]], List[str]]:
    """Load CSV file and return rows and fieldnames."""
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        if not reader.fieldnames:
            raise ValueError(f"{path} has no header")
        return rows, list(reader.fieldnames)


def get_series(rows: Sequence[Dict[str, str]], key: str) -> List[float]:
    """Extract a series of values from CSV rows by key."""
    values: List[float] = []
    for row in rows:
        raw = row.get(key, "")
        values.append(float(raw) if raw not in (None, "") else 0.0)
    return values


def main() -> int:
    args = parse_args()
    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows, fieldnames = load_csv(input_path)
    if not rows:
        raise SystemExit(f"No samples found in {input_path}")

    # Extract time and EE positions
    t = get_series(rows, "t")
    ee_pos_x = get_series(rows, "ee_pos_x")
    ee_pos_y = get_series(rows, "ee_pos_y")
    ee_pos_z = get_series(rows, "ee_pos_z")
    ee_des_pos_x = get_series(rows, "ee_des_pos_x")
    ee_des_pos_y = get_series(rows, "ee_des_pos_y")
    ee_des_pos_z = get_series(rows, "ee_des_pos_z")

    if not ee_pos_x:
        raise SystemExit("CSV does not contain EE position data (ee_pos_x, ee_pos_y, ee_pos_z)")

    # Create 3D trajectory plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Plot actual trajectory
    ax.plot(ee_pos_x, ee_pos_y, ee_pos_z, "r-", linewidth=2, label="Actual EE path", alpha=0.8)
    ax.scatter(ee_pos_x[0], ee_pos_y[0], ee_pos_z[0], color="red", s=100, marker="o", label="Start (actual)")
    ax.scatter(ee_pos_x[-1], ee_pos_y[-1], ee_pos_z[-1], color="darkred", s=100, marker="s", label="End (actual)")

    # Plot desired trajectory
    ax.plot(ee_des_pos_x, ee_des_pos_y, ee_des_pos_z, "g--", linewidth=2, label="Desired EE path", alpha=0.8)
    ax.scatter(ee_des_pos_x[0], ee_des_pos_y[0], ee_des_pos_z[0], color="green", s=100, marker="o", label="Start (desired)")
    ax.scatter(ee_des_pos_x[-1], ee_des_pos_y[-1], ee_des_pos_z[-1], color="darkgreen", s=100, marker="s", label="End (desired)")

    ax.set_xlabel("X [m]", fontsize=12)
    ax.set_ylabel("Y [m]", fontsize=12)
    ax.set_zlabel("Z [m]", fontsize=12)
    ax.set_title("End-Effector 3D Trajectory", fontsize=14, fontweight="bold")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    traj_path = output_dir / "ee_trajectory_3d.png"
    fig.savefig(traj_path, dpi=150, bbox_inches="tight")
    print(f"Wrote 3D trajectory plot to {traj_path}")

    # Create separate tracking error plot in 3D
    fig2 = plt.figure(figsize=(12, 10))
    ax2 = fig2.add_subplot(111, projection="3d")

    # Compute position error
    err_x = [ee_des_pos_x[i] - ee_pos_x[i] for i in range(len(t))]
    err_y = [ee_des_pos_y[i] - ee_pos_y[i] for i in range(len(t))]
    err_z = [ee_des_pos_z[i] - ee_pos_z[i] for i in range(len(t))]

    # Color points by error magnitude for visualization
    error_mag = [((err_x[i] ** 2 + err_y[i] ** 2 + err_z[i] ** 2) ** 0.5) for i in range(len(t))]

    scatter = ax2.scatter(ee_pos_x, ee_pos_y, ee_pos_z, c=error_mag, cmap="RdYlGn_r", s=20, alpha=0.6)
    cbar = fig2.colorbar(scatter, ax=ax2, label="Tracking Error [m]")

    ax2.set_xlabel("X [m]", fontsize=12)
    ax2.set_ylabel("Y [m]", fontsize=12)
    ax2.set_zlabel("Z [m]", fontsize=12)
    ax2.set_title("End-Effector Trajectory (colored by tracking error)", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3)

    error_path = output_dir / "ee_trajectory_error.png"
    fig2.savefig(error_path, dpi=150, bbox_inches="tight")
    print(f"Wrote error-colored trajectory plot to {error_path}")

    # Create 2D projections
    fig3, axes = plt.subplots(1, 3, figsize=(15, 5))

    # XY plane
    axes[0].plot(ee_pos_x, ee_pos_y, "r-", linewidth=2, label="Actual", alpha=0.8)
    axes[0].plot(ee_des_pos_x, ee_des_pos_y, "g--", linewidth=2, label="Desired", alpha=0.8)
    axes[0].scatter(ee_pos_x[0], ee_pos_y[0], color="red", s=100, marker="o")
    axes[0].scatter(ee_des_pos_x[0], ee_des_pos_y[0], color="green", s=100, marker="o")
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].set_title("XY Plane (Top View)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].axis("equal")

    # XZ plane
    axes[1].plot(ee_pos_x, ee_pos_z, "r-", linewidth=2, label="Actual", alpha=0.8)
    axes[1].plot(ee_des_pos_x, ee_des_pos_z, "g--", linewidth=2, label="Desired", alpha=0.8)
    axes[1].scatter(ee_pos_x[0], ee_pos_z[0], color="red", s=100, marker="o")
    axes[1].scatter(ee_des_pos_x[0], ee_des_pos_z[0], color="green", s=100, marker="o")
    axes[1].set_xlabel("X [m]")
    axes[1].set_ylabel("Z [m]")
    axes[1].set_title("XZ Plane (Side View)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    axes[1].axis("equal")

    # YZ plane
    axes[2].plot(ee_pos_y, ee_pos_z, "r-", linewidth=2, label="Actual", alpha=0.8)
    axes[2].plot(ee_des_pos_y, ee_des_pos_z, "g--", linewidth=2, label="Desired", alpha=0.8)
    axes[2].scatter(ee_pos_y[0], ee_pos_z[0], color="red", s=100, marker="o")
    axes[2].scatter(ee_des_pos_y[0], ee_des_pos_z[0], color="green", s=100, marker="o")
    axes[2].set_xlabel("Y [m]")
    axes[2].set_ylabel("Z [m]")
    axes[2].set_title("YZ Plane (Front View)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    axes[2].axis("equal")

    fig3.tight_layout()
    proj_path = output_dir / "ee_trajectory_projections.png"
    fig3.savefig(proj_path, dpi=150, bbox_inches="tight")
    print(f"Wrote projection plots to {proj_path}")

    # Print statistics
    max_error = max(error_mag)
    avg_error = sum(error_mag) / len(error_mag)
    print(f"\nTracking statistics:")
    print(f"  Max EE position error: {max_error:.6f} m")
    print(f"  Avg EE position error: {avg_error:.6f} m")
    print(f"  Duration: {t[-1] - t[0]:.2f} s")

    # Show plots
    try:
        plt.show()
    except Exception:
        pass  # Headless environment

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
