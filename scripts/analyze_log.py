#!/usr/bin/env python3
"""Offline analysis for Piper arm simulation logs.

Reads the CSV written by `arm_sim`, computes simple tracking metrics, and
generates summary plots for joint-space tracking and disturbance rejection.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import fmean
from typing import Dict, List, Sequence, Tuple

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline analysis for piper_arm.csv")
    parser.add_argument(
        "--input",
        default="data/logs/piper_arm.csv",
        help="Path to the simulation CSV log",
    )
    parser.add_argument(
        "--output-dir",
        default="data/fig",
        help="Directory for plots and summary files",
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=0,
        help="Joint index to plot in detail",
    )
    return parser.parse_args()


def detect_joint_count(fieldnames: Sequence[str], prefix: str) -> int:
    indices: List[int] = []
    for field in fieldnames:
        if field.startswith(prefix):
            suffix = field[len(prefix) :]
            if suffix.isdigit():
                indices.append(int(suffix))
    return (max(indices) + 1) if indices else 0


def load_csv(path: Path) -> Tuple[List[Dict[str, str]], List[str]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        if not reader.fieldnames:
            raise ValueError(f"{path} has no header")
        return rows, list(reader.fieldnames)


def get_series(rows: Sequence[Dict[str, str]], key: str) -> List[float]:
    values: List[float] = []
    for row in rows:
        raw = row.get(key, "")
        values.append(float(raw) if raw not in (None, "") else 0.0)
    return values


def rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def max_abs(values: Sequence[float]) -> float:
    return max((abs(v) for v in values), default=0.0)


def try_import_matplotlib():
    try:
        import matplotlib.pyplot as plt  # type: ignore

        return plt
    except Exception:
        return None


def save_summary(output_dir: Path, joint_count: int, t: List[float], q: List[List[float]], q_des: List[List[float]], d_hat: List[List[float]]) -> Path:
    summary_path = output_dir / "summary.txt"
    lines = [
        f"samples: {len(t)}",
        f"duration_s: {t[-1] - t[0] if len(t) > 1 else 0.0:.6f}",
        f"joints: {joint_count}",
        "",
    ]

    for i in range(joint_count):
        err = [q_des[k][i] - q[k][i] for k in range(len(t))]
        lines.append(
            f"joint {i}: rms_error={rms(err):.6f}, max_abs_error={max_abs(err):.6f}, rms_dhat={rms([d_hat[k][i] for k in range(len(t))]):.6f}"
        )

    summary_path.write_text("\n".join(lines) + "\n")
    return summary_path


def main() -> int:
    args = parse_args()
    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows, fieldnames = load_csv(input_path)
    if not rows:
        raise SystemExit(f"No samples found in {input_path}")

    joint_count = detect_joint_count(fieldnames, "q")
    if joint_count == 0:
        raise SystemExit("Could not infer joint count from CSV headers")

    t = get_series(rows, "t")
    q = [[float(row.get(f"q{i}", 0.0)) for i in range(joint_count)] for row in rows]
    dq = [[float(row.get(f"dq{i}", 0.0)) for i in range(joint_count)] for row in rows]
    q_des = [[float(row.get(f"q_des{i}", 0.0)) for i in range(joint_count)] for row in rows]
    tau_ctrl = [[float(row.get(f"tau_ctrl{i}", 0.0)) for i in range(joint_count)] for row in rows]
    d_hat = [[float(row.get(f"d_hat{i}", 0.0)) for i in range(joint_count)] for row in rows]

    summary_path = save_summary(output_dir, joint_count, t, q, q_des, d_hat)

    plt = try_import_matplotlib()
    if plt is None:
        print(f"Wrote summary to {summary_path}")
        print("matplotlib not available, so plots were skipped")
        return 0

    joint = max(0, min(args.joint, joint_count - 1))
    err_joint = [q_des[k][joint] - q[k][joint] for k in range(len(t))]

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(f"Piper Arm Offline Analysis - Joint {joint}")

    axes[0].plot(t, [row[joint] for row in q], label=f"q{joint}", color="tab:red")
    axes[0].plot(t, [row[joint] for row in q_des], label=f"q_des{joint}", color="tab:green", linestyle="--")
    axes[0].set_ylabel("rad")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, err_joint, label="tracking error", color="tab:blue")
    axes[1].set_ylabel("rad")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, [row[joint] for row in tau_ctrl], label=f"tau_ctrl{joint}", color="tab:orange")
    axes[2].set_ylabel("Nm")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, [row[joint] for row in d_hat], label=f"d_hat{joint}", color="tab:purple")
    axes[3].set_ylabel("Nm")
    axes[3].set_xlabel("time [s]")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    plot_path = output_dir / f"joint_{joint}_analysis.png"
    fig.savefig(plot_path, dpi=150)
    print(f"Wrote plot to {plot_path}")
    
    fig2, ax2 = plt.subplots(figsize=(8, 6))
    ax2.plot([row[joint] for row in q], [row[joint] for row in q_des], color="tab:green")
    lo = min(min(row[joint] for row in q), min(row[joint] for row in q_des))
    hi = max(max(row[joint] for row in q), max(row[joint] for row in q_des))
    ax2.plot([lo, hi], [lo, hi], linestyle="--", color="gray", linewidth=1)
    ax2.set_xlabel(f"q{joint} actual [rad]")
    ax2.set_ylabel(f"q{joint} desired [rad]")
    ax2.set_title(f"Tracking phase plot - joint {joint}")
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    phase_path = output_dir / f"joint_{joint}_phase.png"
    fig2.savefig(phase_path, dpi=150)
    print(f"Wrote plot to {phase_path}")
    print(f"Wrote summary to {summary_path}")
    
    # Display plots (will show in matplotlib window if available)
    try:
        plt.show()
    except Exception:
        pass  # Headless environment or display not available
    return 0


if __name__ == "__main__":
    raise SystemExit(main())