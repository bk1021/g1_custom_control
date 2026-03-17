#!/usr/bin/env python3
import argparse
import csv
import math
import os
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Overlay g1_evaluate CSV results per joint. "
                "Only trajectory_active==1 samples are plotted. "
                "Supports 3-mode data in a single CSV."
        )
    )
    parser.add_argument(
        "--csv",
        nargs="+",
        required=True,
        help="Input CSV files produced by g1_evaluate",
    )
    parser.add_argument(
        "--out-dir",
        default="eval_plots",
        help="Output directory for PNG plots",
    )
    parser.add_argument(
        "--plot-dq",
        action="store_true",
        help="Also generate dq plots in addition to q",
    )
    parser.add_argument(
        "--use-command",
        action="store_true",
        help="Use commanded data (cmd_*) instead of measured data (act_*)",
    )
    parser.add_argument(
        "--title-prefix",
        default="g1 evaluation",
        help="Prefix text added to each plot title",
    )
    return parser.parse_args()


def mode_rank(mode: str) -> int:
    order = {
        "cubic_hermite": 0,
        "linear": 1,
        "linear_dq0": 2,
    }
    return order.get(mode, 99)


def init_joint_series(joint_names: List[str]) -> Tuple[Dict[str, List[float]], Dict[str, List[float]], Dict[str, List[float]]]:
    t_by_joint: Dict[str, List[float]] = {j: [] for j in joint_names}
    q_by_joint: Dict[str, List[float]] = {j: [] for j in joint_names}
    dq_by_joint: Dict[str, List[float]] = {j: [] for j in joint_names}
    return t_by_joint, q_by_joint, dq_by_joint


def parse_csv(path: str, value_prefix: str) -> List[Dict[str, object]]:
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"CSV has no header: {path}")

        joint_q_cols = [c for c in reader.fieldnames if c.startswith(f"{value_prefix}_q_")]
        if not joint_q_cols:
            raise RuntimeError(
                f"No columns found for prefix '{value_prefix}_q_' in {path}. "
                "Expected columns like act_q_left_shoulder_pitch_joint"
            )

        joint_names = [c[len(f"{value_prefix}_q_") :] for c in joint_q_cols]

        grouped: Dict[Tuple[str, str], Dict[str, object]] = {}

        for row in reader:
            try:
                active = int(row.get("trajectory_active", "0"))
            except ValueError:
                active = 0
            if active != 1:
                continue

            mode = row.get("mode", "unknown")
            run_label = row.get("run_label", os.path.basename(path))
            key = (mode, run_label)

            if key not in grouped:
                t_by_joint, q_by_joint, dq_by_joint = init_joint_series(joint_names)
                grouped[key] = {
                    "mode": mode,
                    "run_label": run_label,
                    "first_stamp": None,
                    "t": t_by_joint,
                    "q": q_by_joint,
                    "dq": dq_by_joint,
                }

            series = grouped[key]
            stamp_ns = int(row["stamp_ns"])
            if series["first_stamp"] is None:
                series["first_stamp"] = stamp_ns

            t_s = (stamp_ns - int(series["first_stamp"])) * 1e-9

            for j in joint_names:
                q_col = f"{value_prefix}_q_{j}"
                dq_col = f"{value_prefix}_dq_{j}"
                if q_col not in row:
                    continue
                series["t"][j].append(t_s)
                series["q"][j].append(float(row[q_col]))
                if dq_col in row and row[dq_col] != "":
                    series["dq"][j].append(float(row[dq_col]))
                else:
                    series["dq"][j].append(0.0)

        all_series: List[Dict[str, object]] = []
        for (mode, run_label), series in grouped.items():
            if series["first_stamp"] is None:
                continue
            series_label = f"{mode}|{run_label}|{os.path.basename(path)}"
            all_series.append(
                {
                    "label": series_label,
                    "mode": mode,
                    "run_label": run_label,
                    "t": series["t"],
                    "q": series["q"],
                    "dq": series["dq"],
                }
            )

        if not all_series:
            raise RuntimeError(f"No trajectory_active==1 rows found in {path}")

        all_series.sort(key=lambda s: (mode_rank(str(s["mode"])), str(s["label"])))
        return all_series


def compute_smoothness_metrics(t: List[float], q: List[float], dq: List[float]) -> Dict[str, float]:
    if len(t) < 3:
        return {"acc_rms": math.nan, "jerk_rms": math.nan, "dq_tv": math.nan}

    acc = []
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 1e-9:
            continue
        acc.append((dq[i] - dq[i - 1]) / dt)

    jerk = []
    for i in range(1, len(acc)):
        dt = t[i + 1] - t[i]
        if dt <= 1e-9:
            continue
        jerk.append((acc[i] - acc[i - 1]) / dt)

    dq_tv = 0.0
    for i in range(1, len(dq)):
        dq_tv += abs(dq[i] - dq[i - 1])

    acc_rms = math.sqrt(sum(x * x for x in acc) / len(acc)) if acc else math.nan
    jerk_rms = math.sqrt(sum(x * x for x in jerk) / len(jerk)) if jerk else math.nan

    return {"acc_rms": acc_rms, "jerk_rms": jerk_rms, "dq_tv": dq_tv}


def main() -> None:
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    value_prefix = "cmd" if args.use_command else "act"

    all_series: List[Dict[str, object]] = []
    all_joint_names = None

    for csv_path in args.csv:
        series_list = parse_csv(csv_path, value_prefix)
        for s in series_list:
            joint_names = sorted(s["t"].keys())
            if all_joint_names is None:
                all_joint_names = joint_names
            else:
                # Keep only common joints so overlays remain aligned.
                all_joint_names = sorted(set(all_joint_names).intersection(joint_names))
            all_series.append(s)

    all_series.sort(key=lambda s: (mode_rank(str(s["mode"])), str(s["label"])))

    if not all_series:
        raise RuntimeError("No valid CSV series loaded")
    if not all_joint_names:
        raise RuntimeError("No common joint columns found across input CSV files")

    for joint in all_joint_names:
        fig, ax = plt.subplots(figsize=(10, 5))
        for s in all_series:
            t = s["t"][joint]
            y = s["q"][joint]
            if not t:
                continue
            ax.plot(t, y, linewidth=1.5, label=s["label"])

        ax.set_title(f"{args.title_prefix} | q | {joint}")
        ax.set_xlabel("time from first active sample [s]")
        ax.set_ylabel("position [rad]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()

        out_q = os.path.join(args.out_dir, f"{joint}_q.png")
        fig.savefig(out_q, dpi=140)
        plt.close(fig)

        if args.plot_dq:
            fig, ax = plt.subplots(figsize=(10, 5))
            for s in all_series:
                t = s["t"][joint]
                y = s["dq"][joint]
                if not t:
                    continue
                ax.plot(t, y, linewidth=1.5, label=s["label"])

            ax.set_title(f"{args.title_prefix} | dq | {joint}")
            ax.set_xlabel("time from first active sample [s]")
            ax.set_ylabel("velocity [rad/s]")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=8)
            fig.tight_layout()

            out_dq = os.path.join(args.out_dir, f"{joint}_dq.png")
            fig.savefig(out_dq, dpi=140)
            plt.close(fig)

    metrics_by_mode: Dict[str, Dict[str, List[float]]] = {}
    for s in all_series:
        mode = str(s["mode"])
        if mode not in metrics_by_mode:
            metrics_by_mode[mode] = {
                "acc_rms": [],
                "jerk_rms": [],
                "dq_tv": [],
            }
        for joint in all_joint_names:
            t = s["t"][joint]
            q = s["q"][joint]
            dq = s["dq"][joint]
            m = compute_smoothness_metrics(t, q, dq)
            if not math.isnan(m["acc_rms"]):
                metrics_by_mode[mode]["acc_rms"].append(m["acc_rms"])
            if not math.isnan(m["jerk_rms"]):
                metrics_by_mode[mode]["jerk_rms"].append(m["jerk_rms"])
            if not math.isnan(m["dq_tv"]):
                metrics_by_mode[mode]["dq_tv"].append(m["dq_tv"])

    print("Smoothness metrics (lower is smoother):")
    for mode in sorted(metrics_by_mode.keys(), key=mode_rank):
        vals = metrics_by_mode[mode]

        def mean_or_nan(v: List[float]) -> float:
            return (sum(v) / len(v)) if v else math.nan

        print(
            f"  mode={mode}: "
            f"mean_acc_rms={mean_or_nan(vals['acc_rms']):.6f}, "
            f"mean_jerk_rms={mean_or_nan(vals['jerk_rms']):.6f}, "
            f"mean_dq_total_variation={mean_or_nan(vals['dq_tv']):.6f}"
        )

    print(f"Saved plots to: {args.out_dir}")
    print(f"Series count: {len(all_series)}")
    print(f"Joint count: {len(all_joint_names)}")


if __name__ == "__main__":
    main()
