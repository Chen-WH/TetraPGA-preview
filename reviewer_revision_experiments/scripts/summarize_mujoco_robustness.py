#!/usr/bin/env python3
"""Create stable MuJoCo robustness summary tables from one batch directory."""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path


def as_float(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def force_sort_key(level: str) -> float:
    return as_float(level.rstrip("N"))


def offset_sort_key(level: str) -> float:
    return as_float(level.rstrip("cm"))


def human_force(level: str) -> str:
    value = force_sort_key(level)
    return f"{value:g} N"


def human_offset(level: str) -> str:
    value = offset_sort_key(level)
    return f"{value:g} cm"


def classify_case(case_name: str, row: dict[str, str]) -> tuple[str, str, str]:
    if case_name.startswith("nominal_"):
        return "nominal", "0", ""
    if case_name.startswith("mass_"):
        return "mass_inertia_scale", row["plant_mass_scale"], ""
    if case_name.startswith("payload_"):
        model_state = "modeled" if row["model_payload"] == "1" else "ignored"
        return "payload_mass", row["plant_payload_mass"], model_state
    if case_name.startswith("link_com_"):
        parts = case_name.split("_")
        return "six_link_com_offset", parts[2], parts[3]
    if case_name.startswith("com_"):
        parts = case_name.split("_")
        return "payload_com_offset", parts[1], parts[2]
    if case_name.startswith("external_"):
        parts = case_name.split("_")
        direction = parts[2] if len(parts) > 2 else "x"
        return "external_force_persistent", parts[1], direction
    return "unknown", "", ""


def first_nominal(rows: list[dict[str, str]]) -> dict[str, str]:
    for row in rows:
        if row["perturbation"] == "nominal":
            return row
    raise ValueError("missing nominal row")


def table_row(
    label: str,
    row: dict[str, str],
    *,
    window: str,
    direction: str = "",
) -> dict[str, str]:
    if window == "active":
        return {
            "condition": label,
            "direction": direction,
            "rmse_rad": row.get("active_tracking_rmse", ""),
            "mean_error_rad": row.get("active_tracking_mean", ""),
            "mean_torque_ratio": row.get("active_torque_ratio_mean", ""),
            "num_cycles": row.get("active_num_cycles", ""),
        }
    if window == "post2":
        return {
            "condition": label,
            "direction": direction,
            "rmse_rad": row.get("post2_tracking_rmse", ""),
            "mean_error_rad": row.get("post2_tracking_mean", ""),
            "mean_torque_ratio": row.get("post2_torque_ratio_mean", ""),
            "num_cycles": row.get("post2_num_cycles", ""),
        }
    return {
        "condition": label,
        "direction": direction,
        "rmse_rad": row["tracking_rmse"],
        "mean_error_rad": row["tracking_mean"],
        "mean_torque_ratio": row["torque_ratio_mean"],
        "num_cycles": row["num_cycles"],
    }


def write_simple_table(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_inertial_mismatch_table(output_dir: Path, rows: list[dict[str, str]]) -> None:
    out_rows: list[dict[str, str]] = [table_row("Nominal", first_nominal(rows), window="full")]

    mass_rows = [row for row in rows if row["perturbation"] == "mass_inertia_scale"]
    for row in sorted(mass_rows, key=lambda item: as_float(item["level"])):
        out_rows.append(table_row(f"{row['level']} scaling", row, window="full"))

    com_rows = [row for row in rows if row["perturbation"] == "six_link_com_offset"]
    for level in sorted({row["level"] for row in com_rows}, key=offset_sort_key):
        candidates = [row for row in com_rows if row["level"] == level]
        worst = max(candidates, key=lambda item: as_float(item["tracking_rmse"]))
        out_rows.append(
            table_row(f"{human_offset(level)} offset", worst, window="full", direction=worst["direction"])
        )

    write_simple_table(output_dir / "mujoco_inertial_mismatch_table.csv", out_rows)


def write_external_wrench_table(output_dir: Path, rows: list[dict[str, str]]) -> None:
    out_rows: list[dict[str, str]] = [table_row("Nominal", first_nominal(rows), window="post2")]

    external_rows = [row for row in rows if row["perturbation"] == "external_force_persistent"]
    for level in sorted({row["level"] for row in external_rows}, key=force_sort_key):
        candidates = [row for row in external_rows if row["level"] == level]
        worst = max(candidates, key=lambda item: as_float(item.get("active_tracking_rmse", "")))
        out_rows.append(
            table_row(f"{human_force(level)} force", worst, window="active", direction=worst["direction"])
        )

    write_simple_table(output_dir / "mujoco_external_wrench_table.csv", out_rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    combined = args.batch_root / "combined_summary.csv"
    metrics_path = args.batch_root / "closed_loop_metrics_summary.csv"
    if not combined.exists():
        raise SystemExit(f"missing {combined}")
    if not metrics_path.exists():
        raise SystemExit(f"missing {metrics_path}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(combined, args.output_dir / "mujoco_robustness_combined_summary.csv")
    shutil.copyfile(metrics_path, args.output_dir / "mujoco_robustness_metrics_summary.csv")

    combined_rows = list(csv.DictReader(combined.open()))
    metrics = {row["case_name"]: row for row in csv.DictReader(metrics_path.open())}
    if not combined_rows:
        raise SystemExit(f"no rows in {combined}")

    rows: list[dict[str, str]] = []
    for row in combined_rows:
        case_name = row["case_name"]
        metric_row = metrics[case_name]
        perturbation, level, direction = classify_case(case_name, row)
        rows.append(
            {
                "case_name": case_name,
                "perturbation": perturbation,
                "level": level,
                "direction": direction,
                "num_cycles": row["num_cycles"],
                "tracking_rmse": row["tracking_rmse"],
                "tracking_mean": row["tracking_mean"],
                "tracking_p95": row["tracking_p95"],
                "velocity_rmse": metric_row["velocity_rmse"],
                "solve_time_mean_ms": row["solve_time_mean_ms"],
                "solve_time_p95_ms": row["solve_time_p95_ms"],
                "deadline_miss_rate": row["deadline_miss_rate"],
                "failure_rate": row["failure_rate"],
                "torque_ratio_mean": row["torque_ratio_mean"],
                "torque_ratio_p95": row["torque_ratio_p95"],
                "active_num_cycles": metric_row.get("active_num_cycles", ""),
                "active_tracking_rmse": metric_row.get("active_tracking_rmse", ""),
                "active_tracking_mean": metric_row.get("active_tracking_mean", ""),
                "active_tracking_p95": metric_row.get("active_tracking_p95", ""),
                "active_torque_ratio_mean": metric_row.get("active_torque_ratio_mean", ""),
                "active_torque_ratio_p95": metric_row.get("active_torque_ratio_p95", ""),
                "active_window_start_s": metric_row.get("active_window_start_s", ""),
                "active_window_end_s": metric_row.get("active_window_end_s", ""),
                "post2_num_cycles": metric_row.get("post2_num_cycles", ""),
                "post2_tracking_rmse": metric_row.get("post2_tracking_rmse", ""),
                "post2_tracking_mean": metric_row.get("post2_tracking_mean", ""),
                "post2_tracking_p95": metric_row.get("post2_tracking_p95", ""),
                "post2_torque_ratio_mean": metric_row.get("post2_torque_ratio_mean", ""),
                "post2_torque_ratio_p95": metric_row.get("post2_torque_ratio_p95", ""),
                "accel_rms_norm": metric_row["accel_rms_norm"],
                "jerk_rms_norm_from_dq": metric_row["jerk_rms_norm_from_dq"],
                "torque_rate_rms_norm": metric_row["torque_rate_rms_norm"],
                "plant_mass_scale": row["plant_mass_scale"],
                "plant_payload_mass": row["plant_payload_mass"],
                "controller_payload_mass": row["controller_payload_mass"],
                "model_payload": row["model_payload"],
                "plant_payload_com": row["plant_payload_com"],
                "controller_payload_com": row["controller_payload_com"],
                "external_force": row["external_force"],
                "external_force_duration_s": row["external_force_duration_s"],
            }
        )

    output = args.output_dir / "mujoco_robustness_paper_summary.csv"
    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    write_inertial_mismatch_table(args.output_dir, rows)
    write_external_wrench_table(args.output_dir, rows)

    print(args.output_dir / "mujoco_robustness_combined_summary.csv")
    print(args.output_dir / "mujoco_robustness_metrics_summary.csv")
    print(output)
    print(args.output_dir / "mujoco_inertial_mismatch_table.csv")
    print(args.output_dir / "mujoco_external_wrench_table.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
