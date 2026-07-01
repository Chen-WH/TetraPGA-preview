#!/usr/bin/env python3
"""Post-process closed-loop MuJoCo cycle CSVs into tracking/smoothness metrics."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def parse_vector(value: str) -> np.ndarray:
    value = (value or "").strip()
    if not value:
        return np.empty((0,), dtype=float)
    return np.asarray([float(item) for item in value.split()], dtype=float)


def stack_vectors(rows: list[dict[str, str]], key: str) -> np.ndarray:
    vectors = [parse_vector(row.get(key, "")) for row in rows]
    nonempty = [value for value in vectors if value.size > 0]
    if not nonempty:
        return np.empty((0, 0), dtype=float)
    width = nonempty[0].size
    return np.vstack([
        value if value.size == width else np.zeros(width, dtype=float)
        for value in vectors
    ])


def rms(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(math.sqrt(np.mean(np.square(values))))


def rms_norm(vectors: np.ndarray) -> float:
    if vectors.size == 0:
        return 0.0
    return float(math.sqrt(np.mean(np.sum(np.square(vectors), axis=1))))


def mean_energy(vectors: np.ndarray) -> float:
    if vectors.size == 0:
        return 0.0
    return float(np.mean(np.sum(np.square(vectors), axis=1)))


def percentile(values: np.ndarray, p: float) -> float:
    if values.size == 0:
        return 0.0
    return float(np.percentile(values, p))


def safe_float(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def finite_window_mask(
    t: np.ndarray,
    start_s: float,
    duration_s: float,
    force: np.ndarray,
    torque: np.ndarray,
) -> np.ndarray:
    has_wrench = force.size > 0 and float(np.linalg.norm(force)) > 0.0
    has_wrench = has_wrench or (torque.size > 0 and float(np.linalg.norm(torque)) > 0.0)
    if not has_wrench or duration_s <= 0.0:
        return np.zeros_like(t, dtype=bool)
    end_s = start_s + duration_s
    return (t >= start_s) & (t < end_s)


def scalar_window_metrics(
    tracking: np.ndarray,
    velocity: np.ndarray,
    torque_ratio: np.ndarray,
    failed: np.ndarray,
    mask: np.ndarray,
) -> dict[str, str]:
    window_tracking = tracking[mask]
    window_velocity = velocity[mask]
    window_torque_ratio = torque_ratio[mask]
    window_failed = failed[mask]
    if window_tracking.size == 0:
        return {
            "num_cycles": "0",
            "tracking_rmse": "",
            "tracking_mean": "",
            "tracking_p95": "",
            "velocity_rmse": "",
            "torque_ratio_mean": "",
            "torque_ratio_p95": "",
            "failure_rate": "",
        }
    return {
        "num_cycles": str(int(np.sum(mask))),
        "tracking_rmse": f"{rms(window_tracking):.9g}",
        "tracking_mean": f"{float(np.mean(window_tracking)):.9g}",
        "tracking_p95": f"{percentile(window_tracking, 95):.9g}",
        "velocity_rmse": f"{rms(window_velocity):.9g}",
        "torque_ratio_mean": f"{float(np.mean(window_torque_ratio)):.9g}",
        "torque_ratio_p95": f"{percentile(window_torque_ratio, 95):.9g}",
        "failure_rate": f"{float(np.mean(window_failed)):.9g}",
    }


def read_first_summary(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        return next(reader, {})


def summarize_cycles(path: Path) -> dict[str, str]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"{path} has no cycle rows")

    summary = read_first_summary(path.with_name(path.name.replace("_cycles.csv", "_summary.csv")))
    t = np.asarray([float(row["t"]) for row in rows], dtype=float)
    dt = float(np.median(np.diff(t))) if t.size > 1 else 0.0
    if dt <= 0.0 or not math.isfinite(dt):
        dt = float(summary.get("dt", "0") or 0.0)

    tracking = np.asarray([float(row["tracking_error"]) for row in rows], dtype=float)
    velocity = np.asarray([float(row["velocity_error"]) for row in rows], dtype=float)
    solve_ms = np.asarray([float(row["solve_time_ms"]) for row in rows], dtype=float)
    torque_ratio = np.asarray([float(row["torque_ratio"]) for row in rows], dtype=float)
    failed = np.asarray([float(row["failed"]) for row in rows], dtype=float)
    external_force = parse_vector(summary.get("external_force", rows[0].get("external_force", "")))
    external_torque = parse_vector(summary.get("external_torque", rows[0].get("external_torque", "")))
    external_start_s = safe_float(summary.get("external_force_start_s", rows[0].get("external_force_start_s", "0")))
    external_duration_s = safe_float(summary.get("external_force_duration_s", rows[0].get("external_force_duration_s", "0")))
    active_mask = finite_window_mask(t, external_start_s, external_duration_s, external_force, external_torque)
    active_metrics = scalar_window_metrics(tracking, velocity, torque_ratio, failed, active_mask)
    post2_mask = t >= 2.0
    post2_metrics = scalar_window_metrics(tracking, velocity, torque_ratio, failed, post2_mask)

    q = stack_vectors(rows, "q")
    dq = stack_vectors(rows, "dq")
    dq_cmd = stack_vectors(rows, "dq_cmd")
    ddq_cmd = stack_vectors(rows, "ddq_cmd")
    u_cmd = stack_vectors(rows, "u_cmd")
    effort = stack_vectors(rows, "effort")

    plant_accel = (
        np.diff(dq, axis=0) / dt
        if dt > 0.0 and dq.shape[0] > 1
        else np.empty((0, dq.shape[1] if dq.ndim == 2 else 0))
    )
    cmd_accel = ddq_cmd
    if cmd_accel.size == 0 and dt > 0.0 and dq_cmd.shape[0] > 1:
        cmd_accel = np.diff(dq_cmd, axis=0) / dt
    accel = cmd_accel if cmd_accel.size > 0 else plant_accel
    jerk = (
        np.diff(plant_accel, axis=0) / dt
        if dt > 0.0 and plant_accel.shape[0] > 1
        else np.empty((0, plant_accel.shape[1] if plant_accel.ndim == 2 else 0))
    )
    q_jerk = (
        np.diff(q, n=3, axis=0) / (dt ** 3)
        if dt > 0.0 and q.shape[0] > 3
        else np.empty((0, q.shape[1]))
    )
    torque_rate = (
        np.diff(u_cmd, axis=0) / dt
        if dt > 0.0 and u_cmd.shape[0] > 1
        else np.empty((0, u_cmd.shape[1]))
    )
    effort_rate = (
        np.diff(effort, axis=0) / dt
        if dt > 0.0 and effort.shape[0] > 1
        else np.empty((0, effort.shape[1]))
    )

    out = {
        "case_name": path.name.removesuffix("_cycles.csv"),
        "cycles_file": str(path),
        "summary_file": str(path.with_name(path.name.replace("_cycles.csv", "_summary.csv"))),
        "num_cycles": str(len(rows)),
        "dt_estimate_s": f"{dt:.9g}",
        "tracking_rmse": f"{rms(tracking):.9g}",
        "tracking_mean": f"{float(np.mean(tracking)):.9g}",
        "tracking_p95": f"{percentile(tracking, 95):.9g}",
        "velocity_rmse": f"{rms(velocity):.9g}",
        "solve_time_mean_ms": f"{float(np.mean(solve_ms)):.9g}",
        "solve_time_p95_ms": f"{percentile(solve_ms, 95):.9g}",
        "torque_ratio_mean": f"{float(np.mean(torque_ratio)):.9g}",
        "torque_ratio_p95": f"{percentile(torque_ratio, 95):.9g}",
        "failure_rate": f"{float(np.mean(failed)):.9g}",
        "active_window_start_s": f"{external_start_s:.9g}",
        "active_window_end_s": f"{external_start_s + external_duration_s:.9g}",
        "active_num_cycles": active_metrics["num_cycles"],
        "active_tracking_rmse": active_metrics["tracking_rmse"],
        "active_tracking_mean": active_metrics["tracking_mean"],
        "active_tracking_p95": active_metrics["tracking_p95"],
        "active_velocity_rmse": active_metrics["velocity_rmse"],
        "active_torque_ratio_mean": active_metrics["torque_ratio_mean"],
        "active_torque_ratio_p95": active_metrics["torque_ratio_p95"],
        "active_failure_rate": active_metrics["failure_rate"],
        "post2_window_start_s": "2",
        "post2_num_cycles": post2_metrics["num_cycles"],
        "post2_tracking_rmse": post2_metrics["tracking_rmse"],
        "post2_tracking_mean": post2_metrics["tracking_mean"],
        "post2_tracking_p95": post2_metrics["tracking_p95"],
        "post2_velocity_rmse": post2_metrics["velocity_rmse"],
        "post2_torque_ratio_mean": post2_metrics["torque_ratio_mean"],
        "post2_torque_ratio_p95": post2_metrics["torque_ratio_p95"],
        "post2_failure_rate": post2_metrics["failure_rate"],
        "accel_rms_norm": f"{rms_norm(accel):.9g}",
        "accel_energy_mean": f"{mean_energy(accel):.9g}",
        "cmd_accel_rms_norm": f"{rms_norm(cmd_accel):.9g}",
        "cmd_accel_energy_mean": f"{mean_energy(cmd_accel):.9g}",
        "plant_accel_rms_norm": f"{rms_norm(plant_accel):.9g}",
        "plant_accel_energy_mean": f"{mean_energy(plant_accel):.9g}",
        "jerk_rms_norm_from_dq": f"{rms_norm(jerk):.9g}",
        "jerk_rms_norm_from_q": f"{rms_norm(q_jerk):.9g}",
        "torque_rate_rms_norm": f"{rms_norm(torque_rate):.9g}",
        "effort_rate_rms_norm": f"{rms_norm(effort_rate):.9g}",
    }
    for key in [
        "robot",
        "backend",
        "plant_mass_scale",
        "plant_payload_mass",
        "controller_payload_mass",
        "model_payload",
        "plant_payload_com",
        "controller_payload_com",
        "payload_com_attachment",
        "external_force_body_name",
        "external_force_start_s",
        "external_force_duration_s",
        "external_force",
        "external_torque",
    ]:
        out[key] = summary.get(key, rows[0].get(key, ""))
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    args = parser.parse_args()

    cycle_paths = sorted(args.input_root.rglob("*_cycles.csv"))
    if not cycle_paths:
        raise SystemExit(f"No *_cycles.csv files found under {args.input_root}")

    rows = [summarize_cycles(path) for path in cycle_paths]
    fieldnames = list(rows[0].keys())
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(args.output_csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
