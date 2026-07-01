#!/usr/bin/env python3
"""Aggregate UR10 obstacle-margin sweep CSV files.

The raw benchmark emits one row per random obstacle instance.  For revision
figures we need grouped rates and continuous metrics by safety margin and
obstacle count.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


def _float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if value == "":
        return math.nan
    return float(value)


def _int(row: dict[str, str], key: str) -> int:
    value = row.get(key, "")
    if value == "":
        return 0
    return int(float(value))


def _mean(values: list[float]) -> float:
    clean = [v for v in values if not math.isnan(v)]
    return statistics.fmean(clean) if clean else math.nan


def _median(values: list[float]) -> float:
    clean = [v for v in values if not math.isnan(v)]
    return statistics.median(clean) if clean else math.nan


def _minimum(values: list[float]) -> float:
    clean = [v for v in values if not math.isnan(v)]
    return min(clean) if clean else math.nan


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    args = parser.parse_args()

    groups: dict[tuple[float, int], list[dict[str, str]]] = defaultdict(list)
    with args.input_csv.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            groups[(_float(row, "d_safe"), _int(row, "obstacle_count"))].append(row)

    fieldnames = [
        "d_safe",
        "obstacle_count",
        "num_samples",
        "solver_success_rate",
        "failed_rate",
        "collision_free_rate",
        "safety_satisfied_rate",
        "mean_solve_ms",
        "median_solve_ms",
        "mean_initial_min_distance",
        "mean_final_min_distance",
        "mean_trajectory_min_distance",
        "min_trajectory_min_distance",
        "mean_safety_violation_count",
        "mean_collision_violation_count",
        "mean_placement_error_norm",
        "median_placement_error_norm",
        "mean_path_length",
        "mean_jerk_rms",
        "mean_torque_rate_rms",
    ]

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for (d_safe, obstacle_count), rows in sorted(groups.items()):
            n = len(rows)
            safety_ok = [
                _int(row, "safety_violation_count") == 0
                and _float(row, "trajectory_min_distance") >= d_safe
                for row in rows
            ]
            collision_ok = [
                _int(row, "collision_violation_count") == 0
                and _float(row, "trajectory_min_distance") >= 0.0
                for row in rows
            ]
            writer.writerow(
                {
                    "d_safe": f"{d_safe:.6g}",
                    "obstacle_count": obstacle_count,
                    "num_samples": n,
                    "solver_success_rate": _mean([_float(row, "success") for row in rows]),
                    "failed_rate": _mean([_float(row, "failed") for row in rows]),
                    "collision_free_rate": _mean([1.0 if ok else 0.0 for ok in collision_ok]),
                    "safety_satisfied_rate": _mean([1.0 if ok else 0.0 for ok in safety_ok]),
                    "mean_solve_ms": _mean([_float(row, "solve_ms") for row in rows]),
                    "median_solve_ms": _median([_float(row, "solve_ms") for row in rows]),
                    "mean_initial_min_distance": _mean(
                        [_float(row, "initial_min_distance") for row in rows]
                    ),
                    "mean_final_min_distance": _mean(
                        [_float(row, "final_min_distance") for row in rows]
                    ),
                    "mean_trajectory_min_distance": _mean(
                        [_float(row, "trajectory_min_distance") for row in rows]
                    ),
                    "min_trajectory_min_distance": _minimum(
                        [_float(row, "trajectory_min_distance") for row in rows]
                    ),
                    "mean_safety_violation_count": _mean(
                        [_float(row, "safety_violation_count") for row in rows]
                    ),
                    "mean_collision_violation_count": _mean(
                        [_float(row, "collision_violation_count") for row in rows]
                    ),
                    "mean_placement_error_norm": _mean(
                        [_float(row, "placement_error_norm") for row in rows]
                    ),
                    "median_placement_error_norm": _median(
                        [_float(row, "placement_error_norm") for row in rows]
                    ),
                    "mean_path_length": _mean([_float(row, "path_length") for row in rows]),
                    "mean_jerk_rms": _mean([_float(row, "jerk_rms") for row in rows]),
                    "mean_torque_rate_rms": _mean(
                        [_float(row, "torque_rate_rms") for row in rows]
                    ),
                }
            )

    print(f"Wrote {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
