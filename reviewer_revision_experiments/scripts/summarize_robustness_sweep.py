#!/usr/bin/env python3
"""Aggregate UR10 robustness sweep raw CSV files."""

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
    try:
        return float(value)
    except ValueError:
        return math.nan


def _int(row: dict[str, str], key: str) -> int:
    value = row.get(key, "")
    if value == "":
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def _mean(values: list[float]) -> float:
    clean = [v for v in values if math.isfinite(v)]
    return statistics.fmean(clean) if clean else math.nan


def _median(values: list[float]) -> float:
    clean = [v for v in values if math.isfinite(v)]
    return statistics.median(clean) if clean else math.nan


def _p95(values: list[float]) -> float:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    rank = 0.95 * (len(clean) - 1)
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return clean[lo]
    alpha = rank - lo
    return (1.0 - alpha) * clean[lo] + alpha * clean[hi]


def _fmt(value: float) -> str:
    if not math.isfinite(value):
        return "nan"
    text = f"{value:.9f}".rstrip("0").rstrip(".")
    return text or "0"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    args = parser.parse_args()

    groups: dict[tuple[str, float], list[dict[str, str]]] = defaultdict(list)
    with args.input_csv.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            groups[(row["perturbation"], _float(row, "level"))].append(row)

    fieldnames = [
        "perturbation",
        "level",
        "num_samples",
        "solver_failed_rate",
        "solver_converged_rate",
        "rollout_finite_rate",
        "success_rate",
        "mean_terminal_rmse",
        "median_terminal_rmse",
        "p95_terminal_rmse",
        "mean_trajectory_rmse",
        "mean_planning_terminal_rmse",
        "mean_solve_ms",
        "mean_max_tau_ratio",
        "p95_max_abs_q",
        "p95_max_abs_dq",
    ]

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for (perturbation, level), rows in sorted(groups.items()):
            values = {
                "perturbation": perturbation,
                "level": _fmt(level),
                "num_samples": len(rows),
                "solver_failed_rate": _mean([_int(row, "solver_failed") for row in rows]),
                "solver_converged_rate": _mean([_int(row, "solver_converged") for row in rows]),
                "rollout_finite_rate": _mean([_int(row, "rollout_finite") for row in rows]),
                "success_rate": _mean([_int(row, "success") for row in rows]),
                "mean_terminal_rmse": _mean([_float(row, "rollout_terminal_rmse") for row in rows]),
                "median_terminal_rmse": _median([_float(row, "rollout_terminal_rmse") for row in rows]),
                "p95_terminal_rmse": _p95([_float(row, "rollout_terminal_rmse") for row in rows]),
                "mean_trajectory_rmse": _mean([_float(row, "rollout_trajectory_rmse") for row in rows]),
                "mean_planning_terminal_rmse": _mean(
                    [_float(row, "planning_terminal_rmse") for row in rows]
                ),
                "mean_solve_ms": _mean([_float(row, "solve_ms") for row in rows]),
                "mean_max_tau_ratio": _mean([_float(row, "max_tau_ratio") for row in rows]),
                "p95_max_abs_q": _p95([_float(row, "max_abs_q") for row in rows]),
                "p95_max_abs_dq": _p95([_float(row, "max_abs_dq") for row in rows]),
            }
            writer.writerow({key: _fmt(value) if isinstance(value, float) else value
                             for key, value in values.items()})

    print(f"Wrote {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
