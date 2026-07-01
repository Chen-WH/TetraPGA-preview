#!/usr/bin/env python3
"""Run and summarize TetraPGA vs Pinocchio operator microbenchmarks."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
import pathlib
import statistics
import subprocess
from typing import Dict, Iterable, List, Tuple


THIS_FILE = pathlib.Path(__file__).resolve()
EXPERIMENT_DIR = THIS_FILE.parent
REPO_ROOT = THIS_FILE.parents[2]


def run_command(cmd: List[str], *, cwd: pathlib.Path, env: Dict[str, str] | None = None) -> None:
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=str(cwd), env=env, check=True)


def csv_time_to_ns(value: str, unit: str) -> float:
    time_value = float(value)
    if unit == "ns":
        return time_value
    if unit == "us":
        return time_value * 1e3
    if unit == "ms":
        return time_value * 1e6
    if unit == "s":
        return time_value * 1e9
    return time_value


def benchmark_rows(csv_path: pathlib.Path) -> Iterable[dict]:
    lines = csv_path.read_text().splitlines()
    header_index = None
    for idx, line in enumerate(lines):
        if line.startswith("name,"):
            header_index = idx
            break
    if header_index is None:
        raise RuntimeError(f"Could not find Google Benchmark CSV header in {csv_path}")
    yield from csv.DictReader(lines[header_index:])


def split_aggregate_name(name: str) -> Tuple[str, str | None]:
    aggregate = None
    for suffix in ("_mean", "_median", "_stddev", "_cv"):
        if name.endswith(suffix):
            aggregate = suffix[1:]
            name = name[: -len(suffix)]
            break
    iteration_marker = "/iterations:"
    iteration_pos = name.find(iteration_marker)
    if iteration_pos >= 0:
        name = name[:iteration_pos]
    repeat_marker = "/repeats:"
    repeat_pos = name.find(repeat_marker)
    if repeat_pos >= 0:
        name = name[:repeat_pos]
    return name, aggregate


def summarize_raw_csv(raw_csv: pathlib.Path) -> List[dict]:
    per_case: Dict[Tuple[str, str, str], List[float]] = {}
    median_from_benchmark: Dict[Tuple[str, str, str], float] = {}

    for row in benchmark_rows(raw_csv):
        raw_name = row["name"]
        base_name, aggregate = split_aggregate_name(raw_name)
        parts = base_name.split("/")
        if len(parts) != 3:
            continue
        category, backend, operator_name = parts
        if backend not in ("TetraPGA", "Pinocchio"):
            continue
        key = (category, backend, operator_name)
        cpu_ns = csv_time_to_ns(row["cpu_time"], row["time_unit"])
        if aggregate == "median":
            median_from_benchmark[key] = cpu_ns
        elif aggregate is None:
            per_case.setdefault(key, []).append(cpu_ns)

    median_by_key: Dict[Tuple[str, str, str], float] = {}
    all_keys = set(per_case) | set(median_from_benchmark)
    for key in all_keys:
        if key in median_from_benchmark:
            median_by_key[key] = median_from_benchmark[key]
        else:
            median_by_key[key] = statistics.median(per_case[key])

    by_category: Dict[str, Dict[str, Tuple[str, float]]] = {}
    for (category, backend, operator_name), cpu_ns in median_by_key.items():
        by_category.setdefault(category, {})[backend] = (operator_name, cpu_ns)

    summary = []
    for category in sorted(by_category):
        pair = by_category[category]
        if "TetraPGA" not in pair or "Pinocchio" not in pair:
            continue
        ga_operator, ga_ns = pair["TetraPGA"]
        pin_operator, pin_ns = pair["Pinocchio"]
        speedup = pin_ns / ga_ns if ga_ns > 0.0 else math.nan
        faster = "TetraPGA" if speedup > 1.0 else "Pinocchio"
        summary.append(
            {
                "category": category,
                "tetrapga_operator": ga_operator,
                "pinocchio_operator": pin_operator,
                "tetrapga_cpu_ns": ga_ns,
                "pinocchio_cpu_ns": pin_ns,
                "pinocchio_over_tetrapga_speedup": speedup,
                "faster_backend": faster,
            }
        )
    return summary


def write_summary_csv(rows: List[dict], path: pathlib.Path) -> None:
    fieldnames = [
        "category",
        "tetrapga_operator",
        "pinocchio_operator",
        "tetrapga_cpu_ns",
        "pinocchio_cpu_ns",
        "pinocchio_over_tetrapga_speedup",
        "faster_backend",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            out = dict(row)
            for key in ("tetrapga_cpu_ns", "pinocchio_cpu_ns"):
                out[key] = f"{row[key]:.6f}"
            out["pinocchio_over_tetrapga_speedup"] = (
                f"{row['pinocchio_over_tetrapga_speedup']:.6f}"
            )
            writer.writerow(out)


def markdown_table(rows: List[dict]) -> str:
    lines = [
        "| category | TetraPGA op | TetraPGA ns | Pinocchio op | Pinocchio ns | speedup | faster |",
        "| --- | --- | ---: | --- | ---: | ---: | --- |",
    ]
    for row in rows:
        lines.append(
            "| {category} | `{tetrapga_operator}` | {ga:.3f} | `{pinocchio_operator}` | "
            "{pin:.3f} | {speedup:.2f}x | {faster_backend} |".format(
                category=row["category"],
                tetrapga_operator=row["tetrapga_operator"],
                ga=row["tetrapga_cpu_ns"],
                pinocchio_operator=row["pinocchio_operator"],
                pin=row["pinocchio_cpu_ns"],
                speedup=row["pinocchio_over_tetrapga_speedup"],
                faster_backend=row["faster_backend"],
            )
        )
    return "\n".join(lines)


def write_summary_md(rows: List[dict], path: pathlib.Path, raw_csv: pathlib.Path,
                     args: argparse.Namespace) -> None:
    tetrapga_wins = sum(1 for row in rows if row["faster_backend"] == "TetraPGA")
    pinocchio_wins = sum(1 for row in rows if row["faster_backend"] == "Pinocchio")

    content = f"""# Operator Microbenchmarks

Status: completed.

Configuration:

- Benchmark target: `TetraPGA_operator_bench`
- Raw CSV: `{raw_csv.name}`
- Repetitions: {args.repetitions}
- Fixed iterations per benchmark registration: 1,000,000
- Sample batch: 4,096 pre-generated rigid transforms, motors, twists, forces, points, and inertias
- CPU pin: {args.cpu if args.cpu is not None else "not pinned"}
- Random interleaving: {args.random_interleaving}
- Metric: median Google Benchmark CPU time per operator call, in ns

Win count:

- TetraPGA faster: {tetrapga_wins} / {len(rows)}
- Pinocchio faster: {pinocchio_wins} / {len(rows)}

Results:

{markdown_table(rows)}

Interpretation notes:

- `pinocchio_over_tetrapga_speedup > 1` means TetraPGA is faster for that operator pair.
- The transform rows compare direct fixed-size operator calls, not whole-model dynamics.
- `force_transform` uses the TetraPGA force-propagation kernel used in the current dynamics implementation and Pinocchio's `SE3.act(Force)`.
- `inertia_transform` compares the TetraPGA matrix expression used in the current dynamics implementation with Pinocchio's optimized `SE3.act(Inertia)` path.

Additional operator pairs worth mentioning in the reviewer response:

- Direct point transform: validates motor sandwiching for collision/contact point updates.
- Direct motion and force transforms: cover frame-change kernels used in RNEA/ABA passes.
- Commutator/cross product: covers velocity-product and Jacobian time-derivative terms.
- Motor/SE3 composition and inverse: covers kinematic chain propagation.
- Exp/log maps: cover retraction, integration, and local error coordinates used by IK/MPC.
- Inertia transform: covers rigid-body inertia rebase in dynamics.
"""
    path.write_text(content)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=pathlib.Path,
                        default=pathlib.Path("/tmp/tetrapga_operator_bench_build"))
    parser.add_argument("--result-dir", type=pathlib.Path)
    parser.add_argument("--summarize-only", type=pathlib.Path,
                        help="Reuse an existing raw Google Benchmark CSV instead of running.")
    parser.add_argument("--repetitions", type=int, default=7)
    parser.add_argument("--seed", type=str, default="0x20260629")
    parser.add_argument("--cpu", type=int, help="Run benchmark through taskset on this CPU id.")
    parser.add_argument("--random-interleaving", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()

    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = args.result_dir or EXPERIMENT_DIR / f"results_{timestamp}"
    result_dir.mkdir(parents=True, exist_ok=True)

    if args.summarize_only is None and not args.skip_build:
        run_command(
            [
                "cmake",
                "-S",
                str(REPO_ROOT),
                "-B",
                str(args.build_dir),
                "-DTETRAPGA_BUILD_TESTS=OFF",
                "-DTETRAPGA_BUILD_BENCHMARKS=ON",
            ],
            cwd=REPO_ROOT,
        )
        run_command(
            ["cmake", "--build", str(args.build_dir), "--target", "TetraPGA_operator_bench", "-j2"],
            cwd=REPO_ROOT,
        )

    if args.summarize_only is None:
        executable = args.build_dir / "TetraPGA_operator_bench"
        raw_csv = result_dir / "operator_bench_raw.csv"
        env = os.environ.copy()
        env["TETRAPGA_BENCH_SEED"] = args.seed
        benchmark_cmd = [
            str(executable),
            f"--benchmark_out={raw_csv}",
            "--benchmark_out_format=csv",
            f"--benchmark_repetitions={args.repetitions}",
        ]
        if args.random_interleaving:
            benchmark_cmd.append("--benchmark_enable_random_interleaving=true")
        if args.cpu is not None:
            benchmark_cmd = ["taskset", "-c", str(args.cpu)] + benchmark_cmd
        run_command(
            benchmark_cmd,
            cwd=REPO_ROOT,
            env=env,
        )
    else:
        raw_csv = args.summarize_only.resolve()

    rows = summarize_raw_csv(raw_csv)
    summary_csv = result_dir / "operator_bench_summary.csv"
    summary_md = result_dir / "summary.md"
    write_summary_csv(rows, summary_csv)
    write_summary_md(rows, summary_md, raw_csv, args)

    latest_summary_csv = EXPERIMENT_DIR / "operator_bench_summary.csv"
    latest_summary_md = EXPERIMENT_DIR / "summary.md"
    write_summary_csv(rows, latest_summary_csv)
    write_summary_md(rows, latest_summary_md, raw_csv, args)

    print(f"Raw CSV: {raw_csv}")
    print(f"Summary CSV: {summary_csv}")
    print(f"Summary MD: {summary_md}")


if __name__ == "__main__":
    main()
