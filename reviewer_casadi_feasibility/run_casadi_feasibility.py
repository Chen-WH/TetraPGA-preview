#!/usr/bin/env python3
"""Run isolated CasADi graph-construction feasibility sweeps.

Each DoF/case is launched as a separate process so an OOM kill or timeout does
not lose previous measurements. The script records process status and peak RSS
observed from /proc, and it refines the first failure interval to integer DoF.
"""

from __future__ import annotations

import argparse
import csv
import os
import resource
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_CASES = ("rnea_so", "ocp_fd", "ocp_id", "aba_fo", "rnea_fo")


@dataclass(frozen=True)
class ProbeRun:
    case: str
    dof: int
    status: str
    returncode: int
    elapsed_s: float
    peak_rss_mb: float
    timed_out: bool
    model_build_s: str = ""
    graph_build_s: str = ""
    problem_build_s: str = ""
    eval_s: str = ""
    probe_status: str = ""
    error: str = ""

    @property
    def ok(self) -> bool:
        if self.status == "timeout_ok":
            return True
        return self.returncode == 0 and self.status == "ok" and self.probe_status == "ok"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--probe",
        type=Path,
        default=Path(__file__).resolve().parent / "build" / "casadi_graph_probe",
        help="path to casadi_graph_probe executable",
    )
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results")
    parser.add_argument("--cases", default=",".join(DEFAULT_CASES), help="comma-separated case names")
    parser.add_argument("--max-dof", type=int, default=160)
    parser.add_argument("--initial-step", type=int, default=8)
    parser.add_argument(
        "--refine-window",
        type=int,
        default=8,
        help="after bracketing a failure, binary-search until this window then scan every integer DoF",
    )
    parser.add_argument("--branching-factor", type=int, default=2)
    parser.add_argument("--horizon", type=int, default=50)
    parser.add_argument("--eval", choices=("0", "1"), default="1")
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=900.0,
        help="per-probe wall-clock timeout; use 0 to disable and rely only on memory limits",
    )
    parser.add_argument(
        "--timeout-as-pass",
        action="store_true",
        help="treat probes that reach --timeout-s without failure as successful feasibility points",
    )
    parser.add_argument(
        "--memory-limit-gb",
        type=float,
        default=30.0,
        help="per-process virtual memory limit; use 0 to disable",
    )
    parser.add_argument(
        "--rss-kill-gb",
        type=float,
        default=0.0,
        help="kill a probe if observed resident memory exceeds this value; use 0 to disable",
    )
    parser.add_argument("--poll-s", type=float, default=0.1)
    parser.add_argument("--resume", action="store_true", help="reuse rows already present in raw CSV")
    parser.add_argument(
        "--extra-dofs",
        default="1,2,3,4,5,6,7,8,9,10,12,15,16,20,24,31,32,40,48,63,64,80,96,127,128",
        help="comma-separated DoFs always tested before refinement",
    )
    return parser.parse_args()


def parse_dofs(raw: str) -> list[int]:
    out: list[int] = []
    for item in raw.split(","):
        item = item.strip()
        if item:
            out.append(int(item))
    return sorted(set(out))


def read_peak_rss_mb(pid: int) -> float:
    status_path = Path("/proc") / str(pid) / "status"
    try:
        with status_path.open("r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("VmHWM:"):
                    parts = line.split()
                    if len(parts) >= 2:
                        return float(parts[1]) / 1024.0
    except FileNotFoundError:
        return 0.0
    return 0.0


def limit_memory(memory_limit_gb: float):
    if memory_limit_gb <= 0.0:
        return None

    limit_bytes = int(memory_limit_gb * (1024**3))

    def apply_limit() -> None:
        resource.setrlimit(resource.RLIMIT_AS, (limit_bytes, limit_bytes))

    return apply_limit


def decode_signal(returncode: int) -> str:
    if returncode >= 0:
        return f"exit_{returncode}"
    try:
        return f"signal_{signal.Signals(-returncode).name}"
    except ValueError:
        return f"signal_{-returncode}"


def parse_probe_stdout(stdout: str) -> dict[str, str]:
    lines = [line for line in stdout.splitlines() if line.strip()]
    if len(lines) < 2:
        return {}
    reader = csv.DictReader(lines[-2:])
    for row in reader:
        return dict(row)
    return {}


def run_probe(args: argparse.Namespace, case: str, dof: int) -> ProbeRun:
    cmd = [
        str(args.probe),
        f"--case={case}",
        f"--dof={dof}",
        f"--branching_factor={args.branching_factor}",
        f"--horizon={args.horizon}",
        f"--eval={args.eval}",
    ]
    start = time.monotonic()
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        preexec_fn=limit_memory(args.memory_limit_gb),
    )
    peak_rss_mb = 0.0
    timed_out = False

    while proc.poll() is None:
        peak_rss_mb = max(peak_rss_mb, read_peak_rss_mb(proc.pid))
        if args.rss_kill_gb > 0.0 and peak_rss_mb > args.rss_kill_gb * 1024.0:
            proc.kill()
            break
        if args.timeout_s > 0.0 and time.monotonic() - start > args.timeout_s:
            timed_out = True
            proc.kill()
            break
        time.sleep(args.poll_s)

    stdout, stderr = proc.communicate()
    elapsed_s = time.monotonic() - start
    peak_rss_mb = max(peak_rss_mb, read_peak_rss_mb(proc.pid))
    returncode = proc.returncode if proc.returncode is not None else -999
    row = parse_probe_stdout(stdout)
    probe_status = row.get("status", "")
    status = "ok" if returncode == 0 and probe_status == "ok" else decode_signal(returncode)
    error = row.get("error", "")
    if stderr.strip():
        error = (error + " | " if error else "") + stderr.strip().splitlines()[-1]
    if timed_out:
        if args.timeout_as_pass:
            status = "timeout_ok"
            error = (error + " | " if error else "") + (
                f"timeout after {args.timeout_s}s; treated as pass"
            )
        else:
            status = "timeout"
            error = (error + " | " if error else "") + f"timeout after {args.timeout_s}s"
    elif args.rss_kill_gb > 0.0 and peak_rss_mb > args.rss_kill_gb * 1024.0:
        status = "rss_limit"
        error = (error + " | " if error else "") + f"RSS exceeded {args.rss_kill_gb}GB"

    return ProbeRun(
        case=case,
        dof=dof,
        status=status,
        returncode=returncode,
        elapsed_s=elapsed_s,
        peak_rss_mb=peak_rss_mb,
        timed_out=timed_out,
        model_build_s=row.get("model_build_s", ""),
        graph_build_s=row.get("graph_build_s", ""),
        problem_build_s=row.get("problem_build_s", ""),
        eval_s=row.get("eval_s", ""),
        probe_status=probe_status,
        error=error,
    )


def raw_header() -> list[str]:
    return [
        "case",
        "dof",
        "status",
        "returncode",
        "elapsed_s",
        "peak_rss_mb",
        "timed_out",
        "model_build_s",
        "graph_build_s",
        "problem_build_s",
        "eval_s",
        "probe_status",
        "error",
    ]


def append_raw(path: Path, run: ProbeRun) -> None:
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=raw_header())
        if not exists:
            writer.writeheader()
        writer.writerow(
            {
                "case": run.case,
                "dof": run.dof,
                "status": run.status,
                "returncode": run.returncode,
                "elapsed_s": f"{run.elapsed_s:.9f}",
                "peak_rss_mb": f"{run.peak_rss_mb:.3f}",
                "timed_out": int(run.timed_out),
                "model_build_s": run.model_build_s,
                "graph_build_s": run.graph_build_s,
                "problem_build_s": run.problem_build_s,
                "eval_s": run.eval_s,
                "probe_status": run.probe_status,
                "error": run.error,
            }
        )


def load_existing(path: Path) -> dict[tuple[str, int], ProbeRun]:
    if not path.exists():
        return {}
    out: dict[tuple[str, int], ProbeRun] = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            run = ProbeRun(
                case=row["case"],
                dof=int(row["dof"]),
                status=row["status"],
                returncode=int(row["returncode"]),
                elapsed_s=float(row["elapsed_s"]),
                peak_rss_mb=float(row["peak_rss_mb"]),
                timed_out=bool(int(row["timed_out"])),
                model_build_s=row.get("model_build_s", ""),
                graph_build_s=row.get("graph_build_s", ""),
                problem_build_s=row.get("problem_build_s", ""),
                eval_s=row.get("eval_s", ""),
                probe_status=row.get("probe_status", ""),
                error=row.get("error", ""),
            )
            out[(run.case, run.dof)] = run
    return out


def candidate_dofs(args: argparse.Namespace) -> list[int]:
    out = set(parse_dofs(args.extra_dofs))
    out.update(range(args.initial_step, args.max_dof + 1, args.initial_step))
    out = {d for d in out if 1 <= d <= args.max_dof}
    return sorted(out)


def summarize_case(case: str, runs: Iterable[ProbeRun]) -> dict[str, object]:
    ordered = sorted((run for run in runs if run.case == case), key=lambda r: r.dof)
    successes = [run for run in ordered if run.ok]
    failures = [run for run in ordered if not run.ok]
    last_success = max((run.dof for run in successes), default=None)
    first_failure = min((run.dof for run in failures), default=None)
    first_failure_run = next((run for run in failures if run.dof == first_failure), None)
    last_success_run = next((run for run in successes if run.dof == last_success), None)
    return {
        "case": case,
        "tested_dofs": " ".join(str(run.dof) for run in ordered),
        "last_success_dof": "" if last_success is None else last_success,
        "first_failure_dof": "" if first_failure is None else first_failure,
        "first_failure_status": "" if first_failure_run is None else first_failure_run.status,
        "last_success_peak_rss_mb": "" if last_success_run is None else f"{last_success_run.peak_rss_mb:.3f}",
        "last_success_graph_build_s": "" if last_success_run is None else last_success_run.graph_build_s,
    }


def write_summary(path: Path, cases: list[str], runs: Iterable[ProbeRun]) -> None:
    rows = [summarize_case(case, runs) for case in cases]
    fieldnames = [
        "case",
        "last_success_dof",
        "first_failure_dof",
        "first_failure_status",
        "last_success_peak_rss_mb",
        "last_success_graph_build_s",
        "tested_dofs",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    if not args.probe.exists():
        print(f"probe executable not found: {args.probe}", file=sys.stderr)
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_csv = args.output_dir / "casadi_graph_feasibility_raw.csv"
    summary_csv = args.output_dir / "casadi_graph_feasibility_summary.csv"
    cases = [item.strip() for item in args.cases.split(",") if item.strip()]
    runs = load_existing(raw_csv) if args.resume else {}

    def ensure_run(case: str, dof: int) -> ProbeRun:
        key = (case, dof)
        if key in runs:
            return runs[key]
        print(f"[run] case={case} dof={dof}", flush=True)
        run = run_probe(args, case, dof)
        print(
            f"[done] case={case} dof={dof} status={run.status} "
            f"rss={run.peak_rss_mb:.1f}MB elapsed={run.elapsed_s:.2f}s",
            flush=True,
        )
        runs[key] = run
        append_raw(raw_csv, run)
        return run

    base_dofs = candidate_dofs(args)
    for case in cases:
        last_success: int | None = None
        first_failure: int | None = None

        for dof in base_dofs:
            run = ensure_run(case, dof)
            if run.ok:
                last_success = dof if last_success is None else max(last_success, dof)
            else:
                first_failure = dof
                break

        if last_success is not None and first_failure is not None:
            while first_failure - last_success > args.refine_window:
                mid = (last_success + first_failure) // 2
                run = ensure_run(case, mid)
                if run.ok:
                    last_success = mid
                else:
                    first_failure = mid

            for dof in range(last_success + 1, first_failure):
                run = ensure_run(case, dof)
                if run.ok:
                    last_success = max(last_success, dof)
                else:
                    first_failure = min(first_failure, dof)
                    break

    write_summary(summary_csv, cases, runs.values())
    print(f"Wrote {raw_csv}")
    print(f"Wrote {summary_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
