#!/usr/bin/env python3
"""Review reviewer-revision experiment outputs for internal consistency."""

from __future__ import annotations

import csv
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "summary" / "review_report.md"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def as_float(value: str) -> float:
    try:
        return float(value)
    except ValueError:
        return math.nan


def status(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def add_check(lines: list[str], name: str, ok: bool, detail: str) -> None:
    lines.append(f"| {name} | {status(ok)} | {detail} |")


def main() -> int:
    lines: list[str] = [
        "# Reviewer Experiment Output Review",
        "",
        "This report checks CSV shape, failure flags, and the Stanford TidyBot",
        "fixed-budget diagnosis after the paper-scale rerun.",
        "",
        "| Check | Status | Detail |",
        "| --- | --- | --- |",
    ]
    all_ok = True

    casadi = read_csv(ROOT / "00_casadi_feasibility" / "casadi_oom20_summary.csv")
    ok = len(casadi) == 3
    all_ok &= ok
    add_check(lines, "CasADi feasibility rows", ok, f"{len(casadi)} rows")
    expected_failures = {"aba_fo": "408", "rnea_fo": "414"}
    for row in casadi:
      case = row["case"]
      if case in expected_failures:
        ok = row["first_failure_dof"] == expected_failures[case]
        all_ok &= ok
        add_check(
            lines,
            f"CasADi {case} RSS boundary",
            ok,
            f"first_failure_dof={row['first_failure_dof']}",
        )

    casadi_memonly = read_csv(
        ROOT / "00_casadi_feasibility" / "casadi_oom20_memonly_summary.csv"
    )
    expected_memonly_failures = {"aba_fo": "411", "rnea_fo": "418"}
    ok = len(casadi_memonly) == 3
    all_ok &= ok
    add_check(lines, "CasADi mem-only rows", ok, f"{len(casadi_memonly)} rows")
    for row in casadi_memonly:
      case = row["case"]
      if case in expected_memonly_failures:
        ok = (
            row["first_failure_dof"] == expected_memonly_failures[case]
            and row["first_failure_status"] == "exit_2"
        )
      else:
        ok = (
            case == "rnea_so"
            and row["last_success_dof"] == "72"
            and row["first_failure_dof"] == ""
            and row["first_failure_status"] == ""
        )
      all_ok &= ok
      add_check(
          lines,
          f"CasADi {case} mem-only boundary",
          ok,
          f"last_success={row['last_success_dof']}, first_failure={row['first_failure_dof']}",
      )

    rnea_so_10min = read_csv(
        ROOT / "00_casadi_feasibility" / "casadi_oom20_10min_rnea_so_summary.csv"
    )
    ok = (
        len(rnea_so_10min) == 1
        and rnea_so_10min[0]["case"] == "rnea_so"
        and rnea_so_10min[0]["last_success_dof"] == "402"
        and rnea_so_10min[0]["first_failure_dof"] == "403"
        and rnea_so_10min[0]["first_failure_status"] == "exit_2"
    )
    all_ok &= ok
    detail = "missing"
    if rnea_so_10min:
        row = rnea_so_10min[0]
        detail = (
            f"last_success={row['last_success_dof']}, "
            f"first_failure={row['first_failure_dof']}"
        )
    add_check(lines, "CasADi rnea_so 10min memory boundary", ok, detail)

    dynamics = read_csv(ROOT / "01_model_dynamics_three_models" / "model_dynamics_summary.csv")
    ok = len(dynamics) == 3 and all(row["status"] == "pass" for row in dynamics)
    all_ok &= ok
    add_check(lines, "Three-model dynamics", ok, f"{len(dynamics)} pass rows")

    fixed = read_csv(
        ROOT
        / "02_ocp_fixed_budget"
        / "paper_scale_reviewed"
        / "fixed_budget_paper_reviewed_summary.csv"
    )
    ok = len(fixed) == 72
    all_ok &= ok
    add_check(lines, "Fixed-budget reviewed shape", ok, f"{len(fixed)} rows")
    final_budget = [row for row in fixed if row["scenario"] == "stanford_tidybot" and row["budget_ms"] == "200"]
    ok = len(final_budget) == 3 and all(abs(as_float(row["success_rate"]) - 1.0) < 1e-12 for row in final_budget)
    all_ok &= ok
    add_check(
        lines,
        "Stanford TidyBot reviewed final success",
        ok,
        ", ".join(f"{row['method']}={row['success_rate']}" for row in final_budget),
    )

    original_tidy = read_csv(
        ROOT
        / "02_ocp_fixed_budget"
        / "paper_scale"
        / "stanford_tidybot"
        / "stanford_tidybot_samples.csv"
    )
    reviewed_tidy = read_csv(
        ROOT
        / "02_ocp_fixed_budget"
        / "paper_scale_reviewed"
        / "stanford_tidybot"
        / "stanford_tidybot_samples.csv"
    )
    original_caps = sorted({row["max_iterations"] for row in original_tidy})
    reviewed_caps = sorted({row["max_iterations"] for row in reviewed_tidy})
    ok = original_caps == ["25"] and reviewed_caps == ["100"]
    all_ok &= ok
    add_check(
        lines,
        "TidyBot iteration-cap diagnosis",
        ok,
        f"original={original_caps}, reviewed={reviewed_caps}",
    )

    obstacle_raw = read_csv(
        ROOT
        / "03_ur10_obstacle_sensitivity"
        / "paper_scale"
        / "ur10_margin_sweep_paper.csv"
    )
    obstacle_summary = read_csv(
        ROOT
        / "03_ur10_obstacle_sensitivity"
        / "paper_scale"
        / "ur10_margin_sweep_paper_summary.csv"
    )
    ok = len(obstacle_raw) == 400 and len(obstacle_summary) == 20
    all_ok &= ok
    add_check(
        lines,
        "UR10 obstacle paper-scale shape",
        ok,
        f"raw={len(obstacle_raw)}, grouped={len(obstacle_summary)}",
    )
    ok = all(abs(as_float(row["failed_rate"])) < 1e-12 for row in obstacle_summary)
    all_ok &= ok
    add_check(lines, "UR10 obstacle failed_rate", ok, "all grouped failed_rate values are 0")
    rate_columns = ["solver_success_rate", "failed_rate", "collision_free_rate", "safety_satisfied_rate"]
    ok = all(
        0.0 <= as_float(row[column]) <= 1.0
        for row in obstacle_summary
        for column in rate_columns
    )
    all_ok &= ok
    add_check(lines, "UR10 obstacle rate bounds", ok, "all rates are in [0, 1]")

    robustness_raw = read_csv(
        ROOT
        / "05_robustness_expanded_sweep"
        / "paper_scale"
        / "ur10_robustness_sweep.csv"
    )
    robustness_summary = read_csv(
        ROOT
        / "05_robustness_expanded_sweep"
        / "paper_scale"
        / "ur10_robustness_sweep_summary.csv"
    )
    ok = len(robustness_raw) == 240 and len(robustness_summary) == 12
    all_ok &= ok
    add_check(
        lines,
        "UR10 robustness paper-scale shape",
        ok,
        f"raw={len(robustness_raw)}, grouped={len(robustness_summary)}",
    )
    ok = all(as_float(row["solver_failed_rate"]) == 0.0 for row in robustness_summary)
    all_ok &= ok
    add_check(lines, "UR10 robustness solver_failed_rate", ok, "all grouped values are 0")
    ok = all(as_float(row["rollout_finite_rate"]) == 1.0 for row in robustness_summary)
    all_ok &= ok
    add_check(lines, "UR10 robustness finite rollout", ok, "all grouped values are 1")
    robustness_rate_columns = [
        "solver_failed_rate",
        "solver_converged_rate",
        "rollout_finite_rate",
        "success_rate",
    ]
    ok = all(
        0.0 <= as_float(row[column]) <= 1.0
        for row in robustness_summary
        for column in robustness_rate_columns
    )
    all_ok &= ok
    add_check(lines, "UR10 robustness rate bounds", ok, "all rates are in [0, 1]")
    nominal_rows = [
        row for row in robustness_summary
        if row["perturbation"] == "nominal" and row["level"] == "0"
    ]
    ok = (
        len(nominal_rows) == 1
        and as_float(nominal_rows[0]["success_rate"]) == 1.0
        and as_float(nominal_rows[0]["mean_terminal_rmse"]) < 0.1
    )
    all_ok &= ok
    detail = "missing"
    if nominal_rows:
        detail = (
            f"success={nominal_rows[0]['success_rate']}, "
            f"mean_rmse={nominal_rows[0]['mean_terminal_rmse']}"
        )
    add_check(lines, "UR10 robustness nominal baseline", ok, detail)

    closed_loop_metrics = read_csv(
        ROOT
        / "04_closed_loop_mpc_metrics"
        / "mujoco_closed_loop_metrics_summary.csv"
    )
    ok = len(closed_loop_metrics) == 9
    all_ok &= ok
    add_check(lines, "MuJoCo reference closed-loop shape", ok, f"{len(closed_loop_metrics)} rows")
    expected_reference_cases = {
        f"reference_{robot}_{backend}"
        for robot in ["ur", "leap", "tidybot"]
        for backend in ["tetrapga", "pinocchio", "casadi"]
    }
    actual_reference_cases = {row["case_name"] for row in closed_loop_metrics}
    ok = expected_reference_cases == actual_reference_cases
    all_ok &= ok
    add_check(
        lines,
        "MuJoCo reference case coverage",
        ok,
        ", ".join(sorted(actual_reference_cases)),
    )
    expected_robots = {"ur", "leap_left", "stanford_tidybot"}
    actual_robots = {row["robot"] for row in closed_loop_metrics}
    ok = expected_robots == actual_robots
    all_ok &= ok
    add_check(lines, "MuJoCo reference robot coverage", ok, ", ".join(sorted(actual_robots)))
    expected_backends = {"tetrapga", "pinocchio", "casadi"}
    actual_backends = {row["backend"] for row in closed_loop_metrics}
    ok = expected_backends == actual_backends
    all_ok &= ok
    add_check(lines, "MuJoCo reference backend coverage", ok, ", ".join(sorted(actual_backends)))
    metric_columns = [
        "tracking_rmse",
        "velocity_rmse",
        "accel_rms_norm",
        "accel_energy_mean",
        "cmd_accel_rms_norm",
        "cmd_accel_energy_mean",
        "plant_accel_rms_norm",
        "plant_accel_energy_mean",
        "jerk_rms_norm_from_dq",
        "torque_rate_rms_norm",
        "command_torque_ratio_max",
    ]
    ok = all(
        math.isfinite(as_float(row[column]))
        for row in closed_loop_metrics
        for column in metric_columns
    )
    all_ok &= ok
    add_check(lines, "MuJoCo reference metric finiteness", ok, "tracking/smoothness columns finite")
    ok = all(as_float(row["command_torque_ratio_max"]) <= 1.0 + 1e-7 for row in closed_loop_metrics)
    all_ok &= ok
    add_check(lines, "MuJoCo reference command torque hard bounds", ok, "max command torque ratio <= 1")
    ok = all(as_float(row["failure_rate"]) == 0.0 for row in closed_loop_metrics)
    all_ok &= ok
    add_check(lines, "MuJoCo reference failure_rate", ok, "all rows have failure_rate=0")

    mujoco_robustness = read_csv(
        ROOT
        / "06_mujoco_robustness_expanded_sweep"
        / "paper_scale"
        / "mujoco_robustness_paper_summary.csv"
    )
    ok = len(mujoco_robustness) == 22
    all_ok &= ok
    add_check(lines, "MuJoCo robustness paper-scale shape", ok, f"{len(mujoco_robustness)} rows")
    expected_perturbations = {
        "nominal",
        "mass_inertia_scale",
        "payload_mass",
        "payload_com_offset",
        "external_force_impulse",
    }
    perturbations = {row["perturbation"] for row in mujoco_robustness}
    ok = expected_perturbations <= perturbations
    all_ok &= ok
    add_check(lines, "MuJoCo robustness perturbation coverage", ok, ", ".join(sorted(perturbations)))
    ok = all(as_float(row["failure_rate"]) == 0.0 for row in mujoco_robustness)
    all_ok &= ok
    add_check(lines, "MuJoCo robustness failure_rate", ok, "all cases have failure_rate=0")
    ok = all(
        math.isfinite(as_float(row[column]))
        for row in mujoco_robustness
        for column in ["tracking_rmse", "solve_time_mean_ms", "jerk_rms_norm_from_dq"]
    )
    all_ok &= ok
    add_check(lines, "MuJoCo robustness finite core metrics", ok, "tracking/runtime/jerk finite")

    runtime_breakdown = read_csv(
        ROOT
        / "07_runtime_breakdown"
        / "paper_scale"
        / "runtime_breakdown_summary.csv"
    )
    expected_runtime_robots = {"ur10", "leap_hand", "unitree_g1"}
    expected_runtime_backends = {"TetraPGA", "Pinocchio", "CasADi"}
    actual_runtime_robots = {row.get("robot", "") for row in runtime_breakdown}
    actual_runtime_backends = {row.get("backend", "") for row in runtime_breakdown}
    ok = (
        len(runtime_breakdown) == 9
        and expected_runtime_robots == actual_runtime_robots
        and expected_runtime_backends == actual_runtime_backends
    )
    all_ok &= ok
    add_check(
        lines,
        "Runtime breakdown case coverage",
        ok,
        f"robots={','.join(sorted(actual_runtime_robots))}; "
        f"backends={','.join(sorted(actual_runtime_backends))}",
    )
    required_runtime_columns = [
        "solve_total_mean_ms",
        "solve_total_p95_ms",
        "dam_calc_mean_ms",
        "dam_calcdiff_mean_ms",
        "dam_total_mean_ms",
        "cost_total_mean_ms",
        "cost_collision_calc_mean_ms",
        "cost_collision_calcdiff_mean_ms",
        "cost_collision_total_mean_ms",
        "non_cost_model_mean_ms",
        "solver_overhead_mean_ms",
        "stack_total_mean_ms",
    ]
    columns_present = all(
        column in row
        for row in runtime_breakdown
        for column in required_runtime_columns
    )
    ok = columns_present and all(
        math.isfinite(as_float(row[column]))
        for row in runtime_breakdown
        for column in required_runtime_columns
    )
    all_ok &= ok
    add_check(lines, "Runtime solver-internal finite timings", ok, "all required solver timing means finite")
    def runtime_closure_ok(row: dict[str, str]) -> bool:
        solve = as_float(row["solve_total_mean_ms"])
        dam = as_float(row["dam_total_mean_ms"])
        cost = as_float(row["cost_total_mean_ms"])
        non_cost = as_float(row["non_cost_model_mean_ms"])
        overhead = as_float(row["solver_overhead_mean_ms"])
        stack = as_float(row["stack_total_mean_ms"])
        tol = max(1e-6, 1e-6 * solve)
        return (
            solve > 0.0
            and dam > 0.0
            and cost >= 0.0
            and non_cost >= 0.0
            and overhead >= 0.0
            and abs(dam - cost - non_cost) <= tol
            and abs(solve - dam - overhead) <= tol
            and abs(stack - solve) <= tol
        )
    ok = all(
        runtime_closure_ok(row)
        for row in runtime_breakdown
    )
    all_ok &= ok
    add_check(
        lines,
        "Runtime solver total closure",
        ok,
        "solve_total = dam_total + overhead; dam_total = cost_total + non_cost_model",
    )
    ok = (
        all(as_float(row["solve_total_p95_ms"]) >= as_float(row["solve_total_mean_ms"]) > 0.0
            for row in runtime_breakdown)
    )
    all_ok &= ok
    add_check(lines, "Runtime solve-time p95 ordering", ok, "p95 >= mean > 0")
    ok = (
        all(as_float(row["cost_collision_total_mean_ms"]) >= 0.0 for row in runtime_breakdown)
        and all(
            abs(
                as_float(row["cost_collision_total_mean_ms"])
                - as_float(row["cost_collision_calc_mean_ms"])
                - as_float(row["cost_collision_calcdiff_mean_ms"])
            ) <= max(1e-6, 1e-6 * max(1.0, as_float(row["cost_collision_total_mean_ms"])))
            for row in runtime_breakdown
        )
    )
    all_ok &= ok
    add_check(lines, "Runtime collision timing closure", ok, "collision_total = calc + calcdiff")
    ok = all(as_float(row["success_rate"]) == 1.0 for row in runtime_breakdown)
    all_ok &= ok
    add_check(lines, "Runtime breakdown success_rate", ok, "all rows have success_rate=1")

    lines.extend(
        [
            "",
            "## Stanford TidyBot Diagnosis",
            "",
            "- The original paper-scale fixed-budget run used `max_iterations=25`.",
            "- With the same seed and samples, `max_iterations=100` reaches 100% success",
            "  at 200 ms for TetraPGA, Pinocchio, and CasADi.",
            "- The non-saturated TidyBot curve was therefore an iteration-cap artifact,",
            "  not evidence of a model-loading or floating-base dynamics failure.",
            "",
            "## Preferred Fixed-Budget Data",
            "",
            "Use `02_ocp_fixed_budget/paper_scale_reviewed/fixed_budget_paper_reviewed_summary.csv`",
            "for paper tables and plots. Keep the older `paper_scale/` directory as",
            "traceability for the initial rerun.",
            "",
            f"Overall status: {status(all_ok)}",
            "",
        ]
    )

    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text("\n".join(lines))
    print(REPORT)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
