#!/usr/bin/env python3
"""Add closed-loop MPC runtime breakdown fields to GA-OCP."""

from __future__ import annotations

from pathlib import Path


SOURCE = Path("/home/chenwh/ros2_ws/src/GA-OCP/ga_ocp_ros2/src/closed_loop_mpc_node.cpp")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Unable to find replacement anchor for {label}")
    return text.replace(old, new, 1)


def main() -> None:
    text = SOURCE.read_text(encoding="utf-8")

    text = replace_once(
        text,
        "  double solve_time_ms = 0.0;\n"
        "  std::size_t iterations = 0;\n",
        "  double solve_time_ms = 0.0;\n"
        "  double problem_build_ms = 0.0;\n"
        "  double warm_start_ms = 0.0;\n"
        "  double initial_calc_ms = 0.0;\n"
        "  double solver_setup_ms = 0.0;\n"
        "  std::size_t iterations = 0;\n",
        "solve result breakdown fields",
    )

    text = replace_once(
        text,
        "  double solve_time_ms = 0.0;\n"
        "  double cycle_time_ms = 0.0;\n",
        "  double solve_time_ms = 0.0;\n"
        "  double reference_build_ms = 0.0;\n"
        "  double problem_build_ms = 0.0;\n"
        "  double warm_start_ms = 0.0;\n"
        "  double initial_calc_ms = 0.0;\n"
        "  double solver_setup_ms = 0.0;\n"
        "  double mpc_pipeline_time_ms = 0.0;\n"
        "  double publish_command_ms = 0.0;\n"
        "  double cycle_time_ms = 0.0;\n",
        "cycle record breakdown fields",
    )

    text = replace_once(
        text,
        "    std::shared_ptr<crocoddyl::ShootingProblem> problem;\n"
        "    switch (backend_) {\n"
        "      case BackendKind::kTetraPGA:\n"
        "        problem = buildGaProblem(x0, x_refs);\n"
        "        break;\n"
        "      case BackendKind::kPinocchio:\n"
        "        problem = buildPinProblem(x0, x_refs);\n"
        "        break;\n"
        "      case BackendKind::kCasadi:\n"
        "#ifdef GA_OCP_HAS_CASADI_BENCH\n"
        "        problem = buildCasadiProblem(x0, x_refs);\n"
        "        break;\n"
        "#else\n"
        "        throw std::runtime_error(\"CasADi backend not compiled\");\n"
        "#endif\n"
        "    }\n"
        "\n"
        "    std::vector<Eigen::VectorXd> init_xs;\n"
        "    std::vector<Eigen::VectorXd> init_us;\n"
        "    initializeWarmStart(x0, x_refs, init_xs, init_us);\n"
        "\n"
        "    result.best_xs = init_xs;\n"
        "    result.best_us = init_us;\n"
        "    result.best_cost = problem->calc(init_xs, init_us);\n"
        "\n"
        "    crocoddyl::SolverBoxFDDP solver(problem);\n"
        "    solver.set_th_stop(enforce_solve_budget_ ? std::numeric_limits<double>::min() : stop_tol_);\n",
        "    std::shared_ptr<crocoddyl::ShootingProblem> problem;\n"
        "    const Clock::time_point problem_build_start = Clock::now();\n"
        "    switch (backend_) {\n"
        "      case BackendKind::kTetraPGA:\n"
        "        problem = buildGaProblem(x0, x_refs);\n"
        "        break;\n"
        "      case BackendKind::kPinocchio:\n"
        "        problem = buildPinProblem(x0, x_refs);\n"
        "        break;\n"
        "      case BackendKind::kCasadi:\n"
        "#ifdef GA_OCP_HAS_CASADI_BENCH\n"
        "        problem = buildCasadiProblem(x0, x_refs);\n"
        "        break;\n"
        "#else\n"
        "        throw std::runtime_error(\"CasADi backend not compiled\");\n"
        "#endif\n"
        "    }\n"
        "    result.problem_build_ms = DurationSeconds(Clock::now() - problem_build_start).count() * 1e3;\n"
        "\n"
        "    std::vector<Eigen::VectorXd> init_xs;\n"
        "    std::vector<Eigen::VectorXd> init_us;\n"
        "    const Clock::time_point warm_start_start = Clock::now();\n"
        "    initializeWarmStart(x0, x_refs, init_xs, init_us);\n"
        "    result.warm_start_ms = DurationSeconds(Clock::now() - warm_start_start).count() * 1e3;\n"
        "\n"
        "    result.best_xs = init_xs;\n"
        "    result.best_us = init_us;\n"
        "    const Clock::time_point initial_calc_start = Clock::now();\n"
        "    result.best_cost = problem->calc(init_xs, init_us);\n"
        "    result.initial_calc_ms = DurationSeconds(Clock::now() - initial_calc_start).count() * 1e3;\n"
        "\n"
        "    const Clock::time_point solver_setup_start = Clock::now();\n"
        "    crocoddyl::SolverBoxFDDP solver(problem);\n"
        "    solver.set_th_stop(enforce_solve_budget_ ? std::numeric_limits<double>::min() : stop_tol_);\n"
        "    result.solver_setup_ms = DurationSeconds(Clock::now() - solver_setup_start).count() * 1e3;\n",
        "solve cycle breakdown timing",
    )

    text = replace_once(
        text,
        "    const Eigen::VectorXd x_ref_now = referenceStateAt(t);\n"
        "    const std::vector<Eigen::VectorXd> x_refs = buildReferenceTrajectory(t);\n"
        "    SolveCycleResult solve = solveCycle(x0, x_refs);\n",
        "    const Clock::time_point reference_start = Clock::now();\n"
        "    const Eigen::VectorXd x_ref_now = referenceStateAt(t);\n"
        "    const std::vector<Eigen::VectorXd> x_refs = buildReferenceTrajectory(t);\n"
        "    const double reference_build_ms = DurationSeconds(Clock::now() - reference_start).count() * 1e3;\n"
        "    const Clock::time_point mpc_pipeline_start = Clock::now();\n"
        "    SolveCycleResult solve = solveCycle(x0, x_refs);\n"
        "    const double mpc_pipeline_time_ms =\n"
        "        DurationSeconds(Clock::now() - mpc_pipeline_start).count() * 1e3;\n",
        "control loop reference and pipeline timing",
    )

    text = replace_once(
        text,
        "    publishCommand(q_cmd, dq_cmd, u_cmd);\n"
        "    last_best_xs_ = solve.best_xs;\n",
        "    const Clock::time_point publish_start = Clock::now();\n"
        "    publishCommand(q_cmd, dq_cmd, u_cmd);\n"
        "    const double publish_command_ms = DurationSeconds(Clock::now() - publish_start).count() * 1e3;\n"
        "    last_best_xs_ = solve.best_xs;\n",
        "publish command timing",
    )

    text = replace_once(
        text,
        "    record.solve_time_ms = solve.solve_time_ms;\n"
        "    record.cycle_time_ms = DurationSeconds(Clock::now() - cycle_start).count() * 1e3;\n",
        "    record.solve_time_ms = solve.solve_time_ms;\n"
        "    record.reference_build_ms = reference_build_ms;\n"
        "    record.problem_build_ms = solve.problem_build_ms;\n"
        "    record.warm_start_ms = solve.warm_start_ms;\n"
        "    record.initial_calc_ms = solve.initial_calc_ms;\n"
        "    record.solver_setup_ms = solve.solver_setup_ms;\n"
        "    record.mpc_pipeline_time_ms = mpc_pipeline_time_ms;\n"
        "    record.publish_command_ms = publish_command_ms;\n"
        "    record.cycle_time_ms = DurationSeconds(Clock::now() - cycle_start).count() * 1e3;\n",
        "record breakdown timing",
    )

    text = replace_once(
        text,
        "    out << \"robot,backend,t,tracking_error,velocity_error,torque_ratio,solve_time_ms,\"\n"
        "           \"cycle_time_ms,realtime_ratio,iterations,converged,failed,best_cost,final_stop,\"\n",
        "    out << \"robot,backend,t,tracking_error,velocity_error,torque_ratio,solve_time_ms,\"\n"
        "           \"reference_build_ms,problem_build_ms,warm_start_ms,initial_calc_ms,\"\n"
        "           \"solver_setup_ms,mpc_pipeline_time_ms,publish_command_ms,\"\n"
        "           \"cycle_time_ms,realtime_ratio,iterations,converged,failed,best_cost,final_stop,\"\n",
        "cycle csv breakdown header",
    )

    text = replace_once(
        text,
        "          << FormatCsvNumber(record.solve_time_ms) << ','\n"
        "          << FormatCsvNumber(record.cycle_time_ms) << ','\n",
        "          << FormatCsvNumber(record.solve_time_ms) << ','\n"
        "          << FormatCsvNumber(record.reference_build_ms) << ','\n"
        "          << FormatCsvNumber(record.problem_build_ms) << ','\n"
        "          << FormatCsvNumber(record.warm_start_ms) << ','\n"
        "          << FormatCsvNumber(record.initial_calc_ms) << ','\n"
        "          << FormatCsvNumber(record.solver_setup_ms) << ','\n"
        "          << FormatCsvNumber(record.mpc_pipeline_time_ms) << ','\n"
        "          << FormatCsvNumber(record.publish_command_ms) << ','\n"
        "          << FormatCsvNumber(record.cycle_time_ms) << ','\n",
        "cycle csv breakdown row",
    )

    text = replace_once(
        text,
        "    std::vector<double> solve_times;\n"
        "    std::vector<double> realtime_ratios;\n",
        "    std::vector<double> solve_times;\n"
        "    std::vector<double> reference_build_times;\n"
        "    std::vector<double> problem_build_times;\n"
        "    std::vector<double> warm_start_times;\n"
        "    std::vector<double> initial_calc_times;\n"
        "    std::vector<double> solver_setup_times;\n"
        "    std::vector<double> mpc_pipeline_times;\n"
        "    std::vector<double> publish_command_times;\n"
        "    std::vector<double> realtime_ratios;\n",
        "summary breakdown vectors",
    )

    text = replace_once(
        text,
        "    solve_times.reserve(cycle_records_.size());\n"
        "    realtime_ratios.reserve(cycle_records_.size());\n",
        "    solve_times.reserve(cycle_records_.size());\n"
        "    reference_build_times.reserve(cycle_records_.size());\n"
        "    problem_build_times.reserve(cycle_records_.size());\n"
        "    warm_start_times.reserve(cycle_records_.size());\n"
        "    initial_calc_times.reserve(cycle_records_.size());\n"
        "    solver_setup_times.reserve(cycle_records_.size());\n"
        "    mpc_pipeline_times.reserve(cycle_records_.size());\n"
        "    publish_command_times.reserve(cycle_records_.size());\n"
        "    realtime_ratios.reserve(cycle_records_.size());\n",
        "summary breakdown reserves",
    )

    text = replace_once(
        text,
        "      solve_times.push_back(record.solve_time_ms);\n"
        "      realtime_ratios.push_back(record.realtime_ratio);\n",
        "      solve_times.push_back(record.solve_time_ms);\n"
        "      reference_build_times.push_back(record.reference_build_ms);\n"
        "      problem_build_times.push_back(record.problem_build_ms);\n"
        "      warm_start_times.push_back(record.warm_start_ms);\n"
        "      initial_calc_times.push_back(record.initial_calc_ms);\n"
        "      solver_setup_times.push_back(record.solver_setup_ms);\n"
        "      mpc_pipeline_times.push_back(record.mpc_pipeline_time_ms);\n"
        "      publish_command_times.push_back(record.publish_command_ms);\n"
        "      realtime_ratios.push_back(record.realtime_ratio);\n",
        "summary breakdown push",
    )

    text = replace_once(
        text,
        "           \"torque_ratio_p95,torque_ratio_max,solve_time_mean_ms,solve_time_p95_ms,\"\n"
        "           \"realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n",
        "           \"torque_ratio_p95,torque_ratio_max,solve_time_mean_ms,solve_time_p95_ms,\"\n"
        "           \"reference_build_mean_ms,reference_build_p95_ms,problem_build_mean_ms,problem_build_p95_ms,\"\n"
        "           \"warm_start_mean_ms,warm_start_p95_ms,initial_calc_mean_ms,initial_calc_p95_ms,\"\n"
        "           \"solver_setup_mean_ms,solver_setup_p95_ms,mpc_pipeline_mean_ms,mpc_pipeline_p95_ms,\"\n"
        "           \"publish_command_mean_ms,publish_command_p95_ms,\"\n"
        "           \"realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n",
        "summary csv breakdown header",
    )

    text = replace_once(
        text,
        "        << FormatCsvNumber(Percentile(solve_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(realtime_ratios)) << ','\n",
        "        << FormatCsvNumber(Percentile(solve_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(reference_build_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(reference_build_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(problem_build_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(problem_build_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(warm_start_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(warm_start_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(initial_calc_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(initial_calc_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_setup_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_setup_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(mpc_pipeline_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(mpc_pipeline_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(publish_command_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(publish_command_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(realtime_ratios)) << ','\n",
        "summary csv breakdown row",
    )

    SOURCE.write_text(text, encoding="utf-8")
    print("Applied GA-OCP closed-loop runtime breakdown instrumentation.")


if __name__ == "__main__":
    main()
