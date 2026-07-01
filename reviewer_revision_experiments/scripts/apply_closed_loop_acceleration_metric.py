#!/usr/bin/env python3
"""Patch GA-OCP closed-loop reference tests for acceleration smoothing metrics."""

from __future__ import annotations

from pathlib import Path


GA_OCP = Path("/home/chenwh/ros2_ws/src/GA-OCP")
NODE = GA_OCP / "ga_ocp_ros2/src/closed_loop_mpc_node.cpp"
UR_LAUNCH = GA_OCP / "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_ur.launch.py"
LEAP_LAUNCH = GA_OCP / "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_leap.launch.py"
TIDY_LAUNCH = GA_OCP / "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_tidybot.launch.py"


def replace_checked(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"missing patch anchor: {label}")
    return text.replace(old, new, 1)


def patch_node() -> None:
    text = NODE.read_text()

    text = replace_checked(
        text,
        "#include <crocoddyl/core/residuals/control.hpp>\n",
        "#include <crocoddyl/core/residuals/control.hpp>\n"
        "#include <crocoddyl/core/residuals/joint-acceleration.hpp>\n",
        "joint acceleration include",
    )

    text = replace_checked(
        text,
        "  double torque_ratio = 0.0;\n"
        "  double solve_time_ms = 0.0;\n",
        "  double torque_ratio = 0.0;\n"
        "  double command_torque_ratio = 0.0;\n"
        "  double solve_time_ms = 0.0;\n",
        "cycle record command torque ratio",
    )

    text = replace_checked(
        text,
        "  std::string dq_cmd;\n"
        "  std::string u_cmd;\n",
        "  std::string dq_cmd;\n"
        "  std::string ddq_cmd;\n"
        "  std::string u_cmd;\n",
        "cycle record ddq_cmd",
    )

    text = replace_checked(
        text,
        "    control_weight_ = this->declare_parameter<double>(\"control_weight\", 1e-3);\n"
        "    velocity_limit_weight_ = this->declare_parameter<double>(\"velocity_limit_weight\", 20.0);\n",
        "    control_weight_ = this->declare_parameter<double>(\"control_weight\", 1e-3);\n"
        "    acceleration_weight_ = this->declare_parameter<double>(\"acceleration_weight\", 0.0);\n"
        "    velocity_limit_weight_ = this->declare_parameter<double>(\"velocity_limit_weight\", 20.0);\n",
        "acceleration parameter",
    )

    text = replace_checked(
        text,
        "                \"dt=%.3f control_rate=%.1fHz enforce_budget=%s output=%s \"\n",
        "                \"dt=%.3f control_rate=%.1fHz enforce_budget=%s accel_weight=%.3g output=%s \"\n",
        "log format acceleration weight",
    )
    text = replace_checked(
        text,
        "                horizon_, dt_, control_rate_hz_, enforce_solve_budget_ ? \"true\" : \"false\",\n"
        "                output_prefix_.string().c_str(),\n",
        "                horizon_, dt_, control_rate_hz_, enforce_solve_budget_ ? \"true\" : \"false\",\n"
        "                acceleration_weight_, output_prefix_.string().c_str(),\n",
        "log argument acceleration weight",
    )

    text = replace_checked(
        text,
        "    const Eigen::VectorXd x_zero = Eigen::VectorXd::Zero(2 * ga_model_.dof_a);\n"
        "    for (int t = 0; t < horizon_; ++t) {\n",
        "    const Eigen::VectorXd x_zero = Eigen::VectorXd::Zero(2 * ga_model_.dof_a);\n"
        "    const Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(ga_model_.dof_a);\n"
        "    for (int t = 0; t < horizon_; ++t) {\n",
        "ga a_zero",
    )
    text = replace_checked(
        text,
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n",
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      if (acceleration_weight_ > 0.0) {\n"
        "        auto acceleration_residual =\n"
        "            std::make_shared<ResidualModelTetraPGAJointAcceleration<double>>(\n"
        "                state, ga_model_, a_zero);\n"
        "        std::shared_ptr<crocoddyl::CostModelAbstract> acceleration_cost = ProfileCost(\n"
        "            std::make_shared<crocoddyl::CostModelResidual>(state, acceleration_residual),\n"
        "            ga_ocp::RuntimeCostCategory::kOther);\n"
        "        running_cost->addCost(\"acceleration_reg\", acceleration_cost, acceleration_weight_);\n"
        "      }\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n",
        "ga acceleration cost",
    )

    text = replace_checked(
        text,
        "    const Eigen::VectorXd x_zero = Eigen::VectorXd::Zero(2 * ga_model_.dof_a);\n"
        "    for (int t = 0; t < horizon_; ++t) {\n",
        "    const Eigen::VectorXd x_zero = Eigen::VectorXd::Zero(2 * ga_model_.dof_a);\n"
        "    const Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(ga_model_.dof_a);\n"
        "    for (int t = 0; t < horizon_; ++t) {\n",
        "pin a_zero",
    )
    text = replace_checked(
        text,
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n\n"
        "      auto diff_model = std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(\n",
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      if (acceleration_weight_ > 0.0) {\n"
        "        auto acceleration_residual =\n"
        "            std::make_shared<crocoddyl::ResidualModelJointAcceleration>(\n"
        "                state, a_zero, ga_model_.dof_a);\n"
        "        auto acceleration_cost =\n"
        "            std::make_shared<crocoddyl::CostModelResidual>(state, acceleration_residual);\n"
        "        running_cost->addCost(\"acceleration_reg\", acceleration_cost, acceleration_weight_);\n"
        "      }\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n\n"
        "      auto diff_model = std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(\n",
        "pin acceleration cost",
    )

    text = replace_checked(
        text,
        "    const Eigen::VectorXd x_zero = Eigen::VectorXd::Zero(2 * ga_model_.dof_a);\n"
        "    for (int t = 0; t < horizon_; ++t) {\n",
        "    const Eigen::VectorXd x_zero = Eigen::VectorXd::Zero(2 * ga_model_.dof_a);\n"
        "    const Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(ga_model_.dof_a);\n"
        "    for (int t = 0; t < horizon_; ++t) {\n",
        "casadi a_zero",
    )
    text = replace_checked(
        text,
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n\n"
        "      auto diff_model = std::make_shared<DifferentialActionModelPinocchioCasadi>(\n",
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      if (acceleration_weight_ > 0.0) {\n"
        "        auto acceleration_residual =\n"
        "            std::make_shared<ResidualModelAccelerationPinocchioCasadi>(state, a_zero);\n"
        "        auto acceleration_cost =\n"
        "            std::make_shared<crocoddyl::CostModelResidual>(state, acceleration_residual);\n"
        "        running_cost->addCost(\"acceleration_reg\", acceleration_cost, acceleration_weight_);\n"
        "      }\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n\n"
        "      auto diff_model = std::make_shared<DifferentialActionModelPinocchioCasadi>(\n",
        "casadi acceleration cost",
    )

    text = replace_checked(
        text,
        "    Eigen::VectorXd u_cmd = Eigen::VectorXd::Zero(ga_model_.dof_a);\n",
        "    Eigen::VectorXd u_cmd = Eigen::VectorXd::Zero(ga_model_.dof_a);\n"
        "    Eigen::VectorXd ddq_cmd = Eigen::VectorXd::Zero(ga_model_.dof_a);\n",
        "ddq init",
    )
    text = replace_checked(
        text,
        "    if (!solve.best_us.empty()) {\n"
        "      u_cmd = solve.best_us.front();\n"
        "    }\n",
        "    if (!solve.best_us.empty()) {\n"
        "      u_cmd = solve.best_us.front();\n"
        "    }\n"
        "    if (dt_ > 1e-12) {\n"
        "      ddq_cmd = (dq_cmd - joint_vel_) / dt_;\n"
        "    }\n",
        "ddq compute",
    )
    text = replace_checked(
        text,
        "    record.velocity_error = (joint_vel_ - x_ref_now.tail(ga_model_.dof_a)).norm();\n"
        "    record.torque_ratio = ComputeTorqueRatio(joint_effort_, effort_limit_);\n",
        "    record.velocity_error = (joint_vel_ - x_ref_now.tail(ga_model_.dof_a)).norm();\n"
        "    record.torque_ratio = ComputeTorqueRatio(joint_effort_, effort_limit_);\n"
        "    record.command_torque_ratio = ComputeTorqueRatio(u_cmd, effort_limit_);\n",
        "record command torque",
    )
    text = replace_checked(
        text,
        "    record.dq_cmd = FormatVector(dq_cmd);\n"
        "    record.u_cmd = FormatVector(u_cmd);\n",
        "    record.dq_cmd = FormatVector(dq_cmd);\n"
        "    record.ddq_cmd = FormatVector(ddq_cmd);\n"
        "    record.u_cmd = FormatVector(u_cmd);\n",
        "record ddq string",
    )

    text = replace_checked(
        text,
        "    out << \"robot,backend,t,tracking_error,velocity_error,torque_ratio,solve_time_ms,\"\n",
        "    out << \"robot,backend,t,tracking_error,velocity_error,torque_ratio,command_torque_ratio,solve_time_ms,\"\n",
        "cycle header command torque",
    )
    text = replace_checked(
        text,
        "           \"failure_message,q,dq,q_ref,dq_ref,q_cmd,dq_cmd,u_cmd,effort\\n\";\n",
        "           \"failure_message,q,dq,q_ref,dq_ref,q_cmd,dq_cmd,ddq_cmd,u_cmd,effort\\n\";\n",
        "cycle header ddq",
    )
    text = replace_checked(
        text,
        "          << LocalFormatCsvNumber(record.torque_ratio) << ','\n"
        "          << LocalFormatCsvNumber(record.solve_time_ms) << ','\n",
        "          << LocalFormatCsvNumber(record.torque_ratio) << ','\n"
        "          << LocalFormatCsvNumber(record.command_torque_ratio) << ','\n"
        "          << LocalFormatCsvNumber(record.solve_time_ms) << ','\n",
        "cycle row command torque",
    )
    text = replace_checked(
        text,
        "          << LocalCsvEscape(record.dq_cmd) << ','\n"
        "          << LocalCsvEscape(record.u_cmd) << ','\n",
        "          << LocalCsvEscape(record.dq_cmd) << ','\n"
        "          << LocalCsvEscape(record.ddq_cmd) << ','\n"
        "          << LocalCsvEscape(record.u_cmd) << ','\n",
        "cycle row ddq",
    )

    text = replace_checked(
        text,
        "    std::vector<double> torque_ratios;\n"
        "    std::vector<double> solve_times;\n",
        "    std::vector<double> torque_ratios;\n"
        "    std::vector<double> command_torque_ratios;\n"
        "    std::vector<double> solve_times;\n",
        "summary command torque vector",
    )
    text = replace_checked(
        text,
        "    torque_ratios.reserve(cycle_records_.size());\n"
        "    solve_times.reserve(cycle_records_.size());\n",
        "    torque_ratios.reserve(cycle_records_.size());\n"
        "    command_torque_ratios.reserve(cycle_records_.size());\n"
        "    solve_times.reserve(cycle_records_.size());\n",
        "summary command torque reserve",
    )
    text = replace_checked(
        text,
        "      torque_ratios.push_back(record.torque_ratio);\n"
        "      solve_times.push_back(record.solve_time_ms);\n",
        "      torque_ratios.push_back(record.torque_ratio);\n"
        "      command_torque_ratios.push_back(record.command_torque_ratio);\n"
        "      solve_times.push_back(record.solve_time_ms);\n",
        "summary command torque push",
    )
    text = replace_checked(
        text,
        "    out << \"robot,backend,num_cycles,tracking_rmse,tracking_mean,tracking_p95,torque_ratio_mean,\"\n"
        "           \"torque_ratio_p95,torque_ratio_max,solve_time_mean_ms,solve_time_p95_ms,\"\n",
        "    out << \"robot,backend,num_cycles,tracking_rmse,tracking_mean,tracking_p95,torque_ratio_mean,\"\n"
        "           \"torque_ratio_p95,torque_ratio_max,command_torque_ratio_mean,\"\n"
        "           \"command_torque_ratio_p95,command_torque_ratio_max,solve_time_mean_ms,solve_time_p95_ms,\"\n",
        "summary header command torque",
    )
    text = replace_checked(
        text,
        "           \"enable_collision_cost,collision_obstacle_count,collision_weight,collision_safety_distance,\"\n"
        "           \"realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n",
        "           \"enable_collision_cost,collision_obstacle_count,collision_weight,collision_safety_distance,\"\n"
        "           \"acceleration_weight,realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n",
        "summary header acceleration weight",
    )
    text = replace_checked(
        text,
        "        << LocalFormatCsvNumber(torque_ratios.empty()\n"
        "                               ? 0.0\n"
        "                               : *std::max_element(torque_ratios.begin(), torque_ratios.end()))\n"
        "        << ','\n"
        "        << LocalFormatCsvNumber(Mean(solve_times)) << ','\n",
        "        << LocalFormatCsvNumber(torque_ratios.empty()\n"
        "                               ? 0.0\n"
        "                               : *std::max_element(torque_ratios.begin(), torque_ratios.end()))\n"
        "        << ','\n"
        "        << LocalFormatCsvNumber(Mean(command_torque_ratios)) << ','\n"
        "        << LocalFormatCsvNumber(Percentile(command_torque_ratios, 0.95)) << ','\n"
        "        << LocalFormatCsvNumber(command_torque_ratios.empty()\n"
        "                               ? 0.0\n"
        "                               : *std::max_element(command_torque_ratios.begin(), command_torque_ratios.end()))\n"
        "        << ','\n"
        "        << LocalFormatCsvNumber(Mean(solve_times)) << ','\n",
        "summary row command torque",
    )
    text = replace_checked(
        text,
        "        << collision_obstacle_count_ << ','\n"
        "        << LocalFormatCsvNumber(collision_weight_) << ','\n"
        "        << LocalFormatCsvNumber(collision_safety_distance_) << ','\n"
        "        << LocalFormatCsvNumber(Mean(realtime_ratios)) << ','\n",
        "        << collision_obstacle_count_ << ','\n"
        "        << LocalFormatCsvNumber(collision_weight_) << ','\n"
        "        << LocalFormatCsvNumber(collision_safety_distance_) << ','\n"
        "        << LocalFormatCsvNumber(acceleration_weight_) << ','\n"
        "        << LocalFormatCsvNumber(Mean(realtime_ratios)) << ','\n",
        "summary row acceleration weight",
    )
    text = replace_checked(
        text,
        "  double control_weight_{1e-3};\n"
        "  double velocity_limit_weight_{20.0};\n",
        "  double control_weight_{1e-3};\n"
        "  double acceleration_weight_{0.0};\n"
        "  double velocity_limit_weight_{20.0};\n",
        "member acceleration weight",
    )

    NODE.write_text(text)


def patch_launch(path: Path) -> None:
    text = path.read_text()
    if "acceleration_weight = LaunchConfiguration('acceleration_weight')" not in text:
        text = replace_checked(
            text,
            "    solve_budget_ms = LaunchConfiguration('solve_budget_ms')\n",
            "    solve_budget_ms = LaunchConfiguration('solve_budget_ms')\n"
            "    acceleration_weight = LaunchConfiguration('acceleration_weight')\n",
            f"{path.name} launch configuration",
        )
    if "'acceleration_weight': ParameterValue(acceleration_weight, value_type=float)," not in text:
        text = replace_checked(
            text,
            "                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),\n",
            "                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),\n"
            "                'acceleration_weight': ParameterValue(acceleration_weight, value_type=float),\n",
            f"{path.name} node parameter",
        )
    if "DeclareLaunchArgument('acceleration_weight'" not in text:
        text = replace_checked(
            text,
            "        DeclareLaunchArgument('solve_budget_ms', default_value=",
            "        DeclareLaunchArgument('acceleration_weight', default_value='0.0'),\n"
            "        DeclareLaunchArgument('solve_budget_ms', default_value=",
            f"{path.name} declare argument",
        )
    path.write_text(text)


def main() -> int:
    patch_node()
    for path in [UR_LAUNCH, LEAP_LAUNCH, TIDY_LAUNCH]:
        patch_launch(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
