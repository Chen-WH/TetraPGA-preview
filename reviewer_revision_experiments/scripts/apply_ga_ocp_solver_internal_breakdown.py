#!/usr/bin/env python3
"""Add solver-internal runtime profiling for reviewer experiments."""

from __future__ import annotations

from pathlib import Path


ROOT = Path("/home/chenwh/ros2_ws/src/GA-OCP")
PROFILER = ROOT / "ga_ocp_core/include/ga_ocp/RuntimeProfiler.hpp"
ACTIONS = ROOT / "ga_ocp_core/include/ga_ocp/CrocoddylActions.hpp"
RESIDUALS = ROOT / "ga_ocp_core/include/ga_ocp/CrocoddylResiduals.hpp"
NODE = ROOT / "ga_ocp_ros2/src/closed_loop_mpc_node.cpp"
LAUNCH = ROOT / "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_ur.launch.py"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Unable to find replacement anchor for {label}")
    return text.replace(old, new, 1)


def write_profiler_header() -> None:
    PROFILER.write_text(
        r'''#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include <crocoddyl/core/cost-base.hpp>
#include <crocoddyl/core/data-collector-base.hpp>

namespace ga_ocp {

enum class RuntimeCostCategory : std::size_t {
  kState = 0,
  kControl = 1,
  kVelocity = 2,
  kCollision = 3,
  kOther = 4,
  kCount = 5,
};

constexpr std::size_t kRuntimeCostCategoryCount =
    static_cast<std::size_t>(RuntimeCostCategory::kCount);

struct RuntimeProfilerCounters {
  double dam_calc_ms = 0.0;
  double dam_calcdiff_ms = 0.0;
  double dynamics_calc_ms = 0.0;
  double dynamics_calcdiff_ms = 0.0;
  double cost_sum_calc_ms = 0.0;
  double cost_sum_calcdiff_ms = 0.0;
  double collision_residual_calc_ms = 0.0;
  double collision_residual_calcdiff_ms = 0.0;

  std::uint64_t dam_calc_calls = 0;
  std::uint64_t dam_calcdiff_calls = 0;
  std::uint64_t dynamics_calc_calls = 0;
  std::uint64_t dynamics_calcdiff_calls = 0;
  std::uint64_t cost_sum_calc_calls = 0;
  std::uint64_t cost_sum_calcdiff_calls = 0;
  std::uint64_t collision_residual_calc_calls = 0;
  std::uint64_t collision_residual_calcdiff_calls = 0;

  std::array<double, kRuntimeCostCategoryCount> cost_item_calc_ms{};
  std::array<double, kRuntimeCostCategoryCount> cost_item_calcdiff_ms{};
  std::array<std::uint64_t, kRuntimeCostCategoryCount> cost_item_calc_calls{};
  std::array<std::uint64_t, kRuntimeCostCategoryCount> cost_item_calcdiff_calls{};
};

inline RuntimeProfilerCounters& MutableRuntimeProfilerCounters() {
  static thread_local RuntimeProfilerCounters counters;
  return counters;
}

inline bool& MutableRuntimeProfilerEnabled() {
  static thread_local bool enabled = false;
  return enabled;
}

inline void SetRuntimeProfilerEnabled(const bool enabled) {
  MutableRuntimeProfilerEnabled() = enabled;
}

inline bool RuntimeProfilerEnabled() {
  return MutableRuntimeProfilerEnabled();
}

inline void ResetRuntimeProfilerCounters() {
  MutableRuntimeProfilerCounters() = RuntimeProfilerCounters{};
}

inline RuntimeProfilerCounters SnapshotRuntimeProfilerCounters() {
  return MutableRuntimeProfilerCounters();
}

inline std::chrono::steady_clock::time_point RuntimeProfilerNow() {
  return std::chrono::steady_clock::now();
}

inline double RuntimeProfilerElapsedMs(
    const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() * 1e3;
}

inline void RuntimeProfilerRecordDamCalc(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.dam_calc_ms += ms;
  ++c.dam_calc_calls;
}

inline void RuntimeProfilerRecordDamCalcDiff(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.dam_calcdiff_ms += ms;
  ++c.dam_calcdiff_calls;
}

inline void RuntimeProfilerRecordDynamicsCalc(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.dynamics_calc_ms += ms;
  ++c.dynamics_calc_calls;
}

inline void RuntimeProfilerRecordDynamicsCalcDiff(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.dynamics_calcdiff_ms += ms;
  ++c.dynamics_calcdiff_calls;
}

inline void RuntimeProfilerRecordCostSumCalc(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.cost_sum_calc_ms += ms;
  ++c.cost_sum_calc_calls;
}

inline void RuntimeProfilerRecordCostSumCalcDiff(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.cost_sum_calcdiff_ms += ms;
  ++c.cost_sum_calcdiff_calls;
}

inline void RuntimeProfilerRecordCollisionResidualCalc(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.collision_residual_calc_ms += ms;
  ++c.collision_residual_calc_calls;
}

inline void RuntimeProfilerRecordCollisionResidualCalcDiff(const double ms) {
  auto& c = MutableRuntimeProfilerCounters();
  c.collision_residual_calcdiff_ms += ms;
  ++c.collision_residual_calcdiff_calls;
}

inline void RuntimeProfilerRecordCostItemCalc(
    const RuntimeCostCategory category, const double ms) {
  const std::size_t index = static_cast<std::size_t>(category);
  auto& c = MutableRuntimeProfilerCounters();
  c.cost_item_calc_ms[index] += ms;
  ++c.cost_item_calc_calls[index];
}

inline void RuntimeProfilerRecordCostItemCalcDiff(
    const RuntimeCostCategory category, const double ms) {
  const std::size_t index = static_cast<std::size_t>(category);
  auto& c = MutableRuntimeProfilerCounters();
  c.cost_item_calcdiff_ms[index] += ms;
  ++c.cost_item_calcdiff_calls[index];
}

inline double RuntimeProfilerModelTimeMs(const RuntimeProfilerCounters& counters) {
  return counters.dam_calc_ms + counters.dam_calcdiff_ms;
}

inline double RuntimeProfilerCostItemCalcMs(
    const RuntimeProfilerCounters& counters, const RuntimeCostCategory category) {
  return counters.cost_item_calc_ms[static_cast<std::size_t>(category)];
}

inline double RuntimeProfilerCostItemCalcDiffMs(
    const RuntimeProfilerCounters& counters, const RuntimeCostCategory category) {
  return counters.cost_item_calcdiff_ms[static_cast<std::size_t>(category)];
}

inline double RuntimeProfilerCostItemTotalMs(
    const RuntimeProfilerCounters& counters, const RuntimeCostCategory category) {
  return RuntimeProfilerCostItemCalcMs(counters, category) +
         RuntimeProfilerCostItemCalcDiffMs(counters, category);
}

template <typename Scalar>
class ProfilingCostModelTpl : public crocoddyl::CostModelAbstractTpl<Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = crocoddyl::CostModelAbstractTpl<Scalar>;
  using CostDataAbstract = crocoddyl::CostDataAbstractTpl<Scalar>;
  using DataCollectorAbstract = crocoddyl::DataCollectorAbstractTpl<Scalar>;
  using VectorXs = typename crocoddyl::MathBaseTpl<Scalar>::VectorXs;

  ProfilingCostModelTpl(std::shared_ptr<Base> inner, const RuntimeCostCategory category)
      : Base(inner->get_state(), inner->get_activation(), inner->get_residual()),
        inner_(std::move(inner)),
        category_(category) {}

  void calc(const std::shared_ptr<CostDataAbstract>& data,
            const Eigen::Ref<const VectorXs>& x,
            const Eigen::Ref<const VectorXs>& u) override {
    if (!RuntimeProfilerEnabled()) {
      inner_->calc(data, x, u);
      return;
    }
    const auto start = RuntimeProfilerNow();
    inner_->calc(data, x, u);
    RuntimeProfilerRecordCostItemCalc(category_, RuntimeProfilerElapsedMs(start));
  }

  void calc(const std::shared_ptr<CostDataAbstract>& data,
            const Eigen::Ref<const VectorXs>& x) override {
    if (!RuntimeProfilerEnabled()) {
      inner_->calc(data, x);
      return;
    }
    const auto start = RuntimeProfilerNow();
    inner_->calc(data, x);
    RuntimeProfilerRecordCostItemCalc(category_, RuntimeProfilerElapsedMs(start));
  }

  void calcDiff(const std::shared_ptr<CostDataAbstract>& data,
                const Eigen::Ref<const VectorXs>& x,
                const Eigen::Ref<const VectorXs>& u) override {
    if (!RuntimeProfilerEnabled()) {
      inner_->calcDiff(data, x, u);
      return;
    }
    const auto start = RuntimeProfilerNow();
    inner_->calcDiff(data, x, u);
    RuntimeProfilerRecordCostItemCalcDiff(category_, RuntimeProfilerElapsedMs(start));
  }

  void calcDiff(const std::shared_ptr<CostDataAbstract>& data,
                const Eigen::Ref<const VectorXs>& x) override {
    if (!RuntimeProfilerEnabled()) {
      inner_->calcDiff(data, x);
      return;
    }
    const auto start = RuntimeProfilerNow();
    inner_->calcDiff(data, x);
    RuntimeProfilerRecordCostItemCalcDiff(category_, RuntimeProfilerElapsedMs(start));
  }

  std::shared_ptr<CostDataAbstract> createData(
      DataCollectorAbstract* const data) override {
    return inner_->createData(data);
  }

  std::shared_ptr<crocoddyl::CostModelBase> cloneAsDouble() const override {
    return inner_->cloneAsDouble();
  }

  std::shared_ptr<crocoddyl::CostModelBase> cloneAsFloat() const override {
    return inner_->cloneAsFloat();
  }

 private:
  std::shared_ptr<Base> inner_;
  RuntimeCostCategory category_;
};

template <typename Scalar>
std::shared_ptr<crocoddyl::CostModelAbstractTpl<Scalar>> ProfileCost(
    const std::shared_ptr<crocoddyl::CostModelAbstractTpl<Scalar>>& inner,
    const RuntimeCostCategory category) {
  return std::make_shared<ProfilingCostModelTpl<Scalar>>(inner, category);
}

}  // namespace ga_ocp
''',
        encoding="utf-8",
    )


def patch_actions() -> None:
    text = ACTIONS.read_text(encoding="utf-8")
    text = replace_once(
        text,
        '#include "TetraPGA/Kinematics.hpp"\n',
        '#include "TetraPGA/Kinematics.hpp"\n#include "ga_ocp/RuntimeProfiler.hpp"\n',
        "actions profiler include",
    )

    text = replace_once(
        text,
        "    // 3. 调用正向动力学\n"
        "    // forwardDynamics 计算出 acceleration 并存入 ga_data.ddq\n"
        "    forwardDynamics0(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n"
        "\n"
        "    // 4. 将结果赋值给 Crocoddyl 需要的 xout (即 acceleration)\n"
        "    d->xout = d->ga_data.ddq;\n"
        "\n"
        "    // 5. 计算 Cost\n"
        "    if (costs_) {\n"
        "      costs_->calc(d->costs, x, u);\n"
        "      d->cost = d->costs->cost;\n"
        "    } else {\n"
        "      d->cost = 0;\n"
        "    }\n",
        "    const bool profile = ga_ocp::RuntimeProfilerEnabled();\n"
        "    const auto dam_start = ga_ocp::RuntimeProfilerNow();\n"
        "\n"
        "    // 3. 调用正向动力学\n"
        "    // forwardDynamics 计算出 acceleration 并存入 ga_data.ddq\n"
        "    const auto dynamics_start = ga_ocp::RuntimeProfilerNow();\n"
        "    forwardDynamics0(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDynamicsCalc(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dynamics_start));\n"
        "    }\n"
        "\n"
        "    // 4. 将结果赋值给 Crocoddyl 需要的 xout (即 acceleration)\n"
        "    d->xout = d->ga_data.ddq;\n"
        "\n"
        "    // 5. 计算 Cost\n"
        "    if (costs_) {\n"
        "      const auto cost_start = ga_ocp::RuntimeProfilerNow();\n"
        "      costs_->calc(d->costs, x, u);\n"
        "      if (profile) {\n"
        "        ga_ocp::RuntimeProfilerRecordCostSumCalc(\n"
        "            ga_ocp::RuntimeProfilerElapsedMs(cost_start));\n"
        "      }\n"
        "      d->cost = d->costs->cost;\n"
        "    } else {\n"
        "      d->cost = 0;\n"
        "    }\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDamCalc(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dam_start));\n"
        "    }\n",
        "forward action calc profiler",
    )

    text = replace_once(
        text,
        "\t    // 1. 调用一阶导数算法\n"
        "\t    // 该函数会填充 ga_data.pddq_pq, ga_data.pddq_pdq, ga_data.pddq_ptau\n"
        "\t    forwardDynamics_fo(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n",
        "    const bool profile = ga_ocp::RuntimeProfilerEnabled();\n"
        "    const auto dam_start = ga_ocp::RuntimeProfilerNow();\n"
        "\n"
        "    // 1. 调用一阶导数算法\n"
        "    // 该函数会填充 ga_data.pddq_pq, ga_data.pddq_pdq, ga_data.pddq_ptau\n"
        "    const auto dynamics_start = ga_ocp::RuntimeProfilerNow();\n"
        "    forwardDynamics_fo(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDynamicsCalcDiff(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dynamics_start));\n"
        "    }\n",
        "forward action calcDiff dynamics profiler",
    )

    text = replace_once(
        text,
        "    if (costs_) {\n"
        "      costs_->calcDiff(d->costs, x, u);\n"
        "      // 关键：将 cost 梯度复制到 action data\n"
        "      d->Lx = d->costs->Lx;\n"
        "      d->Lu = d->costs->Lu;\n"
        "      d->Lxx = d->costs->Lxx;\n"
        "      d->Lxu = d->costs->Lxu;\n"
        "      d->Luu = d->costs->Luu;\n"
        "    }\n",
        "    if (costs_) {\n"
        "      const auto cost_start = ga_ocp::RuntimeProfilerNow();\n"
        "      costs_->calcDiff(d->costs, x, u);\n"
        "      if (profile) {\n"
        "        ga_ocp::RuntimeProfilerRecordCostSumCalcDiff(\n"
        "            ga_ocp::RuntimeProfilerElapsedMs(cost_start));\n"
        "      }\n"
        "      // 关键：将 cost 梯度复制到 action data\n"
        "      d->Lx = d->costs->Lx;\n"
        "      d->Lu = d->costs->Lu;\n"
        "      d->Lxx = d->costs->Lxx;\n"
        "      d->Lxu = d->costs->Lxu;\n"
        "      d->Luu = d->costs->Luu;\n"
        "    }\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDamCalcDiff(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dam_start));\n"
        "    }\n",
        "forward action calcDiff cost profiler",
    )

    text = replace_once(
        text,
        "    inverseDynamics0(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n"
        "\n"
        "    d->xout = u;\n"
        "\n"
        "    if (costs_) {\n"
        "      costs_->calc(d->costs, x, u);\n"
        "      d->cost = d->costs->cost;\n"
        "    } else {\n"
        "      d->cost = 0;\n"
        "    }\n",
        "    const bool profile = ga_ocp::RuntimeProfilerEnabled();\n"
        "    const auto dam_start = ga_ocp::RuntimeProfilerNow();\n"
        "    const auto dynamics_start = ga_ocp::RuntimeProfilerNow();\n"
        "    inverseDynamics0(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDynamicsCalc(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dynamics_start));\n"
        "    }\n"
        "\n"
        "    d->xout = u;\n"
        "\n"
        "    if (costs_) {\n"
        "      const auto cost_start = ga_ocp::RuntimeProfilerNow();\n"
        "      costs_->calc(d->costs, x, u);\n"
        "      if (profile) {\n"
        "        ga_ocp::RuntimeProfilerRecordCostSumCalc(\n"
        "            ga_ocp::RuntimeProfilerElapsedMs(cost_start));\n"
        "      }\n"
        "      d->cost = d->costs->cost;\n"
        "    } else {\n"
        "      d->cost = 0;\n"
        "    }\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDamCalc(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dam_start));\n"
        "    }\n",
        "inverse action calc profiler",
    )

    text = replace_once(
        text,
        "    inverseDynamics_fo(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n",
        "    const bool profile = ga_ocp::RuntimeProfilerEnabled();\n"
        "    const auto dam_start = ga_ocp::RuntimeProfilerNow();\n"
        "    const auto dynamics_start = ga_ocp::RuntimeProfilerNow();\n"
        "    inverseDynamics_fo(ga_model_, d->ga_data, x.head(nq), x.tail(nv), u);\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDynamicsCalcDiff(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dynamics_start));\n"
        "    }\n",
        "inverse action calcDiff dynamics profiler",
    )

    text = replace_once(
        text,
        "    if (costs_) {\n"
        "      costs_->calcDiff(d->costs, x, u);\n"
        "      d->Lx = d->costs->Lx;\n"
        "      d->Lu = d->costs->Lu;\n"
        "      d->Lxx = d->costs->Lxx;\n"
        "      d->Lxu = d->costs->Lxu;\n"
        "      d->Luu = d->costs->Luu;\n"
        "    }\n",
        "    if (costs_) {\n"
        "      const auto cost_start = ga_ocp::RuntimeProfilerNow();\n"
        "      costs_->calcDiff(d->costs, x, u);\n"
        "      if (profile) {\n"
        "        ga_ocp::RuntimeProfilerRecordCostSumCalcDiff(\n"
        "            ga_ocp::RuntimeProfilerElapsedMs(cost_start));\n"
        "      }\n"
        "      d->Lx = d->costs->Lx;\n"
        "      d->Lu = d->costs->Lu;\n"
        "      d->Lxx = d->costs->Lxx;\n"
        "      d->Lxu = d->costs->Lxu;\n"
        "      d->Luu = d->costs->Luu;\n"
        "    }\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordDamCalcDiff(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(dam_start));\n"
        "    }\n",
        "inverse action calcDiff cost profiler",
    )

    ACTIONS.write_text(text, encoding="utf-8")


def patch_residuals() -> None:
    text = RESIDUALS.read_text(encoding="utf-8")
    text = replace_once(
        text,
        '#include "ga_ocp/CrocoddylActions.hpp"\n',
        '#include "ga_ocp/CrocoddylActions.hpp"\n#include "ga_ocp/RuntimeProfiler.hpp"\n',
        "residuals profiler include",
    )
    text = replace_once(
        text,
        "    auto* d = static_cast<ResidualDataTetraPGACollisionDistance<Scalar>*>(data.get());\n"
        "    \n"
        "    // Compute collision distances for all pairs\n"
        "    computeDistance(ga_model_, *(d->ga_data), d->env, d->env_data);\n"
        "    \n"
        "    // Set residual as (d_safe - distance) for each collision pair\n"
        "    // Activation model will handle max(0, r) to create barrier\n"
        "    for (int i = 0; i < d->env_data.num_collision_pair; ++i) {\n"
        "      data->r(i) = d_safe_ - d->env_data.distance[i];\n"
        "    }\n",
        "    const bool profile = ga_ocp::RuntimeProfilerEnabled();\n"
        "    const auto start = ga_ocp::RuntimeProfilerNow();\n"
        "    auto* d = static_cast<ResidualDataTetraPGACollisionDistance<Scalar>*>(data.get());\n"
        "    \n"
        "    // Compute collision distances for all pairs\n"
        "    computeDistance(ga_model_, *(d->ga_data), d->env, d->env_data);\n"
        "    \n"
        "    // Set residual as (d_safe - distance) for each collision pair\n"
        "    // Activation model will handle max(0, r) to create barrier\n"
        "    for (int i = 0; i < d->env_data.num_collision_pair; ++i) {\n"
        "      data->r(i) = d_safe_ - d->env_data.distance[i];\n"
        "    }\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordCollisionResidualCalc(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(start));\n"
        "    }\n",
        "collision residual calc profiler",
    )
    text = replace_once(
        text,
        "    auto* d = static_cast<ResidualDataTetraPGACollisionDistance<Scalar>*>(data.get());\n"
        "    const std::size_t nq = this->get_state()->get_nq();\n"
        "    // Baseline path: recompute witness geometry inside calcDiff.\n"
        "    computeDistanceJacobian(ga_model_, *(d->ga_data), d->env, d->env_data);\n"
        "    \n"
        "    data->Rx.setZero();\n"
        "    data->Ru.setZero();\n"
        "    \n"
        "    // Set Jacobian for each collision pair\n"
        "    // Negative sign because residual is (d_safe - distance)\n"
        "    // dr/dq = -d(distance)/dq\n"
        "    for (int i = 0; i < d->env_data.num_collision_pair; ++i) {\n"
        "      data->Rx.row(i).head(nq) = -d->env_data.jac_dist[i];\n"
        "    }\n",
        "    const bool profile = ga_ocp::RuntimeProfilerEnabled();\n"
        "    const auto start = ga_ocp::RuntimeProfilerNow();\n"
        "    auto* d = static_cast<ResidualDataTetraPGACollisionDistance<Scalar>*>(data.get());\n"
        "    const std::size_t nq = this->get_state()->get_nq();\n"
        "    // Baseline path: recompute witness geometry inside calcDiff.\n"
        "    computeDistanceJacobian(ga_model_, *(d->ga_data), d->env, d->env_data);\n"
        "    \n"
        "    data->Rx.setZero();\n"
        "    data->Ru.setZero();\n"
        "    \n"
        "    // Set Jacobian for each collision pair\n"
        "    // Negative sign because residual is (d_safe - distance)\n"
        "    // dr/dq = -d(distance)/dq\n"
        "    for (int i = 0; i < d->env_data.num_collision_pair; ++i) {\n"
        "      data->Rx.row(i).head(nq) = -d->env_data.jac_dist[i];\n"
        "    }\n"
        "    if (profile) {\n"
        "      ga_ocp::RuntimeProfilerRecordCollisionResidualCalcDiff(\n"
        "          ga_ocp::RuntimeProfilerElapsedMs(start));\n"
        "    }\n",
        "collision residual calcDiff profiler",
    )
    RESIDUALS.write_text(text, encoding="utf-8")


def patch_node() -> None:
    text = NODE.read_text(encoding="utf-8")

    text = replace_once(
        text,
        "#include \"ga_ocp/CrocoddylResiduals.hpp\"\n"
        "#include \"TetraPGA/ModelRepo.hpp\"\n",
        "#include \"ga_ocp/CrocoddylResiduals.hpp\"\n"
        "#include \"ga_ocp/RuntimeProfiler.hpp\"\n"
        "#include \"TetraPGA/Collision.hpp\"\n"
        "#include \"TetraPGA/ModelRepo.hpp\"\n",
        "node profiler include",
    )

    text = replace_once(
        text,
        "  double solver_setup_ms = 0.0;\n"
        "  std::size_t iterations = 0;\n",
        "  double solver_setup_ms = 0.0;\n"
        "  ga_ocp::RuntimeProfilerCounters initial_profile;\n"
        "  ga_ocp::RuntimeProfilerCounters solver_profile;\n"
        "  std::size_t iterations = 0;\n",
        "solve result profiler fields",
    )

    text = replace_once(
        text,
        "  double publish_command_ms = 0.0;\n"
        "  double cycle_time_ms = 0.0;\n",
        "  double publish_command_ms = 0.0;\n"
        "  ga_ocp::RuntimeProfilerCounters initial_profile;\n"
        "  ga_ocp::RuntimeProfilerCounters solver_profile;\n"
        "  double cycle_time_ms = 0.0;\n",
        "cycle record profiler fields",
    )

    text = replace_once(
        text,
        "double ComputeTorqueRatio(const Eigen::VectorXd& effort, const Eigen::VectorXd& effort_limit) {\n",
        "std::shared_ptr<crocoddyl::CostModelAbstract> ProfileCost(\n"
        "    const std::shared_ptr<crocoddyl::CostModelAbstract>& cost,\n"
        "    const ga_ocp::RuntimeCostCategory category) {\n"
        "  return ga_ocp::ProfileCost<double>(cost, category);\n"
        "}\n"
        "\n"
        "double RuntimeCostItemTotalMs(const ga_ocp::RuntimeProfilerCounters& counters,\n"
        "                              const ga_ocp::RuntimeCostCategory category) {\n"
        "  return ga_ocp::RuntimeProfilerCostItemTotalMs(counters, category);\n"
        "}\n"
        "\n"
        "double RuntimeCostItemTotalMs(const ga_ocp::RuntimeProfilerCounters& counters) {\n"
        "  double total = 0.0;\n"
        "  for (std::size_t i = 0; i < ga_ocp::kRuntimeCostCategoryCount; ++i) {\n"
        "    total += counters.cost_item_calc_ms[i] + counters.cost_item_calcdiff_ms[i];\n"
        "  }\n"
        "  return total;\n"
        "}\n"
        "\n"
        "Environment<double> MakeRuntimeCollisionEnvironment(const int obstacle_count) {\n"
        "  const std::array<Eigen::Vector3d, 8> centers{\n"
        "      Eigen::Vector3d(0.15, -0.35, 0.62), Eigen::Vector3d(-0.25, 0.32, 0.82),\n"
        "      Eigen::Vector3d(-0.42, -0.05, 0.55), Eigen::Vector3d(0.05, 0.42, 0.95),\n"
        "      Eigen::Vector3d(-0.52, 0.18, 0.72), Eigen::Vector3d(0.28, 0.12, 0.48),\n"
        "      Eigen::Vector3d(-0.12, -0.48, 0.88), Eigen::Vector3d(0.34, -0.18, 1.08)};\n"
        "  std::vector<SSP<double>> spheres;\n"
        "  const int count = std::max(0, std::min<int>(obstacle_count, centers.size()));\n"
        "  spheres.reserve(static_cast<std::size_t>(count));\n"
        "  for (int i = 0; i < count; ++i) {\n"
        "    SSP<double> sphere;\n"
        "    sphere.id = i;\n"
        "    sphere.radius = 0.055;\n"
        "    sphere.center = Point3D<double>(centers[static_cast<std::size_t>(i)].x(),\n"
        "                                    centers[static_cast<std::size_t>(i)].y(),\n"
        "                                    centers[static_cast<std::size_t>(i)].z(), 1.0);\n"
        "    spheres.push_back(sphere);\n"
        "  }\n"
        "  return Environment<double>(spheres);\n"
        "}\n"
        "\n"
        "double ComputeTorqueRatio(const Eigen::VectorXd& effort, const Eigen::VectorXd& effort_limit) {\n",
        "runtime helpers",
    )

    text = replace_once(
        text,
        "    external_force_duration_s_ = this->declare_parameter<double>(\"external_force_duration_s\", 0.0);\n"
        "\n"
        "    std::vector<double> amplitude_override = this->declare_parameter<std::vector<double>>(\n",
        "    external_force_duration_s_ = this->declare_parameter<double>(\"external_force_duration_s\", 0.0);\n"
        "\n"
        "    enable_runtime_profiler_ = this->declare_parameter<bool>(\"enable_runtime_profiler\", true);\n"
        "    enable_collision_cost_ = this->declare_parameter<bool>(\"enable_collision_cost\", false);\n"
        "    collision_weight_ = this->declare_parameter<double>(\"collision_weight\", 50.0);\n"
        "    collision_safety_distance_ = this->declare_parameter<double>(\"collision_safety_distance\", 0.08);\n"
        "    collision_obstacle_count_ = this->declare_parameter<int>(\"collision_obstacle_count\", 4);\n"
        "    ga_ocp::SetRuntimeProfilerEnabled(enable_runtime_profiler_);\n"
        "\n"
        "    std::vector<double> amplitude_override = this->declare_parameter<std::vector<double>>(\n",
        "runtime params",
    )

    text = replace_once(
        text,
        "      auto state_cost = std::make_shared<crocoddyl::CostModelResidual>(state, state_residual);\n",
        "      std::shared_ptr<crocoddyl::CostModelAbstract> state_cost = ProfileCost(\n"
        "          std::make_shared<crocoddyl::CostModelResidual>(state, state_residual),\n"
        "          ga_ocp::RuntimeCostCategory::kState);\n",
        "ga state cost wrapper",
    )
    text = replace_once(
        text,
        "      auto control_cost =\n"
        "          std::make_shared<crocoddyl::CostModelResidual>(state, control_residual);\n",
        "      std::shared_ptr<crocoddyl::CostModelAbstract> control_cost = ProfileCost(\n"
        "          std::make_shared<crocoddyl::CostModelResidual>(state, control_residual),\n"
        "          ga_ocp::RuntimeCostCategory::kControl);\n",
        "ga control cost wrapper",
    )
    text = replace_once(
        text,
        "      auto vel_cost =\n"
        "          std::make_shared<crocoddyl::CostModelResidual>(state, vel_activation, vel_residual);\n",
        "      std::shared_ptr<crocoddyl::CostModelAbstract> vel_cost = ProfileCost(\n"
        "          std::make_shared<crocoddyl::CostModelResidual>(state, vel_activation, vel_residual),\n"
        "          ga_ocp::RuntimeCostCategory::kVelocity);\n",
        "ga velocity cost wrapper",
    )

    text = replace_once(
        text,
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n",
        "      running_cost->addCost(\"state_reg\", state_cost, state_running_weight_);\n"
        "      running_cost->addCost(\"control_reg\", control_cost, control_weight_);\n"
        "      running_cost->addCost(\"vel_limit\", vel_cost, velocity_limit_weight_);\n"
        "      if (enable_collision_cost_ && ga_model_.num_collision_ssl > 0 && collision_obstacle_count_ > 0) {\n"
        "        const Environment<double> env = MakeRuntimeCollisionEnvironment(collision_obstacle_count_);\n"
        "        auto collision_residual = std::make_shared<ResidualModelTetraPGACollisionDistance<double>>(\n"
        "            state, ga_model_, env, collision_safety_distance_);\n"
        "        const int num_collision_pairs = ga_model_.num_collision_ssl * env.num_static_sphere;\n"
        "        crocoddyl::ActivationBounds collision_bounds;\n"
        "        collision_bounds.lb = Eigen::VectorXd::Zero(num_collision_pairs);\n"
        "        collision_bounds.ub = Eigen::VectorXd::Constant(\n"
        "            num_collision_pairs, std::numeric_limits<double>::infinity());\n"
        "        auto collision_activation =\n"
        "            std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(collision_bounds);\n"
        "        std::shared_ptr<crocoddyl::CostModelAbstract> collision_cost = ProfileCost(\n"
        "            std::make_shared<crocoddyl::CostModelResidual>(\n"
        "                state, collision_activation, collision_residual),\n"
        "            ga_ocp::RuntimeCostCategory::kCollision);\n"
        "        running_cost->addCost(\"collision\", collision_cost, collision_weight_);\n"
        "      }\n",
        "ga collision cost insertion",
    )

    text = replace_once(
        text,
        "    auto terminal_state_cost =\n"
        "        std::make_shared<crocoddyl::CostModelResidual>(state, terminal_state_residual);\n"
        "    terminal_cost->addCost(\"state_reg\", terminal_state_cost, state_terminal_weight_);\n",
        "    std::shared_ptr<crocoddyl::CostModelAbstract> terminal_state_cost = ProfileCost(\n"
        "        std::make_shared<crocoddyl::CostModelResidual>(state, terminal_state_residual),\n"
        "        ga_ocp::RuntimeCostCategory::kState);\n"
        "    terminal_cost->addCost(\"state_reg\", terminal_state_cost, state_terminal_weight_);\n"
        "    if (enable_collision_cost_ && ga_model_.num_collision_ssl > 0 && collision_obstacle_count_ > 0) {\n"
        "      const Environment<double> env = MakeRuntimeCollisionEnvironment(collision_obstacle_count_);\n"
        "      auto collision_residual = std::make_shared<ResidualModelTetraPGACollisionDistance<double>>(\n"
        "          state, ga_model_, env, collision_safety_distance_);\n"
        "      const int num_collision_pairs = ga_model_.num_collision_ssl * env.num_static_sphere;\n"
        "      crocoddyl::ActivationBounds collision_bounds;\n"
        "      collision_bounds.lb = Eigen::VectorXd::Zero(num_collision_pairs);\n"
        "      collision_bounds.ub = Eigen::VectorXd::Constant(\n"
        "          num_collision_pairs, std::numeric_limits<double>::infinity());\n"
        "      auto collision_activation =\n"
        "          std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(collision_bounds);\n"
        "      std::shared_ptr<crocoddyl::CostModelAbstract> collision_cost = ProfileCost(\n"
        "          std::make_shared<crocoddyl::CostModelResidual>(\n"
        "              state, collision_activation, collision_residual),\n"
        "          ga_ocp::RuntimeCostCategory::kCollision);\n"
        "      terminal_cost->addCost(\"collision\", collision_cost, collision_weight_);\n"
        "    }\n",
        "ga terminal cost wrapper and collision",
    )

    text = replace_once(
        text,
        "    result.best_cost = problem->calc(init_xs, init_us);\n"
        "    result.initial_calc_ms = DurationSeconds(Clock::now() - initial_calc_start).count() * 1e3;\n",
        "    ga_ocp::ResetRuntimeProfilerCounters();\n"
        "    result.best_cost = problem->calc(init_xs, init_us);\n"
        "    result.initial_profile = ga_ocp::SnapshotRuntimeProfilerCounters();\n"
        "    result.initial_calc_ms = DurationSeconds(Clock::now() - initial_calc_start).count() * 1e3;\n",
        "initial profile snapshot",
    )

    text = replace_once(
        text,
        "    bool is_feasible = false;\n"
        "    const Clock::time_point start_time = Clock::now();\n",
        "    bool is_feasible = false;\n"
        "    ga_ocp::ResetRuntimeProfilerCounters();\n"
        "    const Clock::time_point start_time = Clock::now();\n",
        "solver profile reset",
    )

    text = replace_once(
        text,
        "    result.solve_time_ms = DurationSeconds(Clock::now() - start_time).count() * 1e3;\n"
        "    return result;\n",
        "    result.solve_time_ms = DurationSeconds(Clock::now() - start_time).count() * 1e3;\n"
        "    result.solver_profile = ga_ocp::SnapshotRuntimeProfilerCounters();\n"
        "    return result;\n",
        "solver profile snapshot",
    )

    text = replace_once(
        text,
        "    record.publish_command_ms = publish_command_ms;\n"
        "    record.cycle_time_ms = DurationSeconds(Clock::now() - cycle_start).count() * 1e3;\n",
        "    record.publish_command_ms = publish_command_ms;\n"
        "    record.initial_profile = solve.initial_profile;\n"
        "    record.solver_profile = solve.solver_profile;\n"
        "    record.cycle_time_ms = DurationSeconds(Clock::now() - cycle_start).count() * 1e3;\n",
        "record profiler snapshot",
    )

    text = replace_once(
        text,
        "           \"solver_setup_ms,mpc_pipeline_time_ms,publish_command_ms,\"\n"
        "           \"cycle_time_ms,realtime_ratio,iterations,converged,failed,best_cost,final_stop,\"\n",
        "           \"solver_setup_ms,mpc_pipeline_time_ms,publish_command_ms,\"\n"
        "           \"solver_dam_calc_ms,solver_dam_calcdiff_ms,solver_dynamics_calc_ms,\"\n"
        "           \"solver_dynamics_calcdiff_ms,solver_cost_sum_calc_ms,solver_cost_sum_calcdiff_ms,\"\n"
        "           \"solver_cost_item_total_ms,solver_state_cost_total_ms,solver_control_cost_total_ms,\"\n"
        "           \"solver_velocity_cost_total_ms,solver_collision_cost_total_ms,\"\n"
        "           \"solver_collision_residual_total_ms,solver_model_total_ms,solver_overhead_ms,\"\n"
        "           \"solver_dam_calc_calls,solver_dam_calcdiff_calls,\"\n"
        "           \"initial_dam_calc_ms,initial_cost_sum_calc_ms,initial_model_total_ms,\"\n"
        "           \"cycle_time_ms,realtime_ratio,iterations,converged,failed,best_cost,final_stop,\"\n",
        "cycle csv profiler header",
    )

    text = replace_once(
        text,
        "          << FormatCsvNumber(record.publish_command_ms) << ','\n"
        "          << FormatCsvNumber(record.cycle_time_ms) << ','\n",
        "          << FormatCsvNumber(record.publish_command_ms) << ','\n"
        "          << FormatCsvNumber(record.solver_profile.dam_calc_ms) << ','\n"
        "          << FormatCsvNumber(record.solver_profile.dam_calcdiff_ms) << ','\n"
        "          << FormatCsvNumber(record.solver_profile.dynamics_calc_ms) << ','\n"
        "          << FormatCsvNumber(record.solver_profile.dynamics_calcdiff_ms) << ','\n"
        "          << FormatCsvNumber(record.solver_profile.cost_sum_calc_ms) << ','\n"
        "          << FormatCsvNumber(record.solver_profile.cost_sum_calcdiff_ms) << ','\n"
        "          << FormatCsvNumber(RuntimeCostItemTotalMs(record.solver_profile)) << ','\n"
        "          << FormatCsvNumber(RuntimeCostItemTotalMs(record.solver_profile, ga_ocp::RuntimeCostCategory::kState)) << ','\n"
        "          << FormatCsvNumber(RuntimeCostItemTotalMs(record.solver_profile, ga_ocp::RuntimeCostCategory::kControl)) << ','\n"
        "          << FormatCsvNumber(RuntimeCostItemTotalMs(record.solver_profile, ga_ocp::RuntimeCostCategory::kVelocity)) << ','\n"
        "          << FormatCsvNumber(RuntimeCostItemTotalMs(record.solver_profile, ga_ocp::RuntimeCostCategory::kCollision)) << ','\n"
        "          << FormatCsvNumber(record.solver_profile.collision_residual_calc_ms +\n"
        "                             record.solver_profile.collision_residual_calcdiff_ms) << ','\n"
        "          << FormatCsvNumber(ga_ocp::RuntimeProfilerModelTimeMs(record.solver_profile)) << ','\n"
        "          << FormatCsvNumber(record.solve_time_ms -\n"
        "                             ga_ocp::RuntimeProfilerModelTimeMs(record.solver_profile)) << ','\n"
        "          << record.solver_profile.dam_calc_calls << ','\n"
        "          << record.solver_profile.dam_calcdiff_calls << ','\n"
        "          << FormatCsvNumber(record.initial_profile.dam_calc_ms) << ','\n"
        "          << FormatCsvNumber(record.initial_profile.cost_sum_calc_ms) << ','\n"
        "          << FormatCsvNumber(ga_ocp::RuntimeProfilerModelTimeMs(record.initial_profile)) << ','\n"
        "          << FormatCsvNumber(record.cycle_time_ms) << ','\n",
        "cycle csv profiler row",
    )

    text = replace_once(
        text,
        "    std::vector<double> publish_command_times;\n"
        "    std::vector<double> realtime_ratios;\n",
        "    std::vector<double> publish_command_times;\n"
        "    std::vector<double> solver_dam_calc_times;\n"
        "    std::vector<double> solver_dam_calcdiff_times;\n"
        "    std::vector<double> solver_dynamics_calc_times;\n"
        "    std::vector<double> solver_dynamics_calcdiff_times;\n"
        "    std::vector<double> solver_cost_sum_calc_times;\n"
        "    std::vector<double> solver_cost_sum_calcdiff_times;\n"
        "    std::vector<double> solver_cost_item_total_times;\n"
        "    std::vector<double> solver_collision_cost_total_times;\n"
        "    std::vector<double> solver_collision_residual_total_times;\n"
        "    std::vector<double> solver_model_total_times;\n"
        "    std::vector<double> solver_overhead_times;\n"
        "    std::vector<double> initial_model_total_times;\n"
        "    std::vector<double> realtime_ratios;\n",
        "summary profiler vectors",
    )

    text = replace_once(
        text,
        "    publish_command_times.reserve(cycle_records_.size());\n"
        "    realtime_ratios.reserve(cycle_records_.size());\n",
        "    publish_command_times.reserve(cycle_records_.size());\n"
        "    solver_dam_calc_times.reserve(cycle_records_.size());\n"
        "    solver_dam_calcdiff_times.reserve(cycle_records_.size());\n"
        "    solver_dynamics_calc_times.reserve(cycle_records_.size());\n"
        "    solver_dynamics_calcdiff_times.reserve(cycle_records_.size());\n"
        "    solver_cost_sum_calc_times.reserve(cycle_records_.size());\n"
        "    solver_cost_sum_calcdiff_times.reserve(cycle_records_.size());\n"
        "    solver_cost_item_total_times.reserve(cycle_records_.size());\n"
        "    solver_collision_cost_total_times.reserve(cycle_records_.size());\n"
        "    solver_collision_residual_total_times.reserve(cycle_records_.size());\n"
        "    solver_model_total_times.reserve(cycle_records_.size());\n"
        "    solver_overhead_times.reserve(cycle_records_.size());\n"
        "    initial_model_total_times.reserve(cycle_records_.size());\n"
        "    realtime_ratios.reserve(cycle_records_.size());\n",
        "summary profiler reserves",
    )

    text = replace_once(
        text,
        "      publish_command_times.push_back(record.publish_command_ms);\n"
        "      realtime_ratios.push_back(record.realtime_ratio);\n",
        "      publish_command_times.push_back(record.publish_command_ms);\n"
        "      solver_dam_calc_times.push_back(record.solver_profile.dam_calc_ms);\n"
        "      solver_dam_calcdiff_times.push_back(record.solver_profile.dam_calcdiff_ms);\n"
        "      solver_dynamics_calc_times.push_back(record.solver_profile.dynamics_calc_ms);\n"
        "      solver_dynamics_calcdiff_times.push_back(record.solver_profile.dynamics_calcdiff_ms);\n"
        "      solver_cost_sum_calc_times.push_back(record.solver_profile.cost_sum_calc_ms);\n"
        "      solver_cost_sum_calcdiff_times.push_back(record.solver_profile.cost_sum_calcdiff_ms);\n"
        "      solver_cost_item_total_times.push_back(RuntimeCostItemTotalMs(record.solver_profile));\n"
        "      solver_collision_cost_total_times.push_back(\n"
        "          RuntimeCostItemTotalMs(record.solver_profile, ga_ocp::RuntimeCostCategory::kCollision));\n"
        "      solver_collision_residual_total_times.push_back(\n"
        "          record.solver_profile.collision_residual_calc_ms +\n"
        "          record.solver_profile.collision_residual_calcdiff_ms);\n"
        "      const double model_total_ms = ga_ocp::RuntimeProfilerModelTimeMs(record.solver_profile);\n"
        "      solver_model_total_times.push_back(model_total_ms);\n"
        "      solver_overhead_times.push_back(record.solve_time_ms - model_total_ms);\n"
        "      initial_model_total_times.push_back(\n"
        "          ga_ocp::RuntimeProfilerModelTimeMs(record.initial_profile));\n"
        "      realtime_ratios.push_back(record.realtime_ratio);\n",
        "summary profiler push",
    )

    text = replace_once(
        text,
        "           \"publish_command_mean_ms,publish_command_p95_ms,\"\n"
        "           \"realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n",
        "           \"publish_command_mean_ms,publish_command_p95_ms,\"\n"
        "           \"solver_dam_calc_mean_ms,solver_dam_calc_p95_ms,\"\n"
        "           \"solver_dam_calcdiff_mean_ms,solver_dam_calcdiff_p95_ms,\"\n"
        "           \"solver_dynamics_calc_mean_ms,solver_dynamics_calc_p95_ms,\"\n"
        "           \"solver_dynamics_calcdiff_mean_ms,solver_dynamics_calcdiff_p95_ms,\"\n"
        "           \"solver_cost_sum_calc_mean_ms,solver_cost_sum_calc_p95_ms,\"\n"
        "           \"solver_cost_sum_calcdiff_mean_ms,solver_cost_sum_calcdiff_p95_ms,\"\n"
        "           \"solver_cost_item_total_mean_ms,solver_cost_item_total_p95_ms,\"\n"
        "           \"solver_collision_cost_total_mean_ms,solver_collision_cost_total_p95_ms,\"\n"
        "           \"solver_collision_residual_total_mean_ms,solver_collision_residual_total_p95_ms,\"\n"
        "           \"solver_model_total_mean_ms,solver_model_total_p95_ms,\"\n"
        "           \"solver_overhead_mean_ms,solver_overhead_p95_ms,\"\n"
        "           \"initial_model_total_mean_ms,initial_model_total_p95_ms,\"\n"
        "           \"enable_collision_cost,collision_obstacle_count,collision_weight,collision_safety_distance,\"\n"
        "           \"realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n",
        "summary csv profiler header",
    )

    text = replace_once(
        text,
        "        << FormatCsvNumber(Mean(publish_command_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(publish_command_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(realtime_ratios)) << ','\n",
        "        << FormatCsvNumber(Mean(publish_command_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(publish_command_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_dam_calc_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_dam_calc_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_dam_calcdiff_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_dam_calcdiff_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_dynamics_calc_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_dynamics_calc_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_dynamics_calcdiff_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_dynamics_calcdiff_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_cost_sum_calc_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_cost_sum_calc_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_cost_sum_calcdiff_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_cost_sum_calcdiff_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_cost_item_total_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_cost_item_total_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_collision_cost_total_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_collision_cost_total_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_collision_residual_total_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_collision_residual_total_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_model_total_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_model_total_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(solver_overhead_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(solver_overhead_times, 0.95)) << ','\n"
        "        << FormatCsvNumber(Mean(initial_model_total_times)) << ','\n"
        "        << FormatCsvNumber(Percentile(initial_model_total_times, 0.95)) << ','\n"
        "        << (enable_collision_cost_ ? 1 : 0) << ','\n"
        "        << collision_obstacle_count_ << ','\n"
        "        << FormatCsvNumber(collision_weight_) << ','\n"
        "        << FormatCsvNumber(collision_safety_distance_) << ','\n"
        "        << FormatCsvNumber(Mean(realtime_ratios)) << ','\n",
        "summary csv profiler row",
    )

    text = replace_once(
        text,
        "  double velocity_limit_scale_{0.9};\n"
        "\n"
        "  double reference_frequency_hz_{0.12};\n",
        "  double velocity_limit_scale_{0.9};\n"
        "  bool enable_runtime_profiler_{true};\n"
        "  bool enable_collision_cost_{false};\n"
        "  double collision_weight_{50.0};\n"
        "  double collision_safety_distance_{0.08};\n"
        "  int collision_obstacle_count_{4};\n"
        "\n"
        "  double reference_frequency_hz_{0.12};\n",
        "runtime member fields",
    )

    NODE.write_text(text, encoding="utf-8")


def patch_launch() -> None:
    text = LAUNCH.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "    output_prefix = LaunchConfiguration('output_prefix')\n",
        "    output_prefix = LaunchConfiguration('output_prefix')\n"
        "    enable_collision_cost = LaunchConfiguration('enable_collision_cost')\n"
        "    collision_obstacle_count = LaunchConfiguration('collision_obstacle_count')\n"
        "    collision_weight = LaunchConfiguration('collision_weight')\n"
        "    collision_safety_distance = LaunchConfiguration('collision_safety_distance')\n",
        "launch configs",
    )
    text = replace_once(
        text,
        "                'output_prefix': output_prefix,\n",
        "                'output_prefix': output_prefix,\n"
        "                'enable_collision_cost': ParameterValue(enable_collision_cost, value_type=bool),\n"
        "                'collision_obstacle_count': ParameterValue(collision_obstacle_count, value_type=int),\n"
        "                'collision_weight': ParameterValue(collision_weight, value_type=float),\n"
        "                'collision_safety_distance': ParameterValue(collision_safety_distance, value_type=float),\n",
        "launch node params",
    )
    text = replace_once(
        text,
        "        DeclareLaunchArgument('output_prefix', default_value=''),\n",
        "        DeclareLaunchArgument('output_prefix', default_value=''),\n"
        "        DeclareLaunchArgument('enable_collision_cost', default_value='false'),\n"
        "        DeclareLaunchArgument('collision_obstacle_count', default_value='4'),\n"
        "        DeclareLaunchArgument('collision_weight', default_value='50.0'),\n"
        "        DeclareLaunchArgument('collision_safety_distance', default_value='0.08'),\n",
        "launch args",
    )
    LAUNCH.write_text(text, encoding="utf-8")


def main() -> None:
    write_profiler_header()
    patch_actions()
    patch_residuals()
    patch_node()
    patch_launch()
    print("Applied GA-OCP solver-internal runtime profiling instrumentation.")


if __name__ == "__main__":
    main()
