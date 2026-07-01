#!/usr/bin/env python3
"""Avoid BenchUtils include conflicts by localizing the closed-loop CasADi action."""

from __future__ import annotations

from pathlib import Path


PATH = Path("/home/chenwh/ros2_ws/src/GA-OCP/ga_ocp_ros2/src/closed_loop_mpc_node.cpp")


LOCAL_CASADI_CLASSES = r'''
#ifdef GA_OCP_HAS_CASADI_BENCH
class InlineAutoDiffABADerivatives
    : public pinocchio::casadi::AutoDiffABADerivatives<double> {
 public:
  InlineAutoDiffABADerivatives(const pinocchio::Model& model, const std::string& tag)
      : pinocchio::casadi::AutoDiffABADerivatives<double>(model, tag, tag + "_lib",
                                                          tag + "_eval") {
    this->buildMap();
    this->fun = this->ad_fun;
  }
};

struct DifferentialActionDataPinocchioCasadi
    : public crocoddyl::DifferentialActionDataAbstractTpl<double>,
      public crocoddyl::DataCollectorAbstractTpl<double> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DifferentialActionDataPinocchioCasadi(
      crocoddyl::DifferentialActionModelAbstractTpl<double>* const model,
      const pinocchio::Model& pin_model,
      const std::shared_ptr<crocoddyl::CostModelSumTpl<double>>& costs_model)
      : crocoddyl::DifferentialActionDataAbstractTpl<double>(model),
        pinocchio(pin_model) {
    if (costs_model) {
      costs = costs_model->createData(this);
    }
  }

  pinocchio::Data pinocchio;
  std::shared_ptr<crocoddyl::CostDataSumTpl<double>> costs;
};

class DifferentialActionModelPinocchioCasadi
    : public crocoddyl::DifferentialActionModelAbstractTpl<double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DifferentialActionModelPinocchioCasadi(
      std::shared_ptr<crocoddyl::StateMultibody> state,
      const pinocchio::Model& pin_model,
      std::shared_ptr<crocoddyl::CostModelSumTpl<double>> costs,
      std::shared_ptr<InlineAutoDiffABADerivatives> autodiff)
      : crocoddyl::DifferentialActionModelAbstractTpl<double>(state, pin_model.nv),
        state_(std::move(state)),
        pin_model_(pin_model),
        costs_(std::move(costs)),
        autodiff_(std::move(autodiff)) {}

  void calc(const std::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
            const Eigen::Ref<const typename crocoddyl::MathBaseTpl<double>::VectorXs>& x,
            const Eigen::Ref<const typename crocoddyl::MathBaseTpl<double>::VectorXs>& u) override {
    auto* d = static_cast<DifferentialActionDataPinocchioCasadi*>(data.get());
    const std::size_t nq = state_->get_nq();
    const std::size_t nv = state_->get_nv();

    d->xout = pinocchio::aba(pin_model_, d->pinocchio, x.head(nq), x.segment(nq, nv), u);

    if (costs_) {
      costs_->calc(d->costs, x, u);
      d->cost = d->costs->cost;
    } else {
      d->cost = 0.;
    }
  }

  void calcDiff(const std::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
                const Eigen::Ref<const typename crocoddyl::MathBaseTpl<double>::VectorXs>& x,
                const Eigen::Ref<const typename crocoddyl::MathBaseTpl<double>::VectorXs>& u) override {
    auto* d = static_cast<DifferentialActionDataPinocchioCasadi*>(data.get());
    const std::size_t nq = state_->get_nq();
    const std::size_t nv = state_->get_nv();
    const Eigen::VectorXd q = x.head(nq);
    const Eigen::VectorXd v = x.segment(nq, nv);
    const Eigen::VectorXd tau = u;

    autodiff_->evalFunction(q, v, tau);

    d->xout = autodiff_->ddq;
    d->Fx.leftCols(nv) = autodiff_->ddq_dq;
    d->Fx.rightCols(nv) = autodiff_->ddq_dv;
    d->Fu = autodiff_->ddq_dtau;

    if (costs_) {
      costs_->calcDiff(d->costs, x, u);
      d->Lx = d->costs->Lx;
      d->Lu = d->costs->Lu;
      d->Lxx = d->costs->Lxx;
      d->Lxu = d->costs->Lxu;
      d->Luu = d->costs->Luu;
    }
  }

  std::shared_ptr<crocoddyl::DifferentialActionDataAbstract> createData() override {
    return std::make_shared<DifferentialActionDataPinocchioCasadi>(this, pin_model_, costs_);
  }

  std::shared_ptr<crocoddyl::DifferentialActionModelBase> cloneAsDouble() const override {
    throw std::runtime_error(
        "cloneAsDouble not implemented for DifferentialActionModelPinocchioCasadi");
  }

  std::shared_ptr<crocoddyl::DifferentialActionModelBase> cloneAsFloat() const override {
    throw std::runtime_error(
        "cloneAsFloat not implemented for DifferentialActionModelPinocchioCasadi");
  }

 private:
  std::shared_ptr<crocoddyl::StateMultibody> state_;
  pinocchio::Model pin_model_;
  std::shared_ptr<crocoddyl::CostModelSumTpl<double>> costs_;
  std::shared_ptr<InlineAutoDiffABADerivatives> autodiff_;
};
#endif

'''


def replace_once(text: str, old: str, new: str) -> str:
    if old not in text:
        raise RuntimeError(f"pattern not found: {old[:80]!r}")
    return text.replace(old, new, 1)


def main() -> int:
    text = PATH.read_text()
    text = replace_once(
        text,
        '''#ifdef GA_OCP_HAS_CASADI_BENCH
#include "ga_ocp/BenchUtils.hpp"
#endif
''',
        '''#ifdef GA_OCP_HAS_CASADI_BENCH
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/autodiff/casadi-algo.hpp>
#endif
''',
    )
    if "class DifferentialActionModelPinocchioCasadi" not in text:
        text = replace_once(
            text,
            '''std::string BackendName(const BackendKind backend) {
  switch (backend) {
    case BackendKind::kTetraPGA:
      return "tetrapga";
    case BackendKind::kPinocchio:
      return "pinocchio";
    case BackendKind::kCasadi:
      return "casadi";
  }
  return "unknown";
}

''',
            '''std::string BackendName(const BackendKind backend) {
  switch (backend) {
    case BackendKind::kTetraPGA:
      return "tetrapga";
    case BackendKind::kPinocchio:
      return "pinocchio";
    case BackendKind::kCasadi:
      return "casadi";
  }
  return "unknown";
}

''' + LOCAL_CASADI_CLASSES,
        )
    PATH.write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
