#include <benchmark/benchmark.h>

#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/inertia.hpp>

#include "TetraPGA/BenchUtils.hpp"
#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/Motor.hpp"
#include "TetraPGA/Models.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace TetraPGA;
using namespace TetraPGA::bench;

namespace {

constexpr int kSampleBatchSize = 1024;
constexpr int kBenchmarkIterations = 1024000;
constexpr std::uint32_t kBenchmarkSeed = 20260421u;
constexpr double kPositionJitterScale = 0.2;
constexpr double kVelocitySampleScale = 0.5;
constexpr double kAccelerationSampleScale = 0.5;

struct IDSampleBatch {
  std::vector<Eigen::VectorXd> q;
  std::vector<Eigen::VectorXd> dq;
  std::vector<Eigen::VectorXd> ddq;
  std::size_t cursor{0};
};

double Clamp(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(value, upper));
}

Model<double> LoadTetraUr10Model() {
  Model<double> model = ur();
  model.gravity.setZero();
  model.gravity(5) = -9.81;
  return model;
}

pinocchio::Model LoadPinocchioUr10Model() {
  pinocchio::Model model;
  model.gravity.linear() << 0.0, 0.0, -9.81;
  model.gravity.angular().setZero();

  auto motorToSE3 = [](const Motor3D<double>& motor) {
    Eigen::Quaterniond rotation(motor[0], motor[1], motor[2], motor[3]);
    rotation.normalize();
    const Eigen::Quaterniond dual(motor[7], motor[4], motor[5], motor[6]);
    const Eigen::Quaterniond translation_quat = dual * rotation.conjugate();
    const Eigen::Vector3d translation =
        2.0 * Eigen::Vector3d(translation_quat.x(), translation_quat.y(), translation_quat.z());
    return pinocchio::SE3(rotation.toRotationMatrix(), translation);
  };

  const Model<double> tetra_model = LoadTetraUr10Model();
  std::array<pinocchio::SE3, 6> placements{
      motorToSE3(tetra_model.M0[1]),
      motorToSE3(tetra_model.Mj[2]),
      motorToSE3(tetra_model.Mj[3]),
      motorToSE3(tetra_model.Mj[4]),
      motorToSE3(tetra_model.Mj[5]),
      motorToSE3(tetra_model.Mj[6]),
  };
  const std::array<double, 6> masses{7.1, 12.7, 4.27, 2.0, 2.0, 0.365};
  const std::array<Eigen::Vector3d, 6> coms{
      Eigen::Vector3d(0.021, -0.027, 0.000),
      Eigen::Vector3d(-0.232, 0.000, 0.158),
      Eigen::Vector3d(-0.3323, 0.000, 0.068),
      Eigen::Vector3d(0.000, -0.018, 0.007),
      Eigen::Vector3d(0.000, 0.018, -0.007),
      Eigen::Vector3d(0.000, 0.000, -0.026),
  };
  const std::array<Eigen::Matrix3d, 6> inertias{
      (Eigen::Matrix3d() << 0.03408, 0.00425, 0.00002,
                            0.00425, 0.02156, -0.00008,
                            0.00002, -0.00008, 0.03529).finished(),
      (Eigen::Matrix3d() << 0.02814, 0.00005, -0.01561,
                            0.00005, 0.77068, 0.00002,
                            -0.01561, 0.00002, 0.76943).finished(),
      (Eigen::Matrix3d() << 0.01014, 0.00008, 0.00916,
                            0.00008, 0.30928, 0.00000,
                            0.00916, 0.00000, 0.30646).finished(),
      (Eigen::Matrix3d() << 0.00296, 0.00000, -0.00001,
                            0.00000, 0.00258, 0.00024,
                            -0.00001, 0.00024, 0.00222).finished(),
      (Eigen::Matrix3d() << 0.00296, 0.00000, 0.00001,
                            0.00000, 0.00258, 0.00024,
                            0.00001, 0.00024, 0.00222).finished(),
      (Eigen::Matrix3d() << 0.00040, 0.00000, 0.00000,
                            0.00000, 0.00041, 0.00000,
                            0.00000, 0.00000, 0.00034).finished(),
  };

  pinocchio::JointIndex parent_joint = 0;
  for (std::size_t i = 0; i < placements.size(); ++i) {
    const pinocchio::JointIndex joint_id =
        model.addJoint(parent_joint, pinocchio::JointModelRZ(), placements[i],
                       "ur10_joint_" + std::to_string(i + 1));
    model.appendBodyToJoint(
        joint_id,
        pinocchio::Inertia(masses[i], coms[i], inertias[i]),
        pinocchio::SE3::Identity());
    parent_joint = joint_id;
  }

  return model;
}

void ValidateUr10DofCompatibility(const Model<double>& tetra_model,
                                  const pinocchio::Model& pin_model) {
  if (pin_model.nq != pin_model.nv) {
    throw std::runtime_error("UR10 benchmark expects nq == nv.");
  }
  if (pin_model.nv != tetra_model.dof_a) {
    throw std::runtime_error("Pinocchio/TetraPGA DOF mismatch in UR10 dynamics benchmark.");
  }
}

IDSampleBatch MakeUr10IDSamples(const Model<double>& model,
                                const std::size_t sample_count,
                                const std::uint32_t seed) {
  IDSampleBatch batch;
  batch.q.reserve(sample_count);
  batch.dq.reserve(sample_count);
  batch.ddq.reserve(sample_count);

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unif11(-1.0, 1.0);

  for (std::size_t sample_idx = 0; sample_idx < sample_count; ++sample_idx) {
    Eigen::VectorXd q(model.dof_a);
    Eigen::VectorXd dq(model.dof_a);
    Eigen::VectorXd ddq(model.dof_a);

    for (int i = 0; i < model.dof_a; ++i) {
      const double q_center = model.qa0[i];
      const double q_lower = model.lowerPositionLimit[i];
      const double q_upper = model.upperPositionLimit[i];
      const double q_half_span = 0.5 * std::max(0.0, q_upper - q_lower);
      const double q_jitter = kPositionJitterScale * q_half_span * unif11(rng);
      q[i] = Clamp(q_center + q_jitter, q_lower, q_upper);

      const double velocity_limit = model.velocityLimit[i];
      const double dq_limit =
          std::isfinite(velocity_limit) ? std::max(velocity_limit, 1e-3) : 1.0;
      dq[i] = kVelocitySampleScale * dq_limit * unif11(rng);
      ddq[i] = kAccelerationSampleScale * dq_limit * unif11(rng);
    }

    batch.q.push_back(std::move(q));
    batch.dq.push_back(std::move(dq));
    batch.ddq.push_back(std::move(ddq));
  }

  return batch;
}

void RunTetraPGAInverseDynamicsBenchmark(benchmark::State& state) {
  Model<double> tetra_model = LoadTetraUr10Model();
  Data<double> tetra_data(tetra_model);
  IDSampleBatch samples =
      MakeUr10IDSamples(tetra_model, kSampleBatchSize, kBenchmarkSeed ^ 0x1D01u);

  state.counters["DOF"] = static_cast<double>(tetra_model.dof_a);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& dq = samples.dq[i];
    const Eigen::VectorXd& ddq = samples.ddq[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(dq);
    benchmark::DoNotOptimize(ddq);

    const Eigen::VectorXd& tau = inverseDynamics(tetra_model, tetra_data, q, dq, ddq);

    benchmark::DoNotOptimize(tau);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioRNEABenchmark(benchmark::State& state) {
  Model<double> tetra_model = LoadTetraUr10Model();
  // Keep Pinocchio model/data alive for process lifetime; destructor teardown is unstable here.
  auto* pin_model = new pinocchio::Model(LoadPinocchioUr10Model());
  ValidateUr10DofCompatibility(tetra_model, *pin_model);
  auto* pin_data = new pinocchio::Data(*pin_model);
  IDSampleBatch samples =
      MakeUr10IDSamples(tetra_model, kSampleBatchSize, kBenchmarkSeed ^ 0x1D01u);

  state.counters["DOF"] = static_cast<double>(tetra_model.dof_a);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& dq = samples.dq[i];
    const Eigen::VectorXd& ddq = samples.ddq[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(dq);
    benchmark::DoNotOptimize(ddq);

    const Eigen::VectorXd& tau = pinocchio::rnea(*pin_model, *pin_data, q, dq, ddq);

    benchmark::DoNotOptimize(tau);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunTetraPGAInverseDynamicsDerivativesBenchmark(benchmark::State& state) {
  Model<double> tetra_model = LoadTetraUr10Model();
  Data<double> tetra_data(tetra_model);
  IDSampleBatch samples =
      MakeUr10IDSamples(tetra_model, kSampleBatchSize, kBenchmarkSeed ^ 0x1D0Fu);

  state.counters["DOF"] = static_cast<double>(tetra_model.dof_a);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& dq = samples.dq[i];
    const Eigen::VectorXd& ddq = samples.ddq[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(dq);
    benchmark::DoNotOptimize(ddq);

    inverseDynamics_fo(tetra_model, tetra_data, q, dq, ddq);

    benchmark::DoNotOptimize(tetra_data.tau);
    benchmark::DoNotOptimize(tetra_data.ptau_pq);
    benchmark::DoNotOptimize(tetra_data.ptau_pdq);
    benchmark::DoNotOptimize(tetra_data.ptau_pddq);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioRNEADerivativesBenchmark(benchmark::State& state) {
  Model<double> tetra_model = LoadTetraUr10Model();
  // Keep Pinocchio model/data alive for process lifetime; destructor teardown is unstable here.
  auto* pin_model = new pinocchio::Model(LoadPinocchioUr10Model());
  ValidateUr10DofCompatibility(tetra_model, *pin_model);
  auto* pin_data = new pinocchio::Data(*pin_model);
  IDSampleBatch samples =
      MakeUr10IDSamples(tetra_model, kSampleBatchSize, kBenchmarkSeed ^ 0x1D0Fu);

  state.counters["DOF"] = static_cast<double>(tetra_model.dof_a);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& dq = samples.dq[i];
    const Eigen::VectorXd& ddq = samples.ddq[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(dq);
    benchmark::DoNotOptimize(ddq);

    pinocchio::computeRNEADerivatives(*pin_model, *pin_data, q, dq, ddq);

    benchmark::DoNotOptimize(pin_data->tau);
    benchmark::DoNotOptimize(pin_data->dtau_dq);
    benchmark::DoNotOptimize(pin_data->dtau_dv);
    benchmark::DoNotOptimize(pin_data->M);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RegisterAll() {
  benchmark::RegisterBenchmark("ur10/TetraPGA/InverseDynamics",
                               RunTetraPGAInverseDynamicsBenchmark)
      ->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark("ur10/Pinocchio/rnea",
                               RunPinocchioRNEABenchmark)
      ->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark("ur10/TetraPGA/InverseDynamicsDerivatives",
                               RunTetraPGAInverseDynamicsDerivativesBenchmark)
      ->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark("ur10/Pinocchio/computeRNEADerivatives",
                               RunPinocchioRNEADerivativesBenchmark)
      ->Iterations(kBenchmarkIterations);
}

}  // namespace

int main(int argc, char** argv) {
  RegisterAll();
  auto benchmark_args = PrepareBenchmarkCsvArgs(argc, argv, "TetraPGA_ur10_dyn_bench");
  int benchmark_argc = benchmark_args.argc();
  char** benchmark_argv = benchmark_args.data();
  benchmark::Initialize(&benchmark_argc, benchmark_argv);
  benchmark::AddCustomContext("Robot", "ur10");
  benchmark::AddCustomContext("SourceModel", "tetrapga_repo_model_vs_pinocchio_manual_model");
  benchmark::AddCustomContext("FixedIterations", std::to_string(kBenchmarkIterations));
  benchmark::AddCustomContext("SampleBatch", std::to_string(kSampleBatchSize));
  benchmark::AddCustomContext("Seed", std::to_string(kBenchmarkSeed));
  benchmark::AddCustomContext("CSVOutput", benchmark_args.csv_path);
  benchmark::ConsoleReporter console_reporter;
  PivotCsvReporter csv_reporter(
      benchmark_args.csv_path,
      PivotCsvReporterConfig{PivotMetricSource::kCpuTimeMs, "", "case", "DOF"});
  CombinedReporter combined_reporter(&console_reporter, &csv_reporter);
  benchmark::RunSpecifiedBenchmarks(&combined_reporter);
  benchmark::Shutdown();
  return 0;
}
