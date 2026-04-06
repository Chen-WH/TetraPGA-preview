#include <benchmark/benchmark.h>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/BenchUtils.hpp"

using namespace TetraPGA;
using namespace TetraPGA::bench;

namespace {

constexpr int kMinLevel = 1;
constexpr int kMaxLevel = 9;
constexpr int kSampleBatchSize = 1024;
constexpr int kBenchmarkIterations = 1024;

void RunTetraPGAForwardDynamics(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  Model<double> model = MakeGABenchModel(params, "tetrapga_nlink_forward_dynamics_benchmark");
  Data<double> data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0xF001u, static_cast<std::uint32_t>(bf),
                       static_cast<std::uint32_t>(dof));
  FDSampleBatch samples = MakeFDSamples(dof, kSampleBatchSize, seed);

  state.counters["DOF"] = static_cast<double>(dof);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& v = samples.v[i];
    const Eigen::VectorXd& tau = samples.tau[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(v);
    benchmark::DoNotOptimize(tau);

    const Eigen::VectorXd& ddq = forwardDynamics0(model, data, q, v, tau);

    benchmark::DoNotOptimize(ddq);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioABA(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  pinocchio::Model model = BuildPinModel(params);
  pinocchio::Data data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0xF001u, static_cast<std::uint32_t>(bf),
                       static_cast<std::uint32_t>(dof));
  FDSampleBatch samples = MakeFDSamples(dof, kSampleBatchSize, seed);

  state.counters["DOF"] = static_cast<double>(dof);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& v = samples.v[i];
    const Eigen::VectorXd& tau = samples.tau[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(v);
    benchmark::DoNotOptimize(tau);

    const Eigen::VectorXd& ddq = pinocchio::aba(model, data, q, v, tau);

    benchmark::DoNotOptimize(ddq);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunTetraPGAInverseDynamics(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  Model<double> model = MakeGABenchModel(params, "tetrapga_nlink_inverse_dynamics_benchmark");
  Data<double> data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0x1D01u, static_cast<std::uint32_t>(bf),
                       static_cast<std::uint32_t>(dof));
  IDSampleBatch samples = MakeIDSamples(dof, kSampleBatchSize, seed);

  state.counters["DOF"] = static_cast<double>(dof);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& dq = samples.dq[i];
    const Eigen::VectorXd& ddq = samples.ddq[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(dq);
    benchmark::DoNotOptimize(ddq);

    const Eigen::VectorXd& tau = inverseDynamics(model, data, q, dq, ddq);

    benchmark::DoNotOptimize(tau);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioRNEA(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  pinocchio::Model model = BuildPinModel(params);
  pinocchio::Data data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0x1D01u, static_cast<std::uint32_t>(bf),
                       static_cast<std::uint32_t>(dof));
  IDSampleBatch samples = MakeIDSamples(dof, kSampleBatchSize, seed);

  state.counters["DOF"] = static_cast<double>(dof);

  for (auto _ : state) {
    const std::size_t i = samples.cursor;
    const Eigen::VectorXd& q = samples.q[i];
    const Eigen::VectorXd& dq = samples.dq[i];
    const Eigen::VectorXd& ddq = samples.ddq[i];

    benchmark::DoNotOptimize(q);
    benchmark::DoNotOptimize(dq);
    benchmark::DoNotOptimize(ddq);

    const Eigen::VectorXd& tau = pinocchio::rnea(model, data, q, dq, ddq);

    benchmark::DoNotOptimize(tau);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RegisterAll() {
  benchmark::RegisterBenchmark("binary_tree/TetraPGA/ForwardDynamics", [](benchmark::State& s) {
    RunTetraPGAForwardDynamics(s, 2);
  })->DenseRange(kMinLevel, kMaxLevel, 1)->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark("binary_tree/Pinocchio/aba", [](benchmark::State& s) {
    RunPinocchioABA(s, 2);
  })->DenseRange(kMinLevel, kMaxLevel, 1)->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark("binary_tree/TetraPGA/InverseDynamics", [](benchmark::State& s) {
    RunTetraPGAInverseDynamics(s, 2);
  })->DenseRange(kMinLevel, kMaxLevel, 1)->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark("binary_tree/Pinocchio/rnea", [](benchmark::State& s) {
    RunPinocchioRNEA(s, 2);
  })->DenseRange(kMinLevel, kMaxLevel, 1)->Iterations(kBenchmarkIterations);
}

}  // namespace

int main(int argc, char** argv) {
  RegisterAll();
  auto benchmark_args = PrepareBenchmarkCsvArgs(argc, argv, "TetraPGA_dyn_bench");
  int benchmark_argc = benchmark_args.argc();
  char** benchmark_argv = benchmark_args.data();
  benchmark::Initialize(&benchmark_argc, benchmark_argv);
  benchmark::AddCustomContext("FixedIterations", std::to_string(kBenchmarkIterations));
  benchmark::AddCustomContext("SampleBatch", std::to_string(kSampleBatchSize));
  benchmark::AddCustomContext("SeedPolicy", "per-run seed, per-case mixed");
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
