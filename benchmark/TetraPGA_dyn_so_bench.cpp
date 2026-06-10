#include <benchmark/benchmark.h>

#include <pinocchio/algorithm/rnea-second-order-derivatives.hpp>

#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/BenchUtils.hpp"

using namespace TetraPGA;
using namespace TetraPGA::bench;

namespace {

constexpr int kMinLevel = 1;
constexpr int kMaxLevel = 5;
constexpr int kSampleBatchSize = 1024;
constexpr int kBenchmarkIterations = 1024;

void RunTetraPGAInverseDynamicsSecondOrderDerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  Model<double> model = MakeGABenchModel(
      params, "tetrapga_nlink_inverse_dynamics_second_order_derivatives_benchmark");
  Data<double> data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0x2D0Fu, static_cast<std::uint32_t>(bf),
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

    inverseDynamics_so(model, data, q, dq, ddq);

    benchmark::DoNotOptimize(data.p2tau_pqpq);
    benchmark::DoNotOptimize(data.p2tau_pdqpq);
    benchmark::DoNotOptimize(data.p2tau_pdqpdq);
    benchmark::DoNotOptimize(data.p2tau_pqpddq);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioRNEASecondOrderDerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  pinocchio::Model model = BuildPinModel(params);
  pinocchio::Data data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0x2D0Fu, static_cast<std::uint32_t>(bf),
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

    pinocchio::ComputeRNEASecondOrderDerivatives(model, data, q, dq, ddq);

    benchmark::DoNotOptimize(data.d2tau_dqdq);
    benchmark::DoNotOptimize(data.d2tau_dqdv);
    benchmark::DoNotOptimize(data.d2tau_dvdv);
    benchmark::DoNotOptimize(data.d2tau_dadq);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RegisterAll() {
  benchmark::RegisterBenchmark("binary_tree/TetraPGA/InverseDynamicsSecondOrderDerivatives",
                               [](benchmark::State& s) {
                                 RunTetraPGAInverseDynamicsSecondOrderDerivatives(s, 2);
                               })
      ->DenseRange(kMinLevel, kMaxLevel, 1)
      ->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark("binary_tree/Pinocchio/ComputeRNEASecondOrderDerivatives",
                               [](benchmark::State& s) {
                                 RunPinocchioRNEASecondOrderDerivatives(s, 2);
                               })
      ->DenseRange(kMinLevel, kMaxLevel, 1)
      ->Iterations(kBenchmarkIterations);
}

}  // namespace

int main(int argc, char** argv) {
  RegisterAll();
  auto benchmark_args = PrepareBenchmarkCsvArgs(argc, argv, "TetraPGA_dyn_so_bench");
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
