#include <benchmark/benchmark.h>

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/BenchUtils.hpp"

using namespace TetraPGA;
using namespace TetraPGA::bench;

namespace {

constexpr int kMinLevel = 1;
constexpr int kMaxLevel = 9;
constexpr int kMaxLevel_AD = 8;
constexpr int kSampleBatchSize = 1024;
constexpr int kBenchmarkIterations = 1024;

void RunTetraPGAForwardDynamicsDerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  Model<double> model = MakeGABenchModel(params, "tetrapga_nlink_forward_dynamics_derivatives_benchmark");
  Data<double> data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0xF0D1u, static_cast<std::uint32_t>(bf),
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

    forwardDynamics_fo(model, data, q, v, tau);

    benchmark::DoNotOptimize(data.ddq);
    benchmark::DoNotOptimize(data.pddq_pq);
    benchmark::DoNotOptimize(data.pddq_pdq);
    benchmark::DoNotOptimize(data.pddq_ptau);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioABADerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  pinocchio::Model model = BuildPinModel(params);
  pinocchio::Data data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0xF0D1u, static_cast<std::uint32_t>(bf),
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

    pinocchio::computeABADerivatives(model, data, q, v, tau);

    benchmark::DoNotOptimize(data.ddq);
    benchmark::DoNotOptimize(data.ddq_dq);
    benchmark::DoNotOptimize(data.ddq_dv);
    benchmark::DoNotOptimize(data.Minv);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

#ifdef GA4RO_HAS_CASADI_BENCH
void RunPinocchioCasadiABADerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  pinocchio::Model model = BuildPinModel(params);
  InlineAutoDiffABADerivatives autodiff(
      model, "tetrapga_pinocchio_casadi_aba_derivatives_bf" + std::to_string(bf) +
                 "_dof" + std::to_string(dof));
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0xCAD1u, static_cast<std::uint32_t>(bf),
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

    autodiff.evalFunction(q, v, tau);

    benchmark::DoNotOptimize(autodiff.ddq);
    benchmark::DoNotOptimize(autodiff.ddq_dq);
    benchmark::DoNotOptimize(autodiff.ddq_dv);
    benchmark::DoNotOptimize(autodiff.ddq_dtau);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioCasadiRNEADerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  pinocchio::Model model = BuildPinModel(params);
  InlineAutoDiffRNEADerivatives autodiff(
      model, "tetrapga_pinocchio_casadi_rnea_derivatives_bf" + std::to_string(bf) +
                 "_dof" + std::to_string(dof));
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0x1D0Fu, static_cast<std::uint32_t>(bf),
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

    autodiff.evalFunction(q, dq, ddq);

    benchmark::DoNotOptimize(autodiff.tau);
    benchmark::DoNotOptimize(autodiff.dtau_dq);
    benchmark::DoNotOptimize(autodiff.dtau_dv);
    benchmark::DoNotOptimize(autodiff.dtau_da);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}
#endif

void RunTetraPGAInverseDynamicsDerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  Model<double> model = MakeGABenchModel(params, "tetrapga_nlink_inverse_dynamics_derivatives_benchmark");
  Data<double> data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0x1D0Fu, static_cast<std::uint32_t>(bf),
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

    inverseDynamics_fo(model, data, q, dq, ddq);

    benchmark::DoNotOptimize(data.ptau_pq);
    benchmark::DoNotOptimize(data.ptau_pdq);
    benchmark::DoNotOptimize(data.ptau_pddq);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RunPinocchioRNEADerivatives(benchmark::State& state, int bf) {
  const int level = static_cast<int>(state.range(0));
  const int dof = DofFromLevel(level);

  const TreeTemplateParams params = MakeBenchTreeParams(bf, dof);
  pinocchio::Model model = BuildPinModel(params);
  pinocchio::Data data(model);
  const std::uint32_t seed =
      MixBenchmarkSeed(BenchmarkRunSeed(), 0x1D0Fu, static_cast<std::uint32_t>(bf),
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

    pinocchio::computeRNEADerivatives(model, data, q, dq, ddq);

    benchmark::DoNotOptimize(data.dtau_dq);
    benchmark::DoNotOptimize(data.dtau_dv);
    benchmark::DoNotOptimize(data.M);

    samples.cursor = (samples.cursor + 1) % samples.q.size();
  }
}

void RegisterTopology(const std::string& topology, int bf) {
  benchmark::RegisterBenchmark((topology + "/TetraPGA/ForwardDynamicsDerivatives").c_str(),
                               [bf](benchmark::State& s) {
                                 RunTetraPGAForwardDynamicsDerivatives(s, bf);
                               })
      ->DenseRange(kMinLevel, kMaxLevel, 1)
      ->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark((topology + "/Pinocchio/computeABADerivatives").c_str(),
                               [bf](benchmark::State& s) {
                                 RunPinocchioABADerivatives(s, bf);
                               })
      ->DenseRange(kMinLevel, kMaxLevel, 1)
      ->Iterations(kBenchmarkIterations);

#ifdef GA4RO_HAS_CASADI_BENCH
  benchmark::RegisterBenchmark((topology + "/CasADi/computeABADerivatives").c_str(),
                               [bf](benchmark::State& s) {
                                 RunPinocchioCasadiABADerivatives(s, bf);
                               })
      ->DenseRange(kMinLevel, kMaxLevel_AD, 1)
      ->Iterations(kBenchmarkIterations);
#endif

  benchmark::RegisterBenchmark((topology + "/TetraPGA/InverseDynamicsDerivatives").c_str(),
                               [bf](benchmark::State& s) {
                                 RunTetraPGAInverseDynamicsDerivatives(s, bf);
                               })
      ->DenseRange(kMinLevel, kMaxLevel, 1)
      ->Iterations(kBenchmarkIterations);

  benchmark::RegisterBenchmark((topology + "/Pinocchio/computeRNEADerivatives").c_str(),
                               [bf](benchmark::State& s) {
                                 RunPinocchioRNEADerivatives(s, bf);
                               })
      ->DenseRange(kMinLevel, kMaxLevel, 1)
      ->Iterations(kBenchmarkIterations);

#ifdef GA4RO_HAS_CASADI_BENCH
  benchmark::RegisterBenchmark((topology + "/CasADi/computeRNEADerivatives").c_str(),
                               [bf](benchmark::State& s) {
                                 RunPinocchioCasadiRNEADerivatives(s, bf);
                               })
      ->DenseRange(kMinLevel, kMaxLevel_AD, 1)
      ->Iterations(kBenchmarkIterations);
#endif
}

void RegisterAll() {
  RegisterTopology("serial_chain", 1);
  RegisterTopology("binary_tree", 2);
}

}  // namespace

int main(int argc, char** argv) {
  RegisterAll();
  auto benchmark_args = PrepareBenchmarkCsvArgs(argc, argv, "TetraPGA_dyn_fo_bench");
  int benchmark_argc = benchmark_args.argc();
  char** benchmark_argv = benchmark_args.data();
  benchmark::Initialize(&benchmark_argc, benchmark_argv);
  benchmark::AddCustomContext("FixedIterations", std::to_string(kBenchmarkIterations));
  benchmark::AddCustomContext("SampleBatch", std::to_string(kSampleBatchSize));
  benchmark::AddCustomContext("SeedPolicy", "per-run seed, per-case mixed");
  benchmark::AddCustomContext("Topologies", "serial_chain,binary_tree");
  benchmark::AddCustomContext("CSVOutput", benchmark_args.csv_path);
#ifdef GA4RO_HAS_CASADI_BENCH
  benchmark::AddCustomContext("CasADiCases", "compiled_in");
#else
  benchmark::AddCustomContext("CasADiCases", "not_built");
#endif
  benchmark::ConsoleReporter console_reporter;
  PivotCsvReporter csv_reporter(
      benchmark_args.csv_path,
      PivotCsvReporterConfig{PivotMetricSource::kCpuTimeMs, "", "case", "DOF"});
  CombinedReporter combined_reporter(&console_reporter, &csv_reporter);
  benchmark::RunSpecifiedBenchmarks(&combined_reporter);
  benchmark::Shutdown();
  return 0;
}
