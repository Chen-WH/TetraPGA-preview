#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "TetraPGA/BenchUtils.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using DurationSeconds = std::chrono::duration<double>;
using namespace TetraPGA;
using namespace TetraPGA::bench;

struct CliConfig {
  std::string case_name = "rnea_so";
  int dof = 31;
  int branching_factor = 2;
  bool eval = true;
  std::size_t horizon = 50;
  double dt = 0.02;
  unsigned int seed = 12345u;
};

struct ProbeResult {
  std::string case_name;
  int dof = 0;
  int branching_factor = 0;
  int eval = 0;
  std::size_t horizon = 0;
  double model_build_s = 0.0;
  double graph_build_s = 0.0;
  double problem_build_s = 0.0;
  double eval_s = 0.0;
  std::string status = "ok";
  std::string error;
};

bool HasPrefix(const std::string_view value, const std::string_view prefix) {
  return value.substr(0, prefix.size()) == prefix;
}

std::string ValueAfter(const std::string& arg, const std::string_view prefix) {
  return arg.substr(prefix.size());
}

bool ParseBool(const std::string& value) {
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::invalid_argument("expected boolean value, got: " + value);
}

CliConfig ParseCli(int argc, char** argv) {
  CliConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] == nullptr ? "" : argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: casadi_graph_probe --case=<case> --dof=<int> [options]\n"
          << "Cases:\n"
          << "  aba_fo    CasADi ABA first-order derivative graph\n"
          << "  rnea_fo   CasADi RNEA first-order derivative graph\n"
          << "  rnea_so   CasADi RNEA second-order derivative graph\n"
          << "  ocp_fd    CasADi ABA graph used by the forward-dynamics OCP backend\n"
          << "  ocp_id    CasADi RNEA graph used by the inverse-dynamics OCP backend\n"
          << "Options:\n"
          << "  --branching_factor=<int>  default: 2\n"
          << "  --eval=<0|1>              run one post-construction evaluation, default: 1\n"
          << "  --horizon=<int>           OCP horizon, default: 50\n"
          << "  --dt=<double>             OCP time step, default: 0.02\n"
          << "  --seed=<uint>             sample seed, default: 12345\n";
      std::exit(0);
    }
    if (HasPrefix(arg, "--case=")) {
      config.case_name = ValueAfter(arg, "--case=");
    } else if (HasPrefix(arg, "--dof=")) {
      config.dof = std::stoi(ValueAfter(arg, "--dof="));
    } else if (HasPrefix(arg, "--branching_factor=")) {
      config.branching_factor = std::stoi(ValueAfter(arg, "--branching_factor="));
    } else if (HasPrefix(arg, "--eval=")) {
      config.eval = ParseBool(ValueAfter(arg, "--eval="));
    } else if (HasPrefix(arg, "--horizon=")) {
      config.horizon = static_cast<std::size_t>(std::stoul(ValueAfter(arg, "--horizon=")));
    } else if (HasPrefix(arg, "--dt=")) {
      config.dt = std::stod(ValueAfter(arg, "--dt="));
    } else if (HasPrefix(arg, "--seed=")) {
      config.seed = static_cast<unsigned int>(std::stoul(ValueAfter(arg, "--seed=")));
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }
  if (config.dof <= 0) {
    throw std::invalid_argument("--dof must be positive");
  }
  if (config.branching_factor <= 0) {
    throw std::invalid_argument("--branching_factor must be positive");
  }
  if (config.horizon == 0) {
    throw std::invalid_argument("--horizon must be positive");
  }
  return config;
}

double ElapsedSeconds(const Clock::time_point start) {
  return DurationSeconds(Clock::now() - start).count();
}

std::string CsvEscape(const std::string_view value) {
  std::string out;
  out.reserve(value.size() + 2);
  out.push_back('"');
  for (const char c : value) {
    if (c == '"') {
      out.push_back('"');
    }
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

std::string FormatDouble(const double value) {
  std::ostringstream oss;
  oss << std::setprecision(12) << value;
  return oss.str();
}

void PrintCsvHeader() {
  std::cout << "case,dof,branching_factor,eval,horizon,model_build_s,graph_build_s,"
               "problem_build_s,eval_s,status,error\n";
}

void PrintCsvRow(const ProbeResult& result) {
  std::cout << CsvEscape(result.case_name) << ','
            << result.dof << ','
            << result.branching_factor << ','
            << result.eval << ','
            << result.horizon << ','
            << FormatDouble(result.model_build_s) << ','
            << FormatDouble(result.graph_build_s) << ','
            << FormatDouble(result.problem_build_s) << ','
            << FormatDouble(result.eval_s) << ','
            << CsvEscape(result.status) << ','
            << CsvEscape(result.error) << '\n';
}

void ProbeOk(const CliConfig& config, ProbeResult& result) {
  const auto model_start = Clock::now();
  const TreeTemplateParams params = MakeBenchTreeParams(config.branching_factor, config.dof);
  const pinocchio::Model pin_model = BuildPinModel(params);
  result.model_build_s = ElapsedSeconds(model_start);

  if (config.case_name == "aba_fo" || config.case_name == "ocp_fd") {
    const auto graph_start = Clock::now();
    InlineAutoDiffABADerivatives autodiff(
        pin_model, "reviewer_casadi_" + config.case_name + "_dof" + std::to_string(config.dof));
    result.graph_build_s = ElapsedSeconds(graph_start);

    if (config.eval) {
      const FDSampleBatch samples = MakeFDSamples(config.dof, 1, config.seed);
      const auto eval_start = Clock::now();
      autodiff.evalFunction(samples.q[0], samples.v[0], samples.tau[0]);
      result.eval_s = ElapsedSeconds(eval_start);
      benchmark::DoNotOptimize(autodiff.ddq);
      benchmark::DoNotOptimize(autodiff.ddq_dq);
      benchmark::DoNotOptimize(autodiff.ddq_dv);
      benchmark::DoNotOptimize(autodiff.ddq_dtau);
    }
    return;
  }

  if (config.case_name == "rnea_fo" || config.case_name == "ocp_id") {
    const auto graph_start = Clock::now();
    InlineAutoDiffRNEADerivatives autodiff(
        pin_model, "reviewer_casadi_" + config.case_name + "_dof" + std::to_string(config.dof));
    result.graph_build_s = ElapsedSeconds(graph_start);

    if (config.eval) {
      const IDSampleBatch samples = MakeIDSamples(config.dof, 1, config.seed);
      const auto eval_start = Clock::now();
      autodiff.evalFunction(samples.q[0], samples.dq[0], samples.ddq[0]);
      result.eval_s = ElapsedSeconds(eval_start);
      benchmark::DoNotOptimize(autodiff.tau);
      benchmark::DoNotOptimize(autodiff.dtau_dq);
      benchmark::DoNotOptimize(autodiff.dtau_dv);
      benchmark::DoNotOptimize(autodiff.dtau_da);
    }
    return;
  }

  if (config.case_name == "rnea_so") {
    const auto graph_start = Clock::now();
    TetraPGA::bench::InlineAutoDiffRNEASecondOrderDerivatives autodiff(
        pin_model, "reviewer_casadi_rnea_so_dof" + std::to_string(config.dof));
    result.graph_build_s = ElapsedSeconds(graph_start);

    if (config.eval) {
      const IDSampleBatch samples = MakeIDSamples(config.dof, 1, config.seed);
      const auto eval_start = Clock::now();
      autodiff.evalFunction(samples.q[0], samples.dq[0], samples.ddq[0]);
      result.eval_s = ElapsedSeconds(eval_start);
      benchmark::DoNotOptimize(autodiff.d2tau_dqdq);
      benchmark::DoNotOptimize(autodiff.d2tau_dqdv);
      benchmark::DoNotOptimize(autodiff.d2tau_dvdv);
      benchmark::DoNotOptimize(autodiff.d2tau_dadq);
    }
    return;
  }

  throw std::invalid_argument("unsupported case: " + config.case_name);
}

}  // namespace

int main(int argc, char** argv) {
  PrintCsvHeader();

  ProbeResult result;
  try {
    const CliConfig config = ParseCli(argc, argv);
    result.case_name = config.case_name;
    result.dof = config.dof;
    result.branching_factor = config.branching_factor;
    result.eval = config.eval ? 1 : 0;
    result.horizon = config.horizon;

    ProbeOk(config, result);
    PrintCsvRow(result);
    return 0;
  } catch (const std::bad_alloc& e) {
    result.status = "bad_alloc";
    result.error = e.what();
    PrintCsvRow(result);
    return 2;
  } catch (const std::exception& e) {
    result.status = "exception";
    result.error = e.what();
    PrintCsvRow(result);
    return 2;
  } catch (...) {
    result.status = "unknown_exception";
    result.error = "unknown exception";
    PrintCsvRow(result);
    return 2;
  }
}
