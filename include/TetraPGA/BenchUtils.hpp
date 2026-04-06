#pragma once

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <benchmark/benchmark.h>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/autodiff/casadi-algo.hpp>

#include "TetraPGA/ModelRepo.hpp"

namespace TetraPGA::bench {

struct FDSampleBatch {
  std::vector<Eigen::VectorXd> q;
  std::vector<Eigen::VectorXd> v;
  std::vector<Eigen::VectorXd> tau;
  std::size_t cursor = 0;
};

struct IDSampleBatch {
  std::vector<Eigen::VectorXd> q;
  std::vector<Eigen::VectorXd> dq;
  std::vector<Eigen::VectorXd> ddq;
  std::size_t cursor = 0;
};

struct BenchmarkCliArgs {
  std::vector<std::string> storage;
  std::vector<char*> argv;
  std::string csv_path;

  int argc() const { return static_cast<int>(argv.size()); }
  char** data() { return argv.data(); }
};

inline bool HasPrefix(std::string_view value, std::string_view prefix) {
  return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

inline std::string SanitizeBenchmarkName(std::string_view name) {
  std::string out;
  out.reserve(name.size());
  for (const char c : name) {
    const bool is_alnum = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                          (c >= '0' && c <= '9');
    out.push_back(is_alnum ? c : '_');
  }
  if (out.empty()) {
    out = "benchmark";
  }
  return out;
}

inline std::string BenchmarkExecutableName(const char* argv0) {
  if (argv0 == nullptr || argv0[0] == '\0') {
    return "benchmark";
  }
  const std::filesystem::path executable_path(argv0);
  const std::string stem = executable_path.stem().string();
  return stem.empty() ? "benchmark" : stem;
}

inline std::string DefaultBenchmarkCsvPath(std::string_view benchmark_name, const char* argv0) {
  const std::string file_stem = benchmark_name.empty()
                                    ? BenchmarkExecutableName(argv0)
                                    : SanitizeBenchmarkName(benchmark_name);
  const std::filesystem::path package_root =
      std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  const std::filesystem::path output_dir = package_root / "log";
  std::filesystem::create_directories(output_dir);
  return (output_dir / (file_stem + ".csv")).string();
}

inline BenchmarkCliArgs PrepareBenchmarkCsvArgs(int argc, char** argv,
                                                std::string_view benchmark_name) {
  BenchmarkCliArgs out;
  out.storage.reserve(static_cast<std::size_t>(argc));

  for (int i = 0; i < argc; ++i) {
    const std::string_view arg(argv[i] == nullptr ? "" : argv[i]);
    if (arg == "--benchmark_out" && i + 1 < argc) {
      out.csv_path = argv[i + 1] == nullptr ? "" : argv[i + 1];
      ++i;
    } else if (HasPrefix(arg, "--benchmark_out=")) {
      out.csv_path = std::string(arg.substr(std::string_view("--benchmark_out=").size()));
    } else if (arg == "--benchmark_out_format" && i + 1 < argc) {
      ++i;
    } else if (HasPrefix(arg, "--benchmark_out_format=")) {
      continue;
    } else {
      out.storage.emplace_back(arg);
    }
  }

  if (out.csv_path.empty()) {
    out.csv_path = DefaultBenchmarkCsvPath(benchmark_name, argc > 0 ? argv[0] : nullptr);
  }

  if (!out.csv_path.empty()) {
    const std::filesystem::path output_path(out.csv_path);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
  }

  out.argv.reserve(out.storage.size());
  for (std::string& arg : out.storage) {
    out.argv.push_back(arg.data());
  }
  return out;
}

enum class PivotMetricSource {
  kCounter,
  kCpuTimeMs,
  kRealTimeMs,
};

struct PivotCsvReporterConfig {
  PivotMetricSource metric_source = PivotMetricSource::kCpuTimeMs;
  std::string metric_name;
  std::string first_column_name = "case";
  std::string x_axis_counter_name = "DOF";
};

inline std::string CsvEscape(std::string_view value) {
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

inline std::string FormatCsvNumber(double value) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << value;
  std::string out = oss.str();
  while (!out.empty() && out.back() == '0') {
    out.pop_back();
  }
  if (!out.empty() && out.back() == '.') {
    out.pop_back();
  }
  return out.empty() ? "0" : out;
}

inline double TimeUnitToMilliseconds(double value, benchmark::TimeUnit unit) {
  switch (unit) {
    case benchmark::kSecond:
      return value * 1e3;
    case benchmark::kMillisecond:
      return value;
    case benchmark::kMicrosecond:
      return value / 1e3;
    case benchmark::kNanosecond:
      return value / 1e6;
  }
  return value;
}

inline std::string BenchmarkCaseName(const benchmark::BenchmarkReporter::Run& run) {
  std::string name = run.benchmark_name();

  const std::string iterations_tag = "/iterations:";
  const std::size_t iterations_pos = name.find(iterations_tag);
  if (iterations_pos != std::string::npos) {
    name.resize(iterations_pos);
  }

  if (!run.counters.empty()) {
    const std::size_t last_slash = name.find_last_of('/');
    if (last_slash != std::string::npos) {
      bool numeric_suffix = last_slash + 1 < name.size();
      for (std::size_t i = last_slash + 1; i < name.size(); ++i) {
        const char c = name[i];
        if (c < '0' || c > '9') {
          numeric_suffix = false;
          break;
        }
      }
      if (numeric_suffix) {
        name.resize(last_slash);
      }
    }
  }
  return name;
}

inline std::optional<int> BenchmarkXAxisValue(const benchmark::BenchmarkReporter::Run& run,
                                              std::string_view counter_name) {
  const auto it = run.counters.find(std::string(counter_name));
  if (it != run.counters.end()) {
    return static_cast<int>(std::llround(it->second.value));
  }
  return std::nullopt;
}

inline std::optional<double> BenchmarkPivotMetric(
    const benchmark::BenchmarkReporter::Run& run,
    const PivotCsvReporterConfig& config) {
  switch (config.metric_source) {
    case PivotMetricSource::kCounter: {
      const auto it = run.counters.find(config.metric_name);
      if (it == run.counters.end()) {
        return std::nullopt;
      }
      return it->second.value;
    }
    case PivotMetricSource::kCpuTimeMs:
      return TimeUnitToMilliseconds(run.GetAdjustedCPUTime(), run.time_unit);
    case PivotMetricSource::kRealTimeMs:
      return TimeUnitToMilliseconds(run.GetAdjustedRealTime(), run.time_unit);
  }
  return std::nullopt;
}

class PivotCsvReporter final : public benchmark::BenchmarkReporter {
 public:
  PivotCsvReporter(std::string csv_path, PivotCsvReporterConfig config)
      : csv_path_(std::move(csv_path)), config_(std::move(config)) {}

  bool ReportContext(const Context&) override { return true; }

  void ReportRuns(const std::vector<Run>& reports) override {
    for (const Run& run : reports) {
      if (run.error_occurred || run.run_type != Run::RT_Iteration) {
        continue;
      }

      const std::optional<int> dof = BenchmarkXAxisValue(run, config_.x_axis_counter_name);
      const std::optional<double> metric = BenchmarkPivotMetric(run, config_);
      if (!dof.has_value() || !metric.has_value()) {
        continue;
      }

      const std::string case_name = BenchmarkCaseName(run);
      dofs_.insert(*dof);
      rows_[case_name][*dof] = *metric;
    }
  }

  void Finalize() override {
    if (csv_path_.empty()) {
      return;
    }

    std::ofstream out(csv_path_);
    if (!out.is_open()) {
      throw std::runtime_error("Failed to open benchmark CSV output: " + csv_path_);
    }

    out << config_.first_column_name;
    for (const int dof : dofs_) {
      out << ',' << dof;
    }
    out << '\n';

    for (const auto& row : rows_) {
      out << CsvEscape(row.first);
      for (const int dof : dofs_) {
        out << ',';
        const auto it = row.second.find(dof);
        if (it != row.second.end()) {
          out << FormatCsvNumber(it->second);
        }
      }
      out << '\n';
    }
  }

 private:
  std::string csv_path_;
  PivotCsvReporterConfig config_;
  std::set<int> dofs_;
  std::map<std::string, std::map<int, double>> rows_;
};

class CombinedReporter final : public benchmark::BenchmarkReporter {
 public:
  CombinedReporter(benchmark::BenchmarkReporter* primary,
                   benchmark::BenchmarkReporter* secondary)
      : primary_(primary), secondary_(secondary) {}

  bool ReportContext(const Context& context) override {
    bool ok = true;
    if (primary_ != nullptr) {
      ok = primary_->ReportContext(context) && ok;
    }
    if (secondary_ != nullptr) {
      ok = secondary_->ReportContext(context) && ok;
    }
    return ok;
  }

  void ReportRuns(const std::vector<Run>& reports) override {
    if (primary_ != nullptr) {
      primary_->ReportRuns(reports);
    }
    if (secondary_ != nullptr) {
      secondary_->ReportRuns(reports);
    }
  }

  void Finalize() override {
    if (primary_ != nullptr) {
      primary_->Finalize();
    }
    if (secondary_ != nullptr) {
      secondary_->Finalize();
    }
  }

 private:
  benchmark::BenchmarkReporter* primary_;
  benchmark::BenchmarkReporter* secondary_;
};

inline std::uint32_t BenchmarkRunSeed() {
  static const std::uint32_t seed = []() {
    std::random_device rd;
    const std::uint32_t s0 = rd();
    const std::uint32_t s1 = rd();
    const std::uint32_t seed = s0 ^ (s1 + 0x9e3779b9u + (s0 << 6) + (s0 >> 2));
    return seed == 0u ? 0x12345678u : seed;
  }();
  return seed;
}

inline std::uint32_t MixBenchmarkSeed(std::uint32_t base_seed, std::uint32_t stream_id,
                                      std::uint32_t arg0 = 0u, std::uint32_t arg1 = 0u) {
  auto mix = [](std::uint32_t seed, std::uint32_t value) {
    seed ^= value + 0x9e3779b9u + (seed << 6) + (seed >> 2);
    return seed;
  };

  std::uint32_t seed = base_seed;
  seed = mix(seed, stream_id);
  seed = mix(seed, arg0);
  seed = mix(seed, arg1);
  return seed;
}

inline FDSampleBatch MakeFDSamples(int dof, int batch_size, std::uint32_t seed) {
  FDSampleBatch out;
  out.q.reserve(static_cast<std::size_t>(batch_size));
  out.v.reserve(static_cast<std::size_t>(batch_size));
  out.tau.reserve(static_cast<std::size_t>(batch_size));

  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> qdist(-M_PI, M_PI);
  std::uniform_real_distribution<double> vdist(-2.0, 2.0);
  std::uniform_real_distribution<double> tdist(-10.0, 10.0);

  for (int k = 0; k < batch_size; ++k) {
    Eigen::VectorXd q(dof), v(dof), tau(dof);
    for (int i = 0; i < dof; ++i) {
      q(i) = qdist(gen);
      v(i) = vdist(gen);
      tau(i) = tdist(gen);
    }
    out.q.push_back(std::move(q));
    out.v.push_back(std::move(v));
    out.tau.push_back(std::move(tau));
  }
  return out;
}

inline IDSampleBatch MakeIDSamples(int dof, int batch_size, std::uint32_t seed) {
  IDSampleBatch out;
  out.q.reserve(static_cast<std::size_t>(batch_size));
  out.dq.reserve(static_cast<std::size_t>(batch_size));
  out.ddq.reserve(static_cast<std::size_t>(batch_size));

  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> qdist(-M_PI, M_PI);
  std::uniform_real_distribution<double> vdist(-2.0, 2.0);
  std::uniform_real_distribution<double> adist(-20.0, 20.0);

  for (int k = 0; k < batch_size; ++k) {
    Eigen::VectorXd q(dof), dq(dof), ddq(dof);
    for (int i = 0; i < dof; ++i) {
      q(i) = qdist(gen);
      dq(i) = vdist(gen);
      ddq(i) = adist(gen);
    }
    out.q.push_back(std::move(q));
    out.dq.push_back(std::move(dq));
    out.ddq.push_back(std::move(ddq));
  }
  return out;
}

inline int DofFromLevel(int level) {
  return (1 << level) - 1;
}

inline TreeTemplateParams MakeBenchTreeParams(int branching_factor, int dof) {
  return make_tree_template_params(dof, branching_factor, 0.95, 0.0);
}

inline Model<double> MakeGABenchModel(const TreeTemplateParams& params, const std::string& name) {
  return tree_model(params, name);
}

inline pinocchio::Model BuildPinModel(const TreeTemplateParams& params) {
  pinocchio::Model model;
  model.gravity.linear() << params.gravity.x(), params.gravity.y(), params.gravity.z();
  model.gravity.angular().setZero();

  const int n = params.dof;
  std::vector<pinocchio::JointIndex> joint_ids(static_cast<std::size_t>(n + 1), 0);

  for (int i = 1; i <= n; ++i) {
    const int parent_body = params.parent_indices[static_cast<std::size_t>(i)];
    const pinocchio::JointIndex parent_joint = joint_ids[static_cast<std::size_t>(parent_body)];

    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    if (parent_body > 0) {
      trans.x() = params.link_lengths[static_cast<std::size_t>(parent_body - 1)];
      rot = Eigen::AngleAxisd(params.skew, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }
    const pinocchio::SE3 joint_placement(rot, trans);

    const pinocchio::JointIndex jid =
        model.addJoint(parent_joint, pinocchio::JointModelRZ(), joint_placement,
                       "joint_" + std::to_string(i));

    const double m = params.masses[static_cast<std::size_t>(i - 1)];
    const Eigen::Vector3d c = params.coms[static_cast<std::size_t>(i - 1)];
    const Eigen::Matrix<double, 6, 1>& Ivec =
        params.inertia_tensors[static_cast<std::size_t>(i - 1)];
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I << Ivec(0), Ivec(1), Ivec(2),
         Ivec(1), Ivec(3), Ivec(4),
         Ivec(2), Ivec(4), Ivec(5);
    model.appendBodyToJoint(jid, pinocchio::Inertia(m, c, I), pinocchio::SE3::Identity());

    joint_ids[static_cast<std::size_t>(i)] = jid;
  }

  return model;
}

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

class InlineAutoDiffRNEADerivatives {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit InlineAutoDiffRNEADerivatives(const pinocchio::Model& model,
                                         const std::string& tag)
      : ad_model_(model.template cast<casadi::SX>()),
        ad_data_(ad_model_),
        fun_name_(tag + "_eval"),
        q_ad_(model.nq),
        v_ad_(model.nv),
        a_ad_(model.nv),
        q_vec_(static_cast<std::size_t>(model.nq)),
        v_vec_(static_cast<std::size_t>(model.nv)),
        a_vec_(static_cast<std::size_t>(model.nv)),
        tau(model.nv),
        dtau_dq(model.nv, model.nv),
        dtau_dv(model.nv, model.nv),
        dtau_da(model.nv, model.nv) {
    using ADConfigVectorType = pinocchio::ModelTpl<casadi::SX>::ConfigVectorType;
    using ADTangentVectorType = pinocchio::ModelTpl<casadi::SX>::TangentVectorType;

    cs_q_ = casadi::SX::sym("q", model.nq);
    cs_v_ = casadi::SX::sym("v", model.nv);
    cs_a_ = casadi::SX::sym("a", model.nv);
    cs_tau_ = casadi::SX(model.nv, 1);
    cs_dtau_dq_ = casadi::SX(model.nv, model.nv);
    cs_dtau_dv_ = casadi::SX(model.nv, model.nv);
    cs_dtau_da_ = casadi::SX(model.nv, model.nv);

    q_ad_ = Eigen::Map<ADConfigVectorType>(
        static_cast<std::vector<casadi::SX>>(cs_q_).data(), model.nq, 1);
    v_ad_ = Eigen::Map<ADTangentVectorType>(
        static_cast<std::vector<casadi::SX>>(cs_v_).data(), model.nv, 1);
    a_ad_ = Eigen::Map<ADTangentVectorType>(
        static_cast<std::vector<casadi::SX>>(cs_a_).data(), model.nv, 1);

    buildMap();
    fun_ = ad_fun_;
  }

  void evalFunction(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                    const Eigen::VectorXd& a) {
    Eigen::Map<Eigen::VectorXd>(q_vec_.data(), static_cast<Eigen::Index>(q_vec_.size()), 1) = q;
    Eigen::Map<Eigen::VectorXd>(v_vec_.data(), static_cast<Eigen::Index>(v_vec_.size()), 1) = v;
    Eigen::Map<Eigen::VectorXd>(a_vec_.data(), static_cast<Eigen::Index>(a_vec_.size()), 1) = a;

    const casadi::DMVector out = fun_(casadi::DMVector{q_vec_, v_vec_, a_vec_});
    const std::size_t nv = static_cast<std::size_t>(ad_model_.nv);

    tau = Eigen::Map<const Eigen::VectorXd>(
        static_cast<std::vector<double>>(out[0]).data(), static_cast<Eigen::Index>(nv), 1);
    dtau_dq = Eigen::Map<const Eigen::MatrixXd>(
        static_cast<std::vector<double>>(out[1]).data(), static_cast<Eigen::Index>(nv),
        static_cast<Eigen::Index>(nv));
    dtau_dv = Eigen::Map<const Eigen::MatrixXd>(
        static_cast<std::vector<double>>(out[2]).data(), static_cast<Eigen::Index>(nv),
        static_cast<Eigen::Index>(nv));
    dtau_da = Eigen::Map<const Eigen::MatrixXd>(
        static_cast<std::vector<double>>(out[3]).data(), static_cast<Eigen::Index>(nv),
        static_cast<Eigen::Index>(nv));
  }

  Eigen::VectorXd tau;
  Eigen::MatrixXd dtau_dq;
  Eigen::MatrixXd dtau_dv;
  Eigen::MatrixXd dtau_da;

 private:
  void buildMap() {
    pinocchio::rnea(ad_model_, ad_data_, q_ad_, v_ad_, a_ad_);
    pinocchio::computeRNEADerivatives(ad_model_, ad_data_, q_ad_, v_ad_, a_ad_);

    pinocchio::casadi::copy(ad_data_.tau, cs_tau_);
    pinocchio::casadi::copy(ad_data_.dtau_dq, cs_dtau_dq_);
    pinocchio::casadi::copy(ad_data_.dtau_dv, cs_dtau_dv_);
    pinocchio::casadi::copy(ad_data_.M, cs_dtau_da_);

    ad_fun_ = casadi::Function(fun_name_, casadi::SXVector{cs_q_, cs_v_, cs_a_},
                               casadi::SXVector{cs_tau_, cs_dtau_dq_, cs_dtau_dv_,
                                                cs_dtau_da_});
  }

  pinocchio::ModelTpl<casadi::SX> ad_model_;
  pinocchio::DataTpl<casadi::SX> ad_data_;
  std::string fun_name_;
  casadi::Function ad_fun_;
  casadi::Function fun_;
  casadi::SX cs_q_;
  casadi::SX cs_v_;
  casadi::SX cs_a_;
  casadi::SX cs_tau_;
  casadi::SX cs_dtau_dq_;
  casadi::SX cs_dtau_dv_;
  casadi::SX cs_dtau_da_;
  pinocchio::ModelTpl<casadi::SX>::ConfigVectorType q_ad_;
  pinocchio::ModelTpl<casadi::SX>::TangentVectorType v_ad_;
  pinocchio::ModelTpl<casadi::SX>::TangentVectorType a_ad_;
  std::vector<double> q_vec_;
  std::vector<double> v_vec_;
  std::vector<double> a_vec_;
};

}  // namespace TetraPGA::bench
