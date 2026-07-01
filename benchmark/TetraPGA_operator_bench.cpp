#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "TetraPGA/Motor.hpp"
#include "TetraPGA/PGA.hpp"

namespace {

using TetraPGA::Line3D;
using TetraPGA::Motor3D;
using TetraPGA::Point3D;

constexpr int kSampleBatchSize = 4096;
constexpr int kBenchmarkIterations = 1000000;
constexpr double kPi = 3.141592653589793238462643383279502884;

struct OperatorSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Motor3D<> motor;
  Motor3D<> motor_rhs;
  Line3D<> line;
  Line3D<> line_rhs;
  Line3D<> force_line;
  Point3D<> point;
  Eigen::Vector3d point3;
  Eigen::Matrix<double, 6, 6> inertia_ga;

  pinocchio::SE3 se3;
  pinocchio::SE3 se3_rhs;
  pinocchio::Motion motion;
  pinocchio::Motion motion_rhs;
  pinocchio::Force force;
  pinocchio::Inertia inertia;
};

using OperatorSamples =
    std::vector<OperatorSample, Eigen::aligned_allocator<OperatorSample>>;

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d out;
  out << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
  return out;
}

Eigen::Vector3d RandomVector3(std::mt19937& gen, const double range) {
  std::uniform_real_distribution<double> dist(-range, range);
  return Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
}

Eigen::Quaterniond RandomQuaternion(std::mt19937& gen) {
  std::uniform_real_distribution<double> angle_dist(-kPi, kPi);
  Eigen::Vector3d axis = RandomVector3(gen, 1.0);
  if (axis.norm() < 1e-12) {
    axis = Eigen::Vector3d::UnitX();
  }
  axis.normalize();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle_dist(gen), axis));
}

Eigen::Matrix<double, 6, 6> MakeTetraPGAInertia(const double mass,
                                                const Eigen::Vector3d& com,
                                                const Eigen::Matrix3d& inertia_com) {
  const Eigen::Matrix3d inertia_origin =
      inertia_com + mass * (com.squaredNorm() * Eigen::Matrix3d::Identity() -
                            com * com.transpose());
  const Eigen::Matrix3d com_cross = Skew(com);

  Eigen::Matrix<double, 6, 6> out = Eigen::Matrix<double, 6, 6>::Zero();
  out.template block<3, 3>(0, 0) = inertia_origin;
  out.template block<3, 3>(0, 3) = mass * com_cross;
  out.template block<3, 3>(3, 0) = -mass * com_cross;
  out.template block<3, 3>(3, 3) = mass * Eigen::Matrix3d::Identity();
  return out;
}

OperatorSamples MakeSamples() {
  std::mt19937 gen(0x5EED1234u);
  std::uniform_real_distribution<double> scalar_dist(-1.0, 1.0);
  std::uniform_real_distribution<double> positive_dist(0.1, 2.0);

  OperatorSamples samples;
  samples.reserve(kSampleBatchSize);
  for (int i = 0; i < kSampleBatchSize; ++i) {
    OperatorSample sample;

    const Eigen::Quaterniond q = RandomQuaternion(gen);
    const Eigen::Quaterniond q_rhs = RandomQuaternion(gen);
    const Eigen::Vector3d t = RandomVector3(gen, 1.0);
    const Eigen::Vector3d t_rhs = RandomVector3(gen, 1.0);

    sample.motor = TetraPGA::Quat_2motor(q, t);
    sample.motor_rhs = TetraPGA::Quat_2motor(q_rhs, t_rhs);
    sample.se3 = pinocchio::SE3(q.toRotationMatrix(), t);
    sample.se3_rhs = pinocchio::SE3(q_rhs.toRotationMatrix(), t_rhs);

    const Eigen::Vector3d angular = RandomVector3(gen, 2.0);
    const Eigen::Vector3d linear = RandomVector3(gen, 2.0);
    const Eigen::Vector3d angular_rhs = RandomVector3(gen, 2.0);
    const Eigen::Vector3d linear_rhs = RandomVector3(gen, 2.0);
    sample.line << angular, linear;
    sample.line_rhs << angular_rhs, linear_rhs;
    sample.motion = pinocchio::Motion(linear, angular);
    sample.motion_rhs = pinocchio::Motion(linear_rhs, angular_rhs);

    const Eigen::Vector3d torque = RandomVector3(gen, 10.0);
    const Eigen::Vector3d force = RandomVector3(gen, 10.0);
    sample.force_line << torque, force;
    sample.force = pinocchio::Force(force, torque);

    sample.point3 = RandomVector3(gen, 1.0);
    sample.point << sample.point3, 1.0;

    const double mass = positive_dist(gen);
    const Eigen::Vector3d com = RandomVector3(gen, 0.25);
    Eigen::Matrix3d inertia_com = Eigen::Matrix3d::Zero();
    inertia_com.diagonal() << positive_dist(gen), positive_dist(gen), positive_dist(gen);
    sample.inertia = pinocchio::Inertia(mass, com, inertia_com);
    sample.inertia_ga = MakeTetraPGAInertia(mass, com, inertia_com);

    samples.push_back(sample);
  }
  return samples;
}

template <typename Function>
void RunSampledBenchmark(benchmark::State& state, Function&& function) {
  const OperatorSamples samples = MakeSamples();
  std::size_t cursor = 0;
  for (auto _ : state) {
    const OperatorSample& sample = samples[cursor];
    function(sample);
    cursor = (cursor + 1) % samples.size();
  }
  benchmark::DoNotOptimize(cursor);
  state.SetItemsProcessed(state.iterations());
}

void BM_TetraPGAPointTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Point3D<> out = TetraPGA::pga_rbm3(sample.motor, sample.point);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioPointTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Eigen::Vector3d out = sample.se3.act(sample.point3);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_TetraPGAMotionTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Line3D<> out = TetraPGA::ga_rbm(sample.motor, sample.line);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioMotionTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::Motion out = sample.se3.act(sample.motion);
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGAMotionInverseTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Line3D<> out = TetraPGA::ga_AdM(sample.motor, sample.line);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioMotionInverseTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::Motion out = sample.se3.actInv(sample.motion);
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGAForceTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Line3D<> out = TetraPGA::ga_rbm(sample.motor, sample.force_line);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioForceTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::Force out = sample.se3.act(sample.force);
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGAAdjointMatrix(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Eigen::Matrix<double, 6, 6> out = TetraPGA::ga_rbm(sample.motor);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioAdjointMatrix(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::SE3::ActionMatrixType out = sample.se3.toActionMatrix();
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_TetraPGAAdjointInverseMatrix(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Eigen::Matrix<double, 6, 6> out = TetraPGA::ga_AdM(sample.motor);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioAdjointInverseMatrix(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::SE3::ActionMatrixType out = sample.se3.toActionMatrixInverse();
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_TetraPGACommutatorDirect(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Line3D<> out = TetraPGA::ga_com(sample.line, sample.line_rhs);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioCommutatorDirect(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::Motion out = sample.motion.cross(sample.motion_rhs);
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGAPointCommutator(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Point3D<> out = TetraPGA::pga_com23(sample.line, sample.point);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioPointCommutator(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Eigen::Vector3d out =
        sample.motion.linear() + sample.motion.angular().cross(sample.point3);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_TetraPGACommutatorMatrix(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Eigen::Matrix<double, 6, 6> out = TetraPGA::ga_com(sample.line);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioCommutatorMatrix(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::Motion::ActionMatrixType out = sample.motion.toActionMatrix();
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_TetraPGAMotorCompose(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Motor3D<> out = TetraPGA::ga_mul(sample.motor, sample.motor_rhs);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioMotorCompose(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::SE3 out = sample.se3 * sample.se3_rhs;
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGAMotorInverse(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Motor3D<> out = TetraPGA::ga_rev(sample.motor);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioMotorInverse(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::SE3 out = sample.se3.inverse();
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGAExpMap(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Motor3D<> out = TetraPGA::ga_exp(sample.line);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioExpMap(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::SE3 out = pinocchio::exp6(sample.motion);
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGALogMap(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Line3D<> out = TetraPGA::ga_log(sample.motor);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioLogMap(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::Motion out = pinocchio::log6(sample.se3);
    benchmark::DoNotOptimize(out);
  });
}

void BM_TetraPGAInertiaTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const Eigen::Matrix<double, 6, 6> out =
        TetraPGA::ga_rbm(sample.motor) * sample.inertia_ga *
        TetraPGA::ga_AdM(sample.motor);
    benchmark::DoNotOptimize(out.data());
  });
}

void BM_PinocchioInertiaTransform(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const pinocchio::Inertia out = sample.se3.act(sample.inertia);
    benchmark::DoNotOptimize(out);
  });
}

void BM_SampleAccessBaseline(benchmark::State& state) {
  RunSampledBenchmark(state, [](const OperatorSample& sample) {
    const double out = sample.line(0) + sample.motion.linear()(0) + sample.point3(0);
    benchmark::DoNotOptimize(out);
  });
}

}  // namespace

#define REGISTER_OPERATOR_BENCH(function_name, benchmark_name) \
  BENCHMARK(function_name)                                     \
      ->Name(benchmark_name)                                   \
      ->Iterations(kBenchmarkIterations)                       \
      ->Unit(benchmark::kNanosecond)

REGISTER_OPERATOR_BENCH(BM_TetraPGAPointTransform,
                        "point_transform/TetraPGA/pga_rbm3");
REGISTER_OPERATOR_BENCH(BM_PinocchioPointTransform,
                        "point_transform/Pinocchio/SE3_act_Vector3");
REGISTER_OPERATOR_BENCH(BM_TetraPGAMotionTransform,
                        "motion_transform_direct/TetraPGA/ga_rbm");
REGISTER_OPERATOR_BENCH(BM_PinocchioMotionTransform,
                        "motion_transform_direct/Pinocchio/SE3_act_Motion");
REGISTER_OPERATOR_BENCH(BM_TetraPGAMotionInverseTransform,
                        "motion_inverse_transform/TetraPGA/ga_AdM");
REGISTER_OPERATOR_BENCH(BM_PinocchioMotionInverseTransform,
                        "motion_inverse_transform/Pinocchio/SE3_actInv_Motion");
REGISTER_OPERATOR_BENCH(BM_TetraPGAForceTransform,
                        "force_transform/TetraPGA/ga_rbm");
REGISTER_OPERATOR_BENCH(BM_PinocchioForceTransform,
                        "force_transform/Pinocchio/SE3_act_Force");
REGISTER_OPERATOR_BENCH(BM_TetraPGAAdjointMatrix,
                        "adjoint_matrix/TetraPGA/ga_rbm_matrix");
REGISTER_OPERATOR_BENCH(BM_PinocchioAdjointMatrix,
                        "adjoint_matrix/Pinocchio/SE3_toActionMatrix");
REGISTER_OPERATOR_BENCH(BM_TetraPGAAdjointInverseMatrix,
                        "adjoint_inverse_matrix/TetraPGA/ga_AdM_matrix");
REGISTER_OPERATOR_BENCH(BM_PinocchioAdjointInverseMatrix,
                        "adjoint_inverse_matrix/Pinocchio/SE3_toActionMatrixInverse");
REGISTER_OPERATOR_BENCH(BM_TetraPGACommutatorDirect,
                        "commutator_direct/TetraPGA/ga_com");
REGISTER_OPERATOR_BENCH(BM_PinocchioCommutatorDirect,
                        "commutator_direct/Pinocchio/Motion_cross");
REGISTER_OPERATOR_BENCH(BM_TetraPGAPointCommutator,
                        "point_commutator/TetraPGA/pga_com23");
REGISTER_OPERATOR_BENCH(BM_PinocchioPointCommutator,
                        "point_commutator/Pinocchio/Motion_point_velocity");
REGISTER_OPERATOR_BENCH(BM_TetraPGACommutatorMatrix,
                        "commutator_matrix/TetraPGA/ga_com_matrix");
REGISTER_OPERATOR_BENCH(BM_PinocchioCommutatorMatrix,
                        "commutator_matrix/Pinocchio/Motion_toActionMatrix");
REGISTER_OPERATOR_BENCH(BM_TetraPGAMotorCompose,
                        "motor_compose/TetraPGA/ga_mul");
REGISTER_OPERATOR_BENCH(BM_PinocchioMotorCompose,
                        "motor_compose/Pinocchio/SE3_multiply");
REGISTER_OPERATOR_BENCH(BM_TetraPGAMotorInverse,
                        "motor_inverse/TetraPGA/ga_rev");
REGISTER_OPERATOR_BENCH(BM_PinocchioMotorInverse,
                        "motor_inverse/Pinocchio/SE3_inverse");
REGISTER_OPERATOR_BENCH(BM_TetraPGAExpMap,
                        "exp_map/TetraPGA/ga_exp");
REGISTER_OPERATOR_BENCH(BM_PinocchioExpMap,
                        "exp_map/Pinocchio/exp6");
REGISTER_OPERATOR_BENCH(BM_TetraPGALogMap,
                        "log_map/TetraPGA/ga_log");
REGISTER_OPERATOR_BENCH(BM_PinocchioLogMap,
                        "log_map/Pinocchio/log6");
REGISTER_OPERATOR_BENCH(BM_TetraPGAInertiaTransform,
                        "inertia_transform/TetraPGA/rbm_I_AdM");
REGISTER_OPERATOR_BENCH(BM_PinocchioInertiaTransform,
                        "inertia_transform/Pinocchio/SE3_act_Inertia");
REGISTER_OPERATOR_BENCH(BM_SampleAccessBaseline,
                        "baseline/harness/sample_access");

BENCHMARK_MAIN();
