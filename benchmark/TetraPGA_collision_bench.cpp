#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 30

#include <benchmark/benchmark.h>

#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "TetraPGA/Collision.hpp"
#include "TetraPGA/BenchUtils.hpp"
#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/Kinematics.hpp"
#include "TetraPGA/ModelRepo.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace TetraPGA;
using namespace TetraPGA::bench;

namespace {

constexpr int kMinObstacleCount = 2;
constexpr int kMaxObstacleCount = 256;
constexpr int kSampleBatchSize = 64;
constexpr int kBenchmarkIterations = 128;
constexpr std::uint32_t kBenchmarkSeed = 20260320u;

struct CollisionSample {
    CollisionSample(const Model<double>& model,
                    const std::vector<SSP<double>>& spheres,
                    const Eigen::VectorXd& q_sample)
        : q(q_sample), env(spheres), env_data(model, env) {
        sphere_geometries.reserve(static_cast<std::size_t>(env.num_static_sphere));
        sphere_centers.reserve(static_cast<std::size_t>(env.num_static_sphere));
        for (int i = 0; i < env.num_static_sphere; ++i) {
            sphere_geometries.push_back(std::make_shared<coal::Sphere>(env.static_sphere[i].radius));
            sphere_centers.push_back(env.static_sphere[i].center.template head<3>());
        }
    }

    Eigen::VectorXd q;
    Environment<double> env;
    EnvironmentData<double> env_data;
    std::vector<std::shared_ptr<coal::Sphere>> sphere_geometries;
    std::vector<coal::Vec3s> sphere_centers;
};

struct CollisionSampleBatch {
    std::vector<CollisionSample> samples;
    std::size_t cursor{0};
};

Point3D<double> makePoint(const double x, const double y, const double z) {
    Point3D<double> point;
    point << x, y, z, 1.0;
    return point;
}

coal::Transform3s makeCapsuleTransform(const Eigen::Vector3d& endpoint_a,
                                       const Eigen::Vector3d& endpoint_b) {
    const Eigen::Vector3d center = 0.5 * (endpoint_a + endpoint_b);
    const Eigen::Vector3d axis = endpoint_b - endpoint_a;

    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    if (axis.norm() > 1e-12) {
        rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis.normalized());
    }

    return coal::Transform3s(rotation, center);
}

Eigen::MatrixXd computePinocchioPointJacobianWorld(const pinocchio::Model& model,
                                                   pinocchio::Data& data,
                                                   const pinocchio::JointIndex joint_id,
                                                   const Eigen::Vector3d& point_world) {
    Eigen::MatrixXd joint_jacobian(6, model.nv);
    joint_jacobian.setZero();
    pinocchio::getJointJacobian(model, data, joint_id, pinocchio::WORLD, joint_jacobian);

    Eigen::MatrixXd point_jacobian(3, model.nv);
    point_jacobian.noalias() =
        joint_jacobian.topRows<3>() - skew(point_world) * joint_jacobian.bottomRows<3>();
    return point_jacobian;
}

std::vector<SSP<double>> makeRandomSpheres(const int obstacle_count, std::mt19937& rng) {
    std::uniform_real_distribution<double> radius_dist(0.02, 0.08);
    std::uniform_real_distribution<double> x_dist(0.20, 0.85);
    std::uniform_real_distribution<double> y_dist(-0.55, 0.55);
    std::uniform_real_distribution<double> z_dist(0.00, 0.95);

    std::vector<SSP<double>> spheres;
    spheres.reserve(static_cast<std::size_t>(obstacle_count));
    for (int i = 0; i < obstacle_count; ++i) {
        spheres.push_back(
            SSP<double>{0,
                        radius_dist(rng),
                        makePoint(x_dist(rng), y_dist(rng), z_dist(rng))});
    }
    return spheres;
}

Eigen::VectorXd makeRandomConfiguration(const Model<double>& model, std::mt19937& rng) {
    std::uniform_real_distribution<double> offset_dist(-0.6, 0.6);

    Eigen::VectorXd q(model.dof_a);
    for (int i = 0; i < model.dof_a; ++i) {
        q[i] = model.qa0[i] + offset_dist(rng);
    }
    return q;
}

CollisionSampleBatch makeSampleBatch(const Model<double>& model, const int obstacle_count) {
    CollisionSampleBatch batch;
    batch.samples.reserve(kSampleBatchSize);

    std::mt19937 rng(kBenchmarkSeed ^ static_cast<std::uint32_t>(obstacle_count * 2654435761u));
    for (int i = 0; i < kSampleBatchSize; ++i) {
        batch.samples.emplace_back(model,
                                   makeRandomSpheres(obstacle_count, rng),
                                   makeRandomConfiguration(model, rng));
    }

    return batch;
}

std::vector<std::shared_ptr<coal::Capsule>> makeCapsuleGeometries(const Model<double>& model) {
    std::vector<std::shared_ptr<coal::Capsule>> capsule_geometries;
    capsule_geometries.reserve(static_cast<std::size_t>(model.num_collision_ssl));

    for (int i = 0; i < model.num_collision_ssl; ++i) {
        const Eigen::Vector3d local_a = model.collisionSSL[i].endpointA.template head<3>();
        const Eigen::Vector3d local_b = model.collisionSSL[i].endpointB.template head<3>();
        const double capsule_length = (local_b - local_a).norm();
        capsule_geometries.push_back(
            std::make_shared<coal::Capsule>(model.collisionSSL[i].radius, capsule_length));
    }

    return capsule_geometries;
}

double runTetraPGAGradientEvaluation(const Model<double>& model,
                                     Data<double>& data,
                                     CollisionSample& sample) {
    higherKinematics(model, data, sample.q);
    computeDistanceJacobian(model, data, sample.env, sample.env_data);

    double checksum = 0.0;
    for (int i = 0; i < sample.env_data.num_collision_pair; ++i) {
        checksum += sample.env_data.distance[i];
        checksum += sample.env_data.jac_dist[i].squaredNorm();
    }
    return checksum;
}

double runPinocchioFCLGradientEvaluation(
    const Model<double>& model,
    const pinocchio::Model& pin_model,
    pinocchio::Data& pin_data,
    CollisionSample& sample,
    const std::vector<std::shared_ptr<coal::Capsule>>& capsule_geometries) {
    pinocchio::forwardKinematics(pin_model, pin_data, sample.q);
    pinocchio::computeJointJacobians(pin_model, pin_data, sample.q);

    double checksum = 0.0;
    coal::DistanceRequest request(true, true);

    for (int capsule_idx = 0; capsule_idx < model.num_collision_ssl; ++capsule_idx) {
        const pinocchio::JointIndex joint_id =
            static_cast<pinocchio::JointIndex>(model.collisionSSL[capsule_idx].id);
        const Eigen::Vector3d local_a = model.collisionSSL[capsule_idx].endpointA.template head<3>();
        const Eigen::Vector3d local_b = model.collisionSSL[capsule_idx].endpointB.template head<3>();
        const Eigen::Vector3d world_a = pin_data.oMi[joint_id].act(local_a);
        const Eigen::Vector3d world_b = pin_data.oMi[joint_id].act(local_b);

        const coal::CollisionObject capsule_object(
            capsule_geometries[static_cast<std::size_t>(capsule_idx)],
            makeCapsuleTransform(world_a, world_b));

        for (int sphere_idx = 0; sphere_idx < sample.env.num_static_sphere; ++sphere_idx) {
            const coal::CollisionObject sphere_object(
                sample.sphere_geometries[static_cast<std::size_t>(sphere_idx)],
                coal::Transform3s(sample.sphere_centers[static_cast<std::size_t>(sphere_idx)]));

            coal::DistanceResult result;
            checksum += coal::distance(&capsule_object, &sphere_object, request, result);

            const Eigen::MatrixXd point_jacobian =
                computePinocchioPointJacobianWorld(pin_model,
                                                   pin_data,
                                                   joint_id,
                                                   result.nearest_points[0]);
            const Eigen::VectorXd projected_gradient =
                -point_jacobian.transpose() * result.normal;
            checksum += projected_gradient.squaredNorm();
        }
    }

    return checksum;
}

void ApplyObstacleCounts(benchmark::internal::Benchmark* benchmark) {
    for (int obstacle_count = kMinObstacleCount; obstacle_count <= kMaxObstacleCount;
         obstacle_count *= 2) {
        benchmark->Arg(obstacle_count);
    }
}

void RunTetraPGACollisionGradientBenchmark(benchmark::State& state) {
    const int obstacle_count = static_cast<int>(state.range(0));

    Model<double> model = ur();
    Data<double> data(model);
    CollisionSampleBatch batch = makeSampleBatch(model, obstacle_count);

    state.counters["DOF"] = static_cast<double>(model.dof_a);
    state.counters["ObstacleCount"] = static_cast<double>(obstacle_count);
    state.counters["CollisionPairs"] =
        static_cast<double>(model.num_collision_ssl * obstacle_count);

    for (auto _ : state) {
        CollisionSample& sample = batch.samples[batch.cursor];
        const double checksum =
            runTetraPGAGradientEvaluation(model, data, sample);

        benchmark::DoNotOptimize(checksum);
        batch.cursor = (batch.cursor + 1) % batch.samples.size();
    }
}

void RunPinocchioFCLCollisionGradientBenchmark(benchmark::State& state) {
    const int obstacle_count = static_cast<int>(state.range(0));
    const std::string robot_assets_dir = std::string(TETRAPGA_ROBOT_ASSETS_DIR);
    const std::string urdf_path = robot_assets_dir + "/ur10/urdf/ur10.urdf";

    Model<double> model = ur();
    pinocchio::Model pin_model;
    pinocchio::urdf::buildModel(urdf_path, pin_model);
    if (pin_model.nv != model.dof_a) {
        throw std::runtime_error("Pinocchio/TetraPGA DOF mismatch in collision gradient benchmark.");
    }

    pinocchio::Data pin_data(pin_model);
    CollisionSampleBatch batch = makeSampleBatch(model, obstacle_count);
    const auto capsule_geometries = makeCapsuleGeometries(model);

    state.counters["DOF"] = static_cast<double>(model.dof_a);
    state.counters["ObstacleCount"] = static_cast<double>(obstacle_count);
    state.counters["CollisionPairs"] =
        static_cast<double>(model.num_collision_ssl * obstacle_count);

    for (auto _ : state) {
        CollisionSample& sample = batch.samples[batch.cursor];
        const double checksum = runPinocchioFCLGradientEvaluation(
            model, pin_model, pin_data, sample, capsule_geometries);

        benchmark::DoNotOptimize(checksum);
        batch.cursor = (batch.cursor + 1) % batch.samples.size();
    }
}

void RegisterAll() {
    benchmark::RegisterBenchmark("ur10/TetraPGA/CollisionGradient",
                                 RunTetraPGACollisionGradientBenchmark)
        ->Apply(ApplyObstacleCounts)
        ->Iterations(kBenchmarkIterations);

    benchmark::RegisterBenchmark("ur10/PinocchioFCL/CollisionGradient",
                                 RunPinocchioFCLCollisionGradientBenchmark)
        ->Apply(ApplyObstacleCounts)
        ->Iterations(kBenchmarkIterations);
}

}  // namespace

int main(int argc, char** argv) {
    RegisterAll();
    auto benchmark_args = PrepareBenchmarkCsvArgs(argc, argv, "TetraPGA_collision_bench");
    int benchmark_argc = benchmark_args.argc();
    char** benchmark_argv = benchmark_args.data();
    benchmark::Initialize(&benchmark_argc, benchmark_argv);
    benchmark::AddCustomContext("Robot", "ur10");
    benchmark::AddCustomContext("ObstacleSchedule", "2,4,8,...,256");
    benchmark::AddCustomContext("FixedIterations", std::to_string(kBenchmarkIterations));
    benchmark::AddCustomContext("SampleBatch", std::to_string(kSampleBatchSize));
    benchmark::AddCustomContext("Seed", std::to_string(kBenchmarkSeed));
    benchmark::AddCustomContext("CSVOutput", benchmark_args.csv_path);
    benchmark::ConsoleReporter console_reporter;
    PivotCsvReporter csv_reporter(
        benchmark_args.csv_path,
        PivotCsvReporterConfig{PivotMetricSource::kCpuTimeMs, "", "case", "ObstacleCount"});
    CombinedReporter combined_reporter(&console_reporter, &csv_reporter);
    benchmark::RunSpecifiedBenchmarks(&combined_reporter);
    benchmark::Shutdown();
    return 0;
}
