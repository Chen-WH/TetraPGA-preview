#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 30

#include "TetraPGA/Collision.hpp"
#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/Kinematics.hpp"
#include "TetraPGA/ModelRepo.hpp"

#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace TetraPGA;

namespace {

struct CoalWitnessResult {
    double distance{0.0};
    Eigen::Vector3d point_capsule{Eigen::Vector3d::Zero()};
    Eigen::Vector3d point_sphere{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
};

Point3D<double> makePoint(const double x, const double y, const double z) {
    Point3D<double> point;
    point << x, y, z, 1.0;
    return point;
}

coal::Transform3s makeCapsuleTransform(const Point3D<double>& endpoint_a,
                                       const Point3D<double>& endpoint_b) {
    const Eigen::Vector3d a = endpoint_a.template head<3>();
    const Eigen::Vector3d b = endpoint_b.template head<3>();
    const Eigen::Vector3d center = 0.5 * (a + b);
    const Eigen::Vector3d axis = b - a;

    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    if (axis.norm() > 1e-12) {
        rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis.normalized());
    }

    return coal::Transform3s(rotation, center);
}

double computeCoalDistance(const SSL<double>& capsule,
                           const Point3D<double>& endpoint_a,
                           const Point3D<double>& endpoint_b,
                           const SSP<double>& sphere) {
    const double capsule_length = (endpoint_b.template head<3>() - endpoint_a.template head<3>()).norm();

    auto capsule_geometry = std::make_shared<coal::Capsule>(capsule.radius, capsule_length);
    auto sphere_geometry = std::make_shared<coal::Sphere>(sphere.radius);
    const coal::Vec3s sphere_center = sphere.center.template head<3>();

    const coal::CollisionObject capsule_object(capsule_geometry, makeCapsuleTransform(endpoint_a, endpoint_b));
    const coal::CollisionObject sphere_object(sphere_geometry, coal::Transform3s(sphere_center));

    coal::DistanceRequest request(true, true);
    coal::DistanceResult result;
    return coal::distance(&capsule_object, &sphere_object, request, result);
}

CoalWitnessResult computeCoalWitness(const SSL<double>& capsule,
                                     const Point3D<double>& endpoint_a,
                                     const Point3D<double>& endpoint_b,
                                     const SSP<double>& sphere) {
    const double capsule_length = (endpoint_b.template head<3>() - endpoint_a.template head<3>()).norm();

    auto capsule_geometry = std::make_shared<coal::Capsule>(capsule.radius, capsule_length);
    auto sphere_geometry = std::make_shared<coal::Sphere>(sphere.radius);
    const coal::Vec3s sphere_center = sphere.center.template head<3>();

    const coal::CollisionObject capsule_object(capsule_geometry, makeCapsuleTransform(endpoint_a, endpoint_b));
    const coal::CollisionObject sphere_object(sphere_geometry, coal::Transform3s(sphere_center));

    coal::DistanceRequest request(true, true);
    coal::DistanceResult result;
    CoalWitnessResult witness;
    witness.distance = coal::distance(&capsule_object, &sphere_object, request, result);
    witness.point_capsule = result.nearest_points[0];
    witness.point_sphere = result.nearest_points[1];
    witness.normal = result.normal;
    return witness;
}

bool expectNear(const double lhs,
                const double rhs,
                const double tolerance,
                const std::string& label) {
    if (std::abs(lhs - rhs) <= tolerance) {
        return true;
    }

    std::cerr << "[FAIL] " << label << " mismatch: tetrapga=" << std::setprecision(15) << lhs
              << ", coal=" << rhs << ", diff=" << std::abs(lhs - rhs) << std::endl;
    return false;
}

bool expectNearVector(const Eigen::VectorXd& lhs,
                      const Eigen::VectorXd& rhs,
                      const double tolerance,
                      const std::string& label) {
    const double error = (lhs - rhs).norm();
    if (error <= tolerance) {
        return true;
    }

    std::cerr << "[FAIL] " << label << " mismatch, norm=" << error << std::endl;
    std::cerr << "  lhs = " << lhs.transpose() << std::endl;
    std::cerr << "  rhs = " << rhs.transpose() << std::endl;
    return false;
}

std::vector<double> computeCoalDistancesForConfiguration(const Model<double>& model,
                                                         const Environment<double>& env,
                                                         const Eigen::VectorXd& q) {
    Data<double> data(model);
    forwardKinematics(model, data, q);

    std::vector<double> distances;
    distances.reserve(static_cast<size_t>(model.num_collision_ssl * env.num_static_sphere));
    for (int capsule_idx = 0; capsule_idx < model.num_collision_ssl; ++capsule_idx) {
        const int link_id = model.collisionSSL[capsule_idx].id;
        data.SSL_A[capsule_idx] = pga_rbm3(data.Mi.col(link_id), model.collisionSSL[capsule_idx].endpointA);
        data.SSL_B[capsule_idx] = pga_rbm3(data.Mi.col(link_id), model.collisionSSL[capsule_idx].endpointB);
        for (int sphere_idx = 0; sphere_idx < env.num_static_sphere; ++sphere_idx) {
            distances.push_back(computeCoalDistance(model.collisionSSL[capsule_idx],
                                                    data.SSL_A[capsule_idx],
                                                    data.SSL_B[capsule_idx],
                                                    env.static_sphere[sphere_idx]));
        }
    }

    return distances;
}

Eigen::VectorXd finiteDifferenceCoalGradient(const Model<double>& model,
                                             const Environment<double>& env,
                                             const Eigen::VectorXd& q,
                                             const int pair_idx,
                                             const double step) {
    Eigen::VectorXd gradient(model.dof_a);
    for (int joint_idx = 0; joint_idx < model.dof_a; ++joint_idx) {
        Eigen::VectorXd q_plus = q;
        Eigen::VectorXd q_minus = q;
        q_plus[joint_idx] += step;
        q_minus[joint_idx] -= step;

        const auto dist_plus = computeCoalDistancesForConfiguration(model, env, q_plus);
        const auto dist_minus = computeCoalDistancesForConfiguration(model, env, q_minus);
        gradient[joint_idx] = (dist_plus[pair_idx] - dist_minus[pair_idx]) / (2.0 * step);
    }
    return gradient;
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

}  // namespace

int main() {
    const std::string robot_assets_dir = std::string(TETRAPGA_ROBOT_ASSETS_DIR);
    const std::string urdf_path = robot_assets_dir + "/ur10/urdf/ur10.urdf";

    Model<double> model = ur();
    Data<double> data(model);
    Data<double> jac_data(model);

    const Eigen::VectorXd q_test =
        (Eigen::VectorXd(6) << 0.35, -1.35, 1.15, -1.05, 0.55, 0.25).finished();

    const std::vector<SSP<double>> spheres = {
        SSP<double>{0, 0.03, makePoint(0.45, -0.15, 0.18)},
        SSP<double>{0, 0.05, makePoint(0.62, -0.05, 0.12)},
        SSP<double>{0, 0.04, makePoint(0.30, -0.30, 0.35)},
        SSP<double>{0, 0.06, makePoint(0.58, -0.02, 0.02)},
    };

    const Environment<double> env(spheres);
    EnvironmentData<double> env_data(model, env);
    EnvironmentData<double> jac_env_data(model, env);

    constexpr double kDistanceTolerance = 1e-9;
    constexpr double kGradientTolerance = 5e-4;
    constexpr double kPinocchioGradientTolerance = 1e-6;
    constexpr double kFiniteDiffStep = 1e-5;
    bool all_ok = true;

    forwardKinematics(model, data, q_test);
    computeDistance(model, data, env, env_data);

    higherKinematics(model, jac_data, q_test);
    computeDistanceJacobianCache(model, jac_data, env, jac_env_data);

    pinocchio::Model pin_model;
    pinocchio::urdf::buildModel(urdf_path, pin_model);
    if (pin_model.nv != model.dof_a) {
        throw std::runtime_error("Pinocchio/TetraPGA DOF mismatch in collision test.");
    }
    pinocchio::Data pin_data(pin_model);
    pinocchio::forwardKinematics(pin_model, pin_data, q_test);
    pinocchio::computeJointJacobians(pin_model, pin_data, q_test);

    std::cout << std::fixed << std::setprecision(9);
    std::cout << "Test joint configuration q = " << q_test.transpose() << std::endl;

    int pair_idx = 0;
    for (int capsule_idx = 0; capsule_idx < model.num_collision_ssl; ++capsule_idx) {
        for (int sphere_idx = 0; sphere_idx < env.num_static_sphere; ++sphere_idx, ++pair_idx) {
            const double coal_distance =
                computeCoalDistance(model.collisionSSL[capsule_idx],
                                    data.SSL_A[capsule_idx],
                                    data.SSL_B[capsule_idx],
                                    env.static_sphere[sphere_idx]);

            std::cout << "\n[pair " << pair_idx << "] capsule=" << capsule_idx
                      << " sphere=" << sphere_idx << std::endl;
            std::cout << "  distance  TetraPGA = " << env_data.distance[pair_idx]
                      << ", coal = " << coal_distance
                      << ", diff = " << std::abs(env_data.distance[pair_idx] - coal_distance) << std::endl;

            all_ok &= expectNear(env_data.distance[pair_idx],
                                 coal_distance,
                                 kDistanceTolerance,
                                 "distance pair[" + std::to_string(pair_idx) + "]");

            const Eigen::VectorXd fd_gradient =
                finiteDifferenceCoalGradient(model, env, q_test, pair_idx, kFiniteDiffStep);
            const Eigen::VectorXd analytic_gradient = jac_env_data.jac_dist[pair_idx].transpose();

            std::cout << "  grad(TetraPGA) = " << analytic_gradient.transpose() << std::endl;
            std::cout << "  grad(coal FD) = " << fd_gradient.transpose() << std::endl;
            std::cout << "  grad diff norm = " << (analytic_gradient - fd_gradient).norm() << std::endl;

            for (int joint_idx = 0; joint_idx < model.dof_a; ++joint_idx) {
                all_ok &= expectNear(analytic_gradient[joint_idx],
                                     fd_gradient[joint_idx],
                                     kGradientTolerance,
                                     "gradient pair[" + std::to_string(pair_idx) + "] joint[" +
                                         std::to_string(joint_idx) + "]");
            }

            const int link_id = model.collisionSSL[capsule_idx].id;
            const pinocchio::JointIndex joint_id = static_cast<pinocchio::JointIndex>(link_id);
            const CoalWitnessResult witness =
                computeCoalWitness(model.collisionSSL[capsule_idx],
                                   data.SSL_A[capsule_idx],
                                   data.SSL_B[capsule_idx],
                                   env.static_sphere[sphere_idx]);
            const Eigen::MatrixXd point_jacobian =
                computePinocchioPointJacobianWorld(pin_model, pin_data, joint_id, witness.point_capsule);
            const Eigen::VectorXd projected_gradient =
                -point_jacobian.transpose() * witness.normal;

            std::cout << "  grad(Pinocchio/FCL) = " << projected_gradient.transpose() << std::endl;
            std::cout << "  pinocchio diff norm = "
                      << (analytic_gradient - projected_gradient).norm() << std::endl;

            all_ok &= expectNearVector(analytic_gradient,
                                       projected_gradient,
                                       kPinocchioGradientTolerance,
                                       "pinocchio gradient pair[" + std::to_string(pair_idx) + "]");
        }
    }

    if (!all_ok) {
        return 1;
    }

    std::cout << "\nTetraPGA collision distances and gradients match coal finite differences and "
                 "Pinocchio/FCL witness-point Jacobians for "
              << env.num_static_sphere * model.num_collision_ssl << " capsule-sphere queries." << std::endl;
    return 0;
}
