#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/Models.hpp"

using namespace TetraPGA;

int main() {
    const std::string robot_assets_dir = std::string(TETRAPGA_ROBOT_ASSETS_DIR);
    const std::string urdf_path = robot_assets_dir + "/leap_hand/urdf/leap_hand_left.urdf";

    Model<double> tetra_model(urdf_path);
    if (tetra_model.n <= 1 || tetra_model.dof_a <= 0) {
        std::cerr << "[FAIL] invalid TetraPGA model loaded from: " << urdf_path << std::endl;
        return 1;
    }

    tetra_model.gravity(5) = -9.81;
    Data<double> tetra_data(tetra_model);

    // Build Pinocchio model from the same URDF
    pinocchio::Model pin_model;
    pinocchio::urdf::buildModel(urdf_path, pin_model);
    pin_model.gravity.linear() << 0.0, 0.0, -9.81;
    pin_model.gravity.angular() << 0.0, 0.0, 0.0;
    pinocchio::Data pin_data(pin_model);

    if (pin_model.nq != pin_model.nv) {
        throw std::runtime_error("This test expects nq == nv for leap_hand.");
    }
    if (pin_model.nv != tetra_model.dof_a) {
        std::cerr << "[FAIL] DOF mismatch: pinocchio nv=" << pin_model.nv
                  << ", TetraPGA dof_a=" << tetra_model.dof_a << std::endl;
        return 2;
    }

    const int n = tetra_model.dof_a;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> unif11(-1.0, 1.0);

    Eigen::VectorXd q(n), dq(n), ddq(n);
    for (int i = 0; i < n; ++i) {
        q(i) = unif11(rng) * 0.25 * M_PI;
        dq(i) = unif11(rng);
        ddq(i) = unif11(rng);
    }

    const Eigen::VectorXd tau_pin = pinocchio::rnea(pin_model, pin_data, q, dq, ddq);
    const Eigen::VectorXd tau_tetra = inverseDynamics(tetra_model, tetra_data, q, dq, ddq);
    const Eigen::VectorXd ddq_tetra = forwardDynamics(tetra_model, tetra_data, q, dq, tau_pin);

    const double tau_err = (tau_pin - tau_tetra).norm();
    const double ddq_err = (ddq - ddq_tetra).norm();

    const double tau_tol = 1e-6;
    const double ddq_tol = 1e-6;
    const bool pass = (tau_err < tau_tol) && (ddq_err < ddq_tol);
    if (!pass) {
        std::cerr << "[FAIL] Models dynamics mismatch: tau_err=" << tau_err
                  << " ddq_err=" << ddq_err << std::endl;
        return 3;
    }

    std::cout << "TetraPGA models tests passed." << std::endl;
    return 0;
}
