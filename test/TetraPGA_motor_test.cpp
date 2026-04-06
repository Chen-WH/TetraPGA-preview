#include "TetraPGA/Motor.hpp"

#include <cmath>
#include <iostream>
#include <string>

using namespace TetraPGA;

namespace {

template <typename DerivedA, typename DerivedB>
bool ExpectApprox(const std::string& label,
                  const Eigen::MatrixBase<DerivedA>& lhs,
                  const Eigen::MatrixBase<DerivedB>& rhs,
                  const double tolerance = 1e-9) {
  const double error = (lhs - rhs).norm();
  if (error <= tolerance) {
    return true;
  }

  std::cerr << "[FAIL] " << label << " error=" << error << std::endl;
  return false;
}

}  // namespace

int main() {
  bool ok = true;
  constexpr double kFiniteDifferenceStep = 1e-9;

  Line3D<> body_twist;
  body_twist << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  const Motor3D<> motor = ga_exp(body_twist);
  const Line3D<> recovered_twist = ga_log(motor);
  ok &= ExpectApprox("log(exp(B))", recovered_twist, body_twist);

  Motor3D<> identity;
  identity << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ok &= ExpectApprox("M * rev(M)", ga_mul(motor, ga_rev(motor)), identity);

  Line3D<> velocity_body;
  velocity_body << 0.680375, -0.211234, 0.566198, 0.596880, 0.823295, 0.604897;

  const auto motor_at = [&](const double t) -> Motor3D<> {
    return ga_mul(ga_exp(0.5 * t * velocity_body), motor);
  };

  const Motor3D<> motor_mid = motor_at(0.5);
  const Motor3D<> dmotor_fd =
      0.5 * (motor_at(0.5 + kFiniteDifferenceStep) -
             motor_at(0.5 - kFiniteDifferenceStep)) /
      kFiniteDifferenceStep;
  ok &= ExpectApprox("0.5 * B * M", dmotor_fd, 0.5 * ga_prodBM(velocity_body, motor_mid), 1e-6);

  const Line3D<> velocity_spatial = ga_rbm(motor_mid, velocity_body);
  ok &= ExpectApprox("0.5 * M * Bs", dmotor_fd, 0.5 * ga_prodMB(motor_mid, velocity_spatial), 1e-6);

  ok &= ExpectApprox("rbm(M) * B", ga_rbm(motor_mid) * velocity_body, velocity_spatial);
  ok &= ExpectApprox("Ad(M) * Bs", ga_AdM(motor_mid) * velocity_spatial, velocity_body);

  Line3D<> second_twist;
  second_twist << 0.05, 0.1, 0.15, 0.005, 0.01, 0.015;
  const Line3D<> commutator = ga_com(body_twist, second_twist);
  ok &= ExpectApprox("commutator matrix form", ga_com(body_twist) * second_twist, commutator);
  ok &= ExpectApprox("ad operator", ga_adB(body_twist) * second_twist, ga_adB(body_twist, second_twist));

  const Line3D<> random_twist = Line3D<>::Random();
  Eigen::Matrix<double, 6, 6> dexp_fd;
  for (int i = 0; i < 6; ++i) {
    Line3D<> perturbation = Line3D<>::Zero();
    perturbation(i) = kFiniteDifferenceStep;
    dexp_fd.col(i) = ga_log(ga_mul(ga_exp(random_twist + perturbation), ga_exp(-random_twist))) /
                     kFiniteDifferenceStep;
  }
  ok &= ExpectApprox("dexp", ga_dexp(random_twist), dexp_fd, 1e-6);

  if (!ok) {
    return 1;
  }

  std::cout << "TetraPGA motor tests passed." << std::endl;
  return 0;
}
