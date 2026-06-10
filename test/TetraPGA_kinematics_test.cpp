#include "TetraPGA/Kinematics.hpp"
#include "TetraPGA/ModelRepo.hpp"

#include <iostream>
#include <string>

using namespace TetraPGA;

namespace {

template <typename DerivedA, typename DerivedB>
bool ExpectApprox(const std::string& label,
                  const Eigen::MatrixBase<DerivedA>& lhs,
                  const Eigen::MatrixBase<DerivedB>& rhs,
                  const double tolerance = 1e-6) {
  const double error = (lhs - rhs).norm();
  if (error <= tolerance) {
    return true;
  }

  std::cerr << "[FAIL] " << label << " error=" << error << std::endl;
  return false;
}

}  // namespace

int main() {
  const Model<double> model = ur();
  Data<double> data(model);
  bool ok = true;

  VectorXs<double> q(6);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  forwardKinematics(model, data, q);
  const Motor3D<double> target_motor = data.M.col(model.n - 1);

  geometricJacobian(model, data, q);
  ok &= (data.jac.rows() == 6 && data.jac.cols() == model.dof_a);
  if (!ok) {
    std::cerr << "[FAIL] geometricJacobian shape mismatch." << std::endl;
    return 1;
  }

  VectorXs<double> q0(6);
  q0 << -0.1, 0.15, 0.05, -0.2, 0.25, -0.05;
  const VectorXs<double> q_result = inverseKinematics(model, data, target_motor, q0);

  forwardKinematics(model, data, q_result);
  ok &= ExpectApprox("inverseKinematics end-effector pose",
                     data.M.col(model.n - 1),
                     target_motor,
                     1e-6);

  const Line3D<double> log_motor = ga_log(target_motor);
  analyticJacobian(model, data, q, log_motor);
  ok &= (data.jac.rows() == 6 && data.jac.cols() == model.dof_a);
  if (!ok) {
    std::cerr << "[FAIL] analyticJacobian shape mismatch." << std::endl;
    return 1;
  }

  motorJacobian(model, data, q);
  ok &= (data.jacM.rows() == 8 && data.jacM.cols() == model.dof_a);
  if (!ok) {
    std::cerr << "[FAIL] motorJacobian shape mismatch." << std::endl;
    return 1;
  }

  if (!ok) {
    return 1;
  }

  std::cout << "TetraPGA kinematics tests passed." << std::endl;
  return 0;
}
