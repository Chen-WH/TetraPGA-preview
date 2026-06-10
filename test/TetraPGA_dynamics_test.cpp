#include <iostream>

#include "TetraPGA/Dynamics.hpp"
#include "TetraPGA/ModelRepo.hpp"

using namespace TetraPGA;

namespace {

bool ExpectApprox(const Eigen::VectorXd& lhs,
                  const Eigen::VectorXd& rhs,
                  const double tolerance,
                  const std::string& label) {
  const double error = (lhs - rhs).norm();
  if (error <= tolerance) {
    return true;
  }

  std::cerr << "[FAIL] " << label << " error=" << error << std::endl;
  std::cerr << "  lhs = " << lhs.transpose() << std::endl;
  std::cerr << "  rhs = " << rhs.transpose() << std::endl;
  return false;
}

bool ExpectLessEqual(const double value, const double tolerance, const std::string& label) {
  if (value <= tolerance) {
    return true;
  }

  std::cerr << "[FAIL] " << label << " value=" << value << " tolerance=" << tolerance << std::endl;
  return false;
}

Eigen::MatrixXd SliceTensorByThirdIndex(const std::vector<Eigen::MatrixXd>& tensor, const int third_index) {
  const int dof = static_cast<int>(tensor.size());
  Eigen::MatrixXd slice = Eigen::MatrixXd::Zero(dof, dof);
  for (int tau_idx = 0; tau_idx < dof; ++tau_idx) {
    slice.row(tau_idx) = tensor[static_cast<std::size_t>(tau_idx)].col(third_index).transpose();
  }
  return slice;
}

}  // namespace

int main() {
  Model<double> model = ur();
  Data<double> data(model);

  Eigen::VectorXd q(6);
  q << -1.2006, 1.4207, 1.7773, 1.2176, -3.0800, 2.1565;

  Eigen::VectorXd dq(6);
  dq << 4.2233, 2.7095, -4.5734, -1.2181, 2.0434, 2.2951;

  Eigen::VectorXd ddq(6);
  ddq << -2.7572, -2.3095, 1.7303, -0.2251, 1.2372, -2.6356;

  Eigen::Matrix<double, 6, Eigen::Dynamic> fext(6, model.n);
  fext << 0.0000, 0.8794, 0.5041, 0.1586, 0.1119, 0.0774, 0.3095,
      0.0000, 0.3117, 0.7308, 0.5712, 0.9846, 0.7225, 0.4805,
      0.0000, 0.3914, 0.1333, 0.5646, 0.7365, 0.6728, 0.1969,
      0.0000, 0.5105, 0.5751, 0.0680, 0.2575, 0.7776, 0.0670,
      0.0000, 0.2517, 0.7743, 0.0844, 0.1812, 0.7161, 0.6654,
      0.0000, 0.2004, 0.4559, 0.8587, 0.3871, 0.0232, 0.6683;

  inverseDynamics(model, data, q, dq, ddq, fext);
  const Eigen::VectorXd tau = data.tau;

  bool ok = true;

  inverseDynamics0(model, data, q, dq, ddq, fext);
  ok &= ExpectApprox(tau, data.tau, 1e-6, "inverseDynamics0");

  forwardDynamics(model, data, q, dq, tau, fext);
  ok &= ExpectApprox(ddq, data.ddq, 1e-6, "forwardDynamics");

  forwardDynamics0(model, data, q, dq, tau, fext);
  ok &= ExpectApprox(ddq, data.ddq, 1e-6, "forwardDynamics0");

  inverseDynamics_fo(model, data, q, dq, ddq, fext);

  const double step = 1e-9;
  Eigen::MatrixXd ptau_pq_fd = Eigen::MatrixXd::Zero(model.dof_a, model.dof_a);
  Eigen::MatrixXd ptau_pdq_fd = Eigen::MatrixXd::Zero(model.dof_a, model.dof_a);
  Eigen::MatrixXd ptau_pddq_fd = Eigen::MatrixXd::Zero(model.dof_a, model.dof_a);

  Eigen::VectorXd q_tmp(6);
  Eigen::VectorXd dq_tmp(6);
  Eigen::VectorXd ddq_tmp(6);

  for (int i = 0; i < model.dof_a; ++i) {
    q_tmp = q;
    q_tmp(i) += step;
    inverseDynamics(model, data, q_tmp, dq, ddq, fext);
    ptau_pq_fd.col(i) = (data.tau - tau) / step;
  }

  for (int i = 0; i < model.dof_a; ++i) {
    dq_tmp = dq;
    dq_tmp(i) += step;
    inverseDynamics(model, data, q, dq_tmp, ddq, fext);
    ptau_pdq_fd.col(i) = (data.tau - tau) / step;
  }

  for (int i = 0; i < model.dof_a; ++i) {
    ddq_tmp = ddq;
    ddq_tmp(i) += step;
    inverseDynamics(model, data, q, dq, ddq_tmp, fext);
    ptau_pddq_fd.col(i) = (data.tau - tau) / step;
  }

  ok &= ExpectLessEqual((data.ptau_pq - ptau_pq_fd).norm(), 1e-3, "inverseDynamics_fo ptau_pq");
  ok &= ExpectLessEqual((data.ptau_pdq - ptau_pdq_fd).norm(), 1e-3, "inverseDynamics_fo ptau_pdq");
  ok &= ExpectLessEqual((data.ptau_pddq - ptau_pddq_fd).norm(), 1e-3, "inverseDynamics_fo ptau_pddq");

  const Eigen::MatrixXd ptau_pq_so = data.ptau_pq;
  const Eigen::MatrixXd ptau_pdq_so = data.ptau_pdq;
  const Eigen::MatrixXd ptau_pddq_so = data.ptau_pddq;

  forwardDynamics_fo(model, data, q, dq, tau, fext);

  Eigen::MatrixXd pddq_ptau_fd = Eigen::MatrixXd::Zero(model.dof_a, model.dof_a);
  Eigen::MatrixXd pddq_pq_fd = Eigen::MatrixXd::Zero(model.dof_a, model.dof_a);
  Eigen::MatrixXd pddq_pdq_fd = Eigen::MatrixXd::Zero(model.dof_a, model.dof_a);
  Eigen::VectorXd tau_tmp(6);

  for (int i = 0; i < model.dof_a; ++i) {
    q_tmp = q;
    q_tmp(i) += step;
    forwardDynamics(model, data, q_tmp, dq, tau, fext);
    pddq_pq_fd.col(i) = (data.ddq - ddq) / step;
  }

  for (int i = 0; i < model.dof_a; ++i) {
    dq_tmp = dq;
    dq_tmp(i) += step;
    forwardDynamics(model, data, q, dq_tmp, tau, fext);
    pddq_pdq_fd.col(i) = (data.ddq - ddq) / step;
  }

  for (int i = 0; i < model.dof_a; ++i) {
    tau_tmp = tau;
    tau_tmp(i) += step;
    forwardDynamics(model, data, q, dq, tau_tmp, fext);
    pddq_ptau_fd.col(i) = (data.ddq - ddq) / step;
  }

  ok &= ExpectLessEqual((data.pddq_ptau - pddq_ptau_fd).norm(), 1e-3, "forwardDynamics_fo pddq_ptau");
  ok &= ExpectLessEqual((data.pddq_pq - pddq_pq_fd).norm(), 1e-3, "forwardDynamics_fo pddq_pq");
  ok &= ExpectLessEqual((data.pddq_pdq - pddq_pdq_fd).norm(), 1e-3, "forwardDynamics_fo pddq_pdq");

  inverseDynamics_so(model, data, q, dq, ddq, fext);

  const std::vector<Eigen::MatrixXd> p2tau_pqpq = data.p2tau_pqpq;
  const std::vector<Eigen::MatrixXd> p2tau_pdqpq = data.p2tau_pdqpq;
  const std::vector<Eigen::MatrixXd> p2tau_pdqpdq = data.p2tau_pdqpdq;
  const std::vector<Eigen::MatrixXd> p2tau_pqpddq = data.p2tau_pqpddq;

  const double second_step = 1e-7;
  for (int i = 0; i < model.dof_a; ++i) {
    q_tmp = q;
    q_tmp(i) += second_step;
    inverseDynamics_fo(model, data, q_tmp, dq, ddq, fext);
    const Eigen::MatrixXd p2tau_pqpq_fd = (data.ptau_pq - ptau_pq_so) / second_step;
    const Eigen::MatrixXd p2tau_pqpddq_fd = (data.ptau_pddq - ptau_pddq_so) / second_step;
    ok &= ExpectLessEqual((SliceTensorByThirdIndex(p2tau_pqpq, i) - p2tau_pqpq_fd).norm(),
                          2e-2,
                          "inverseDynamics_so p2tau_pqpq axis " + std::to_string(i));
    ok &= ExpectLessEqual((SliceTensorByThirdIndex(p2tau_pqpddq, i) - p2tau_pqpddq_fd).norm(),
                          2e-2,
                          "inverseDynamics_so p2tau_pqpddq axis " + std::to_string(i));
  }

  for (int i = 0; i < model.dof_a; ++i) {
    dq_tmp = dq;
    dq_tmp(i) += second_step;
    inverseDynamics_fo(model, data, q, dq_tmp, ddq, fext);
    const Eigen::MatrixXd p2tau_pdqpq_fd = (data.ptau_pq - ptau_pq_so) / second_step;
    const Eigen::MatrixXd p2tau_pdqpdq_fd = (data.ptau_pdq - ptau_pdq_so) / second_step;
    ok &= ExpectLessEqual((SliceTensorByThirdIndex(p2tau_pdqpq, i) - p2tau_pdqpq_fd).norm(),
                          2e-2,
                          "inverseDynamics_so p2tau_pdqpq axis " + std::to_string(i));
    ok &= ExpectLessEqual((SliceTensorByThirdIndex(p2tau_pdqpdq, i) - p2tau_pdqpdq_fd).norm(),
                          2e-2,
                          "inverseDynamics_so p2tau_pdqpdq axis " + std::to_string(i));
  }

  if (!ok) {
    return 1;
  }

  std::cout << "TetraPGA dynamics tests passed." << std::endl;
  return 0;
}
