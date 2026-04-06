#pragma once

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "TetraPGA/Models.hpp"

namespace TetraPGA {

namespace detail {

[[noreturn]] inline void throwUnsupportedJointType(const char joint_type, const int index, const char* context) {
	throw std::invalid_argument(
	    std::string(context) + ": unsupported joint type '" + std::string(1, joint_type) +
	    "' at index " + std::to_string(index));
}

[[noreturn]] inline void throwFloatingJointUnsupported(const char* context) {
	throw std::logic_error(std::string(context) + ": floating joints require dexp support");
}

}  // namespace detail

// Support tree-structure, floating-joint
template <typename Scalar, typename DerivedQ>
void forwardKinematics(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q) 
{
	for (int i = 1; i < model.n; ++i) {
		const auto& q_id = model.id_map.at(i);
		const int parent = model.parent[i];
    const Scalar qi = q[q_id[0]];
		switch (model.type[i]) {
		case 'R': {
	      data.Mi.col(i) = ga_mul_exp_R(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
				break;
			}
			case 'P': {
	      data.Mi.col(i) = ga_mul_exp_P(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
				break;
			}
			case 'F': {
	      Line3D<Scalar> L_free;
	      L_free << q[q_id[0]], q[q_id[1]], q[q_id[2]], q[q_id[3]], q[q_id[4]], q[q_id[5]];
	      data.Mi.col(i) = ga_mul(ga_exp(Scalar(0.5) * L_free), data.Mi.col(parent));
				break;
			}
		default:
			detail::throwUnsupportedJointType(model.type[i], i, "forwardKinematics");
		}
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
	}
}

// Support tree-structure
template <typename Scalar, typename DerivedQ>
void geometricJacobian(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q) 
{
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q[i - 1];
		switch (model.type[i]) {
		case 'R': {
      data.Mi.col(i) = ga_mul_exp_R(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			data.jac.col(i-1) = ga_rbm(data.Mi.col(parent), model.L0[i]);
			break;
		}
		case 'P': {
      data.Mi.col(i) = ga_mul_exp_P(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			data.jac.col(i-1) = ga_rbm(data.Mi.col(parent), model.L0[i]);
			break;
		}
		case 'F': {
      detail::throwFloatingJointUnsupported("geometricJacobian");
		}
		default:
			detail::throwUnsupportedJointType(model.type[i], i, "geometricJacobian");
		}
	}
}

// Support tree-structure
template <typename Scalar, typename DerivedQ>
VectorXs<Scalar> inverseKinematics(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Motor3D<Scalar>& M, 
	const Eigen::MatrixBase<DerivedQ>& q0,
	const int max_iterations = 30,
	const Scalar tolerance = Scalar(1e-7),
	const Scalar max_step_norm = Scalar(1.0)) 
{
	data.q = q0;
	const Motor3D<Scalar> iM = ga_rev(M);
	Line3D<Scalar> e;
	e.setOnes();
	MatrixXs<Scalar> JtJ(model.dof_a, model.dof_a);
	VectorXs<Scalar> rhs(model.dof_a);
	VectorXs<Scalar> delta_q(model.dof_a);
	int iter = 0;
	while (e.norm() > tolerance) {
		forwardKinematics(model, data, data.q);
		e = ga_log(ga_mul(iM, data.M.col(model.n - 1)));
		geometricJacobian(model, data, data.q);
		JtJ.noalias() = data.jac.transpose() * data.jac;
		JtJ.diagonal().array() += 1e-9;
		rhs.noalias() = data.jac.transpose() * e;
		delta_q = JtJ.ldlt().solve(rhs);
		const Scalar delta_norm = delta_q.norm();
		if (delta_norm > max_step_norm) {
			delta_q *= max_step_norm / delta_norm;
		}
		data.q -= delta_q;
		iter++;
		if (iter >= max_iterations) {
			break;
		}
	}

	const Scalar two_pi = Scalar(2) * Scalar(M_PI);
	for (int i = 0; i < model.dof_a; ++i) {
		if (!std::isfinite(data.q[i])) {
			continue;
		}

		const Scalar lower = model.lowerPositionLimit[i];
		const Scalar upper = model.upperPositionLimit[i];
		const int model_joint_idx = i + 1;
		const bool is_revolute =
			model_joint_idx < static_cast<int>(model.type.size()) && model.type[model_joint_idx] == 'R';

		if (!is_revolute) {
			if (std::isfinite(lower) || std::isfinite(upper)) {
				data.q[i] = std::min(std::max(data.q[i], lower), upper);
			}
			continue;
		}

		Scalar q_projected = data.q[i];
		bool found_equivalent = false;
		if (std::isfinite(lower) && std::isfinite(upper)) {
			const Scalar k_min = std::ceil((lower - data.q[i]) / two_pi);
			const Scalar k_max = std::floor((upper - data.q[i]) / two_pi);
			if (k_min <= k_max) {
				const Scalar k_ref = std::round((q0[i] - data.q[i]) / two_pi);
				const Scalar k_best = std::min(std::max(k_ref, k_min), k_max);
				q_projected = data.q[i] + k_best * two_pi;
				found_equivalent = true;
			}
		}

		if (!found_equivalent) {
			q_projected = data.q[i] + std::round((q0[i] - data.q[i]) / two_pi) * two_pi;
			if (std::isfinite(lower)) {
				q_projected = std::max(q_projected, lower);
			}
			if (std::isfinite(upper)) {
				q_projected = std::min(q_projected, upper);
			}
		}

		data.q[i] = q_projected;
	}
	return data.q;
}

// Support tree-structure
template <typename Scalar, typename DerivedQ>
void analyticJacobian(
	const Model<Scalar>& model, Data<Scalar>& data, 
	const Eigen::MatrixBase<DerivedQ>& q, 
	const Line3D<Scalar>& r) 
{
	geometricJacobian(model, data, q);
	const Eigen::Matrix<Scalar, 6, 6> JL = Scalar(2) * ga_dexp(r);
	data.jac = JL.partialPivLu().solve(data.jac);
}

// Support tree-structure
template <typename Scalar, typename DerivedQ>
void motorJacobian(
	const Model<Scalar>& model, Data<Scalar>& data, 
	const Eigen::MatrixBase<DerivedQ>& q) 
{
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q[i - 1];
		switch (model.type[i]) {
		case 'R': {
			data.Mi.col(i) = ga_mul_exp_R(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		case 'P': {
			data.Mi.col(i) = ga_mul_exp_P(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		case 'F': {
			detail::throwFloatingJointUnsupported("motorJacobian");
		}
		default:
			detail::throwUnsupportedJointType(model.type[i], i, "motorJacobian");
		}
	}

	const Motor3D<Scalar> M0 = Scalar(0.5) * ga_mul(model.M0[model.n-1], data.Mi.col(model.n-1));
	for (int i = 1; i < model.n; ++i) {
		data.jacM.col(i-1) = ga_mul(ga_mul(M0, ga_rev(data.Mi.col(i))), ga_prodBM(model.L0[i], data.Mi.col(i)));
	}
}

// Higher-order kinematics for zero derivatives
template <typename Scalar, typename DerivedQ>
void higherKinematics(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q) 
{
	// Forward iterations
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q[i - 1];
		switch (model.type[i]) {
		case 'R': {
			data.Mi.col(i) = ga_mul_exp_R(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		case 'P': {
			data.Mi.col(i) = ga_mul_exp_P(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		default:
			detail::throwUnsupportedJointType(model.type[i], i, "higherKinematics(q)");
		}
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
		data.L.col(i) = ga_rbm(data.Mi.col(parent), model.L0[i]);
	}
}

// Higher-order kinematics for first derivatives
template <typename Scalar, typename DerivedQ, typename DerivedQvel>
void higherKinematics(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq) 
{
	// Forward iterations
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q[i - 1];
    const Scalar dqi = dq[i - 1];
		switch (model.type[i]) {
		case 'R': {
			data.Mi.col(i) = ga_mul_exp_R(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		case 'P': {
			data.Mi.col(i) = ga_mul_exp_P(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		default:
			detail::throwUnsupportedJointType(model.type[i], i, "higherKinematics(q,dq)");
		}
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
		data.L.col(i) = ga_rbm(data.Mi.col(parent), model.L0[i]);
		data.V.col(i) = data.V.col(parent) + dqi * data.L.col(i);
		data.dL.col(i) = ga_com(data.L.col(i), data.V.col(parent));
	}
}

// Higher-order kinematics for second derivatives
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc>
void higherKinematics(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedQacc>& ddq) 
{
	// Forward iterations
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q[i - 1];
    const Scalar dqi = dq[i - 1];
    const Scalar ddqi = ddq[i - 1];
		switch (model.type[i]) {
		case 'R': {
			data.Mi.col(i) = ga_mul_exp_R(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		case 'P': {
			data.Mi.col(i) = ga_mul_exp_P(model.L0[i], Scalar(0.5) * qi, data.Mi.col(parent));
			break;
		}
		default:
			detail::throwUnsupportedJointType(model.type[i], i, "higherKinematics(q,dq,ddq)");
		}
		data.L.col(i) = ga_rbm(data.Mi.col(parent), model.L0[i]);
		data.V.col(i) = data.V.col(parent) + dqi * data.L.col(i);
		data.dL.col(i) = ga_com(data.L.col(i), data.V.col(parent));
    data.dV.col(i) = data.dV.col(parent) + dqi * data.dL.col(i) + ddqi * data.L.col(i);
		data.ddL.col(i) = ga_com(data.dL.col(i), data.V.col(parent)) + ga_com(data.L.col(i), data.dV.col(parent));
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
	}
}

}  // namespace TetraPGA
