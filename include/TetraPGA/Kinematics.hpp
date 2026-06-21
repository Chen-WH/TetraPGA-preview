#pragma once

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "TetraPGA/Models.hpp"

namespace TetraPGA {

// Support tree-structure, floating-joint
template <typename Scalar, typename DerivedQ>
void forwardKinematics(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q) 
{
	for (int i = 1; i < model.n; ++i) {
		const int q_idx = model.joint_q_start[i];
		const int parent = model.parent[i];
    const Scalar qi = q[q_idx];
		data.Mi.col(i) = joint::transformFromParent(
		    model.type[i], model.L0[i], qi, data.Mi.col(parent), i, "forwardKinematics");
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
		const int q_idx = model.joint_q_start[i];
    const int parent = model.parent[i];
    const Scalar qi = q[q_idx];
		data.Mi.col(i) = joint::transformFromParent(
		    model.type[i], model.L0[i], qi, data.Mi.col(parent), i, "geometricJacobian");
		data.jac.col(q_idx) = joint::axisFromParent(
		    model.type[i], data.Mi.col(parent), model.L0[i], i, "geometricJacobian");
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
	for (int joint_id = 1; joint_id < model.n; ++joint_id) {
		const int q_begin = model.joint_q_start[joint_id];
		const int q_dof = model.joint_q_dof[joint_id];
		for (int local_idx = 0; local_idx < q_dof; ++local_idx) {
			const int q_idx = q_begin + local_idx;
			if (!std::isfinite(data.q[q_idx])) {
				continue;
			}

			const Scalar lower = model.lowerPositionLimit[q_idx];
			const Scalar upper = model.upperPositionLimit[q_idx];
			const bool is_revolute = model.type[joint_id] == joint::kRevolute;

			if (!is_revolute) {
				if (std::isfinite(lower) || std::isfinite(upper)) {
					data.q[q_idx] = std::min(std::max(data.q[q_idx], lower), upper);
				}
				continue;
			}

			Scalar q_projected = data.q[q_idx];
			bool found_equivalent = false;
			if (std::isfinite(lower) && std::isfinite(upper)) {
				const Scalar k_min = std::ceil((lower - data.q[q_idx]) / two_pi);
				const Scalar k_max = std::floor((upper - data.q[q_idx]) / two_pi);
				if (k_min <= k_max) {
					const Scalar k_ref = std::round((q0[q_idx] - data.q[q_idx]) / two_pi);
					const Scalar k_best = std::min(std::max(k_ref, k_min), k_max);
					q_projected = data.q[q_idx] + k_best * two_pi;
					found_equivalent = true;
				}
			}

			if (!found_equivalent) {
				q_projected = data.q[q_idx] + std::round((q0[q_idx] - data.q[q_idx]) / two_pi) * two_pi;
				if (std::isfinite(lower)) {
					q_projected = std::max(q_projected, lower);
				}
				if (std::isfinite(upper)) {
					q_projected = std::min(q_projected, upper);
				}
			}

			data.q[q_idx] = q_projected;
		}
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
	const Eigen::Matrix<Scalar, 6, 6> JL = Scalar(2) * ga_dexp(-r);
	data.jac = JL.partialPivLu().solve(data.jac);
}

// Support tree-structure
template <typename Scalar, typename DerivedQ>
void motorJacobian(
	const Model<Scalar>& model, Data<Scalar>& data, 
	const Eigen::MatrixBase<DerivedQ>& q) 
{
	for (int i = 1; i < model.n; ++i) {
		const int q_idx = model.joint_q_start[i];
    const int parent = model.parent[i];
    const Scalar qi = q[q_idx];
		data.Mi.col(i) = joint::transformFromParent(
		    model.type[i], model.L0[i], qi, data.Mi.col(parent), i, "motorJacobian");
	}

	const Motor3D<Scalar> M0 = Scalar(0.5) * ga_mul(model.M0[model.n-1], data.Mi.col(model.n-1));
	for (int i = 1; i < model.n; ++i) {
		const int q_idx = model.joint_q_start[i];
		if (!joint::isImplemented(model.type[i])) {
			joint::throwUnimplemented(model.type[i], i, "motorJacobian");
		}
		data.jacM.col(q_idx) =
		    ga_mul(ga_mul(M0, ga_rev(data.Mi.col(i))), ga_prodBM(model.L0[i], data.Mi.col(i)));
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
		const int q_idx = model.joint_q_start[i];
    const int parent = model.parent[i];
    const Scalar qi = q[q_idx];
		data.Mi.col(i) = joint::transformFromParent(
		    model.type[i], model.L0[i], qi, data.Mi.col(parent), i, "higherKinematics(q)");
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
		data.L.col(i) = joint::axisFromParent(
		    model.type[i], data.Mi.col(parent), model.L0[i], i, "higherKinematics(q)");
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
		const int q_idx = model.joint_q_start[i];
    const int parent = model.parent[i];
    const Scalar qi = q[q_idx];
    const Scalar dqi = dq[q_idx];
		data.Mi.col(i) = joint::transformFromParent(
		    model.type[i], model.L0[i], qi, data.Mi.col(parent), i, "higherKinematics(q,dq)");
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
		data.L.col(i) = joint::axisFromParent(
		    model.type[i], data.Mi.col(parent), model.L0[i], i, "higherKinematics(q,dq)");
		data.V.col(i) = joint::spatialVelocity(
		    model.type[i], data.V.col(parent), data.L.col(i), dqi, i, "higherKinematics(q,dq)");
		data.dL.col(i) = joint::axisDot(
		    model.type[i], data.L.col(i), data.V.col(parent), i, "higherKinematics(q,dq)");
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
		const int q_idx = model.joint_q_start[i];
    const int parent = model.parent[i];
    const Scalar qi = q[q_idx];
    const Scalar dqi = dq[q_idx];
    const Scalar ddqi = ddq[q_idx];
		data.Mi.col(i) = joint::transformFromParent(
		    model.type[i], model.L0[i], qi, data.Mi.col(parent), i, "higherKinematics(q,dq,ddq)");
		data.L.col(i) = joint::axisFromParent(
		    model.type[i], data.Mi.col(parent), model.L0[i], i, "higherKinematics(q,dq,ddq)");
		data.V.col(i) = joint::spatialVelocity(
		    model.type[i], data.V.col(parent), data.L.col(i), dqi, i, "higherKinematics(q,dq,ddq)");
		data.dL.col(i) = joint::axisDot(
		    model.type[i], data.L.col(i), data.V.col(parent), i, "higherKinematics(q,dq,ddq)");
    data.dV.col(i) = joint::spatialAcceleration(
        model.type[i], data.dV.col(parent), data.L.col(i), data.dL.col(i), dqi, ddqi, i,
        "higherKinematics(q,dq,ddq)");
		data.ddL.col(i) = joint::axisDDot(
		    model.type[i], data.L.col(i), data.dL.col(i), data.V.col(parent), data.dV.col(parent),
		    i, "higherKinematics(q,dq,ddq)");
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
	}
}

}  // namespace TetraPGA
