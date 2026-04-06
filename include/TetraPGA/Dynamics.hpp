#pragma once

#include <Eigen/Cholesky>
#include <stdexcept>
#include "TetraPGA/Models.hpp"

namespace TetraPGA {

// Inverse dynamics with external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc, typename DerivedForce>
const VectorXs<Scalar>& inverseDynamics(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedQacc>& ddq, 
  const Eigen::Matrix<DerivedForce, 6, Eigen::Dynamic>& fext)
{
	Line3D<Scalar> t;
	// Forward iterations
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q(i - 1);
    const Scalar dqi = dq(i - 1);
    const Scalar ddqi = ddq(i - 1);
		switch (model.type[i]) {
		case 'R': {
			data.Mi.col(i) = ga_mul_exp_R(model.Lj[i], Scalar(0.5) * qi, model.Mj[i]);
			break;
		}
		case 'P': {
			data.Mi.col(i) = ga_mul_exp_P(model.Lj[i], Scalar(0.5) * qi, model.Mj[i]);
			break;
		}
			default:
				throw std::invalid_argument(
				    "inverseDynamics: unsupported joint type '" + std::string(1, model.type[i]) +
				    "' at index " + std::to_string(i));
			}
		// Calculate velocity and acceleration
    t = ga_AdM(data.Mi.col(i), data.V.col(parent));
    data.V.col(i) = t + dqi * model.Lj[i];
    t = ga_com(model.Lj[i], t);
    data.dV.col(i) = ga_AdM(data.Mi.col(i), data.dV.col(parent)) + dqi * t + ddqi * model.Lj[i];
		// Calculate momentum derivative with external forces
		data.dPi.col(i).noalias() = model.I[i] * data.dV.col(i);
		data.dPi.col(i) += ga_com(model.I[i] * data.V.col(i), data.V.col(i)) - fext.col(i);
	}
	
	// Backward iterations
	for (int i = model.n - 1; i >= 1; --i) {
		data.tau(i-1) = model.Ljstar[i].dot(data.dPi.col(i));
		data.dPi.col(model.parent[i]) += ga_rbm(data.Mi.col(i), data.dPi.col(i));
	}
	return data.tau;
}

// Overload without external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc>
const VectorXs<Scalar>& inverseDynamics(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedQacc>& ddq)
{
	return inverseDynamics(model, data, q, dq, ddq, data.fext_zero);
}

// Inverse dynamics with external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc, typename DerivedForce>
const VectorXs<Scalar>& inverseDynamics0(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedQacc>& ddq, 
  const Eigen::Matrix<DerivedForce, 6, Eigen::Dynamic>& fext)
{
	Line3D<Scalar> Pi;
	// Forward iterations
	for (int i = 1; i < model.n; ++i) {
		const int parent = model.parent[i];
    const Scalar qi = q(i - 1);
    const Scalar dqi = dq(i - 1);
    const Scalar ddqi = ddq(i - 1);
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
				throw std::invalid_argument(
				    "inverseDynamics0: unsupported joint type '" + std::string(1, model.type[i]) +
				    "' at index " + std::to_string(i));
			}
		data.L.col(i) = ga_rbm(data.Mi.col(parent), model.L0[i]);
		data.Lstar.row(i) = ga_metric(data.L.col(i));
		data.V.col(i) = data.V.col(parent) + dqi * data.L.col(i);
		data.dL.col(i) = ga_com(data.L.col(i), data.V.col(parent));
    data.dV.col(i) = data.dV.col(parent) + dqi * data.dL.col(i) + ddqi * data.L.col(i);

		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
		data.hI[i] = ga_rbm(data.M.col(i)) * model.I[i] * ga_AdM(data.M.col(i));
		Pi.noalias() = data.hI[i] * data.V.col(i);
		data.dPi.col(i).noalias() = data.hI[i] * data.dV.col(i);
		data.dPi.col(i) += ga_com(Pi, data.V.col(i)) - ga_rbm(data.M.col(i), fext.col(i));
	}
	
	// Backward iterations
	for (int i = model.n - 1; i >= 1; --i) {
		data.tau(i-1) = data.Lstar.row(i).dot(data.dPi.col(i));
		data.dPi.col(model.parent[i]) += data.dPi.col(i);
	}
	return data.tau;
}

// Overload without external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc>
const VectorXs<Scalar>& inverseDynamics0(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedQacc>& ddq)
{
	return inverseDynamics0(model, data, q, dq, ddq, data.fext_zero);
}

// Forward dynamics with external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedTau, typename DerivedForce>
const VectorXs<Scalar>& forwardDynamics(const Model<Scalar>& model, Data<Scalar>& data,
  const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedTau>& tau, 
  const Eigen::Matrix<DerivedForce, 6, Eigen::Dynamic>& fext)
{
  // Forward pass: kinematics, joint bias and body bias force.
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const auto& Li = model.Lj[i];
    const Scalar qi = q(i - 1);
    const Scalar dqi = dq(i - 1);

		switch (model.type[i]) {
		case 'R': {
			data.Mi.col(i) = ga_mul_exp_R(Li, Scalar(0.5) * qi, model.Mj[i]);
			break;
		}
		case 'P': {
			data.Mi.col(i) = ga_mul_exp_P(Li, Scalar(0.5) * qi, model.Mj[i]);
			break;
		}
			default:
				throw std::invalid_argument(
				    "forwardDynamics: unsupported joint type '" + std::string(1, model.type[i]) +
				    "' at index " + std::to_string(i));
			}
    data.V.col(i) = ga_AdM(data.Mi.col(i), data.V.col(parent)) + dqi * Li;
    data.c.col(i) = dqi * ga_com(Li, data.V.col(i));
		data.Ia[i] = model.I[i];
    data.F.col(i).noalias() = model.I[i] * data.V.col(i);
    data.F.col(i) = ga_com(data.F.col(i), data.V.col(i)) - fext.col(i);
	}

	// Backward pass: articulated body inertia and bias force propagation.
	Eigen::Matrix<Scalar, 6, 6> hI;
	Line3D<Scalar> hF;
	for (int i = model.n - 1; i >= 1; --i) {
    const int parent = model.parent[i];
    const auto& Li = model.Lj[i];

		data.gamma.col(i).noalias() = data.Ia[i] * Li;
		data.gammaT.row(i).noalias() = model.Ljstar[i] * data.Ia[i];
		data.d(i) = Scalar(1) / model.Ljstar[i].dot(data.gamma.col(i));
		data.u(i) = tau(i-1) - model.Ljstar[i].dot(data.F.col(i));
		
		hI.noalias() = data.Ia[i] - data.d(i) * data.gamma.col(i) * data.gammaT.row(i);
		hF.noalias() = data.F.col(i) + hI * data.c.col(i) + data.gamma.col(i) * (data.d(i) * data.u(i));
		data.Ia[parent].noalias() += ga_rbm(data.Mi.col(i)) * hI * ga_AdM(data.Mi.col(i));
		data.F.col(parent) += ga_rbm(data.Mi.col(i), hF);
	}

	// Forward pass: acceleration propagation.
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const auto& Li = model.Lj[i];

		data.dV.col(i) = ga_AdM(data.Mi.col(i), data.dV.col(parent));
    data.dV.col(i) += data.c.col(i);
		data.ddq[i-1] = data.d(i) * (data.u(i) - data.gammaT.row(i).dot(data.dV.col(i)));
		data.dV.col(i) += data.ddq[i-1] * Li;
	}
	return data.ddq;
}

// Overload without external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedTau>
const VectorXs<Scalar>& forwardDynamics(const Model<Scalar>& model, Data<Scalar>& data,
  const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedTau>& tau)
{
	return forwardDynamics(model, data, q, dq, tau, data.fext_zero);
}

// Forward dynamics with external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedTau, typename DerivedForce>
const VectorXs<Scalar>& forwardDynamics0(const Model<Scalar>& model, Data<Scalar>& data,
  const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedTau>& tau, 
  const Eigen::Matrix<DerivedForce, 6, Eigen::Dynamic>& fext)
{
  // Forward pass: kinematics, joint bias and body bias force.
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q(i - 1);
    const Scalar dqi = dq(i - 1);

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
				throw std::invalid_argument(
				    "forwardDynamics0: unsupported joint type '" + std::string(1, model.type[i]) +
				    "' at index " + std::to_string(i));
			}
		data.L.col(i) = ga_rbm(data.Mi.col(parent), model.L0[i]);
		data.Lstar.row(i) = ga_metric(data.L.col(i));
		data.V.col(i) = data.V.col(parent) + dqi * data.L.col(i);
		data.dL.col(i) = ga_com(data.L.col(i), data.V.col(parent));
		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
		data.Ia[i] = ga_rbm(data.M.col(i)) * model.I[i] * ga_AdM(data.M.col(i));
    data.F.col(i) = ga_com(data.Ia[i] * data.V.col(i), data.V.col(i)) - ga_rbm(data.M.col(i), fext.col(i));
	}

	// Backward pass: articulated body inertia and bias force propagation.
	for (int i = model.n - 1; i >= 1; --i) {
    const int parent = model.parent[i];
    const Scalar dqi = dq(i - 1);

		data.gamma.col(i).noalias() = data.Ia[i] * data.L.col(i);
		data.gammaT.row(i).noalias() = data.Lstar.row(i) * data.Ia[i];
		data.d(i) = Scalar(1) / data.Lstar.row(i).dot(data.gamma.col(i));
		data.u(i) = tau(i-1) - data.Lstar.row(i).dot(data.F.col(i)) - data.gammaT.row(i).dot(dqi * data.dL.col(i));
		
		data.Ia[parent].noalias() += data.Ia[i] - data.d(i) * data.gamma.col(i) * data.gammaT.row(i);
		data.F.col(parent) += data.F.col(i) + data.Ia[i] * (dqi * data.dL.col(i)) + data.gamma.col(i) * (data.d(i) * data.u(i));
	}

	// Forward pass: acceleration propagation.
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar dqi = dq(i - 1);
		data.ddq[i-1] = data.d(i) * (data.u(i) - data.gammaT.row(i).dot(data.dV.col(parent)));
		data.dV.col(i) = data.dV.col(parent) + data.ddq[i-1] * data.L.col(i) + dqi * data.dL.col(i);
	}
	return data.ddq;
}

// Overload without external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedTau>
const VectorXs<Scalar>& forwardDynamics0(const Model<Scalar>& model, Data<Scalar>& data,
  const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedTau>& tau)
{
	return forwardDynamics0(model, data, q, dq, tau, data.fext_zero);
}

template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
Eigen::Matrix<typename Derived1::Scalar, 6, 3> ga_vi(const Eigen::MatrixBase<Derived1>& I, const Eigen::MatrixBase<Derived2>& V) {
	using Scalar = typename Derived1::Scalar;
	const Scalar m = I(0, 3);
	const Eigen::Matrix<Scalar, 3, 3> Rc = I.template block<3, 3>(3, 3);
	const Eigen::Matrix<Scalar, 3, 3> J = I.template block<3, 3>(3, 0);
	const Eigen::Matrix<Scalar, 3, 1> omega = V.template head<3>();
	const Eigen::Matrix<Scalar, 3, 3> Omega = skew(omega);
	const Eigen::Matrix<Scalar, 3, 1> v = V.template tail<3>();

	Eigen::Matrix<Scalar, 6, 3> VI;
	const Eigen::Matrix<Scalar, 3, 1> top_term = Rc * omega - m * v;
	VI.template block<3, 3>(0, 0) = skew(Scalar(2) * top_term);
	VI.template block<3, 3>(3, 0) = (Omega * J - J * Omega) - skew(J * omega) - Scalar(2) * Rc * skew(v);
	return VI;
}

// First-order derivative of inverse dynamics with external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc, typename DerivedForce>
void inverseDynamics_fo(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedQacc>& ddq, 
  const Eigen::Matrix<DerivedForce, 6, Eigen::Dynamic>& fext) 
{
	Line3D<Scalar> Pi;
	// Forward iterations
	for (int i = 1; i < model.n; ++i) {
    const int parent = model.parent[i];
    const Scalar qi = q(i - 1);
    const Scalar dqi = dq(i - 1);
    const Scalar ddqi = ddq(i - 1);
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
				throw std::invalid_argument(
				    "inverseDynamics_fo: unsupported joint type '" + std::string(1, model.type[i]) +
				    "' at index " + std::to_string(i));
			}
		data.L.col(i) = ga_rbm(data.Mi.col(parent), model.L0[i]);
		data.Lstar.row(i) = ga_metric(data.L.col(i));
		data.V.col(i) = data.V.col(parent) + dqi * data.L.col(i);
		data.dL.col(i) = ga_com(data.L.col(i), data.V.col(parent));
    data.dV.col(i) = data.dV.col(parent) + dqi * data.dL.col(i) + ddqi * data.L.col(i);
		data.ddL.col(i) = ga_com(data.dL.col(i), data.V.col(parent)) + ga_com(data.L.col(i), data.dV.col(parent));

		data.M.col(i) = ga_mul(model.M0[i], data.Mi.col(i));
		data.hI[i] = ga_rbm(data.M.col(i)) * model.I[i] * ga_AdM(data.M.col(i));
		Pi.noalias() = data.hI[i] * data.V.col(i);
		data.VI[i] = ga_vi(data.hI[i], data.V.col(i));
		data.dPi.col(i).noalias() = data.hI[i] * data.dV.col(i);
		data.dPi.col(i) += ga_com(Pi, data.V.col(i)) - ga_rbm(data.M.col(i), fext.col(i));
	}
	
	// Backward iterations
	for (int i = model.n - 1; i >= 1; --i) {
		const Eigen::Matrix<Scalar, 6, 1> Li = data.L.col(i);
		const Eigen::Matrix<Scalar, 6, 1> dLi = data.dL.col(i);
		const Eigen::Matrix<Scalar, 6, 1> ddLi = data.ddL.col(i);
		const Eigen::Matrix<Scalar, 1, 6> row_hI = data.Lstar.row(i) * data.hI[i];
		const Eigen::Matrix<Scalar, 1, 3> row_VI = data.Lstar.row(i) * data.VI[i];

		const Line3D<Scalar> common_q_self = data.hI[i] * ddLi + data.VI[i] * dLi.head(3);
		const Line3D<Scalar> common_dq_self = Scalar(2) * data.hI[i] * dLi + data.VI[i] * Li.head(3);
		const Line3D<Scalar> common_q_cross = common_q_self + ga_com(data.dPi.col(i), Li);

		for (int idx : model.ancestor[i]) {
			data.ptau_pq(i-1, idx-1) = row_hI.dot(data.ddL.col(idx)) + row_VI.dot(data.dL.col(idx).template head<3>());
			data.ptau_pdq(i-1, idx-1) = Scalar(2) * row_hI.dot(data.dL.col(idx)) + row_VI.dot(data.L.col(idx).template head<3>());
			data.ptau_pddq(i-1, idx-1) = row_hI.dot(data.L.col(idx));
			
			data.ptau_pq(idx-1, i-1) = data.Lstar.row(idx).dot(common_q_cross);
			data.ptau_pdq(idx-1, i-1) = data.Lstar.row(idx).dot(common_dq_self);
			data.ptau_pddq(idx-1, i-1) = data.ptau_pddq(i-1, idx-1);
    }
		data.ptau_pq(i-1, i-1) = row_hI.dot(ddLi) + row_VI.dot(dLi.template head<3>());
		data.ptau_pdq(i-1, i-1) = Scalar(2) * row_hI.dot(dLi) + row_VI.dot(Li.template head<3>());
		data.ptau_pddq(i-1, i-1) = row_hI.dot(Li);

		data.hI[model.parent[i]] += data.hI[i];
		data.VI[model.parent[i]] += data.VI[i];
		data.dPi.col(model.parent[i]) += data.dPi.col(i);
	}
}

// Overload without external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc>
void inverseDynamics_fo(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedQacc>& ddq) 
{
	inverseDynamics_fo(model, data, q, dq, ddq, data.fext_zero);
}

// First-order derivative of forward dynamics with external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedTau, typename DerivedForce>
void forwardDynamics_fo(const Model<Scalar>& model, Data<Scalar>& data,
  const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedTau>& tau, 
  const Eigen::Matrix<DerivedForce, 6, Eigen::Dynamic>& fext) 
{
	forwardDynamics0(model, data, q, dq, tau, fext);
	inverseDynamics_fo(model, data, q, dq, data.ddq, fext);

	if (model.n < 80) {
		data.pddq_ptau.setIdentity();
		data.pddq_ptau = data.ptau_pddq.llt().solve(data.pddq_ptau);
		data.pddq_pq.noalias() = -data.pddq_ptau * data.ptau_pq;
		data.pddq_pdq.noalias() = -data.pddq_ptau * data.ptau_pdq;
		return;
  }
	
	const int dof = model.dof_a;
	data.u_aza.block(0, 0, dof, dof) = -data.ptau_pq;
	data.u_aza.block(0, dof, dof, dof) = -data.ptau_pdq;
	data.pddq_ptau.setIdentity();
	for (int i = 0; i < model.n; ++i) {
		data.F2_aza[i].setZero();
		data.F_aza[i].setZero();
		data.dV2_aza[i].setZero();
		data.dV_aza[i].setZero();
	}

	for (int i = model.n - 1; i >= 1; --i) {
		const int idx = i - 1;
		const int right_cols = dof - idx;
		const int parent = model.parent[i];
		const auto& Li = data.L.col(i);

		data.u_aza.row(idx).noalias() -= data.Lstar.row(i) * data.F2_aza[i];
		data.pddq_ptau.block(idx, idx, 1, right_cols).noalias() -= data.Lstar.row(i) * data.F_aza[i].rightCols(right_cols);
		data.F2_aza[parent].noalias() += data.F2_aza[i] + data.d(i) * data.gamma.col(i) * data.u_aza.row(idx);
		data.F_aza[parent].rightCols(right_cols).noalias() += data.F_aza[i].rightCols(right_cols) + data.d(i) * data.gamma.col(i) * data.pddq_ptau.block(idx, idx, 1, right_cols);
	}

	for (int i = 1; i < model.n; ++i) {
		const int idx = i - 1;
		const int right_cols = dof - idx;
		const int parent = model.parent[i];
		const auto& Li = data.L.col(i);
		data.u_aza.row(idx).noalias() = data.d(i) * (data.u_aza.row(idx) - data.gammaT.row(i) * data.dV2_aza[parent]);
		data.pddq_ptau.block(idx, idx, 1, right_cols).noalias() = data.d(i) * (data.pddq_ptau.block(idx, idx, 1, right_cols) - data.gammaT.row(i) * data.dV_aza[parent].rightCols(right_cols));
		data.dV2_aza[i] = data.dV2_aza[parent] + Li * data.u_aza.row(idx);
		data.dV_aza[i].rightCols(right_cols) = data.dV_aza[parent].rightCols(right_cols) + Li * data.pddq_ptau.block(idx, idx, 1, right_cols);
	}

	data.pddq_ptau.template triangularView<Eigen::StrictlyLower>() = data.pddq_ptau.transpose().template triangularView<Eigen::StrictlyLower>();
	data.pddq_pq = data.u_aza.block(0, 0, dof, dof);
	data.pddq_pdq = data.u_aza.block(0, dof, dof, dof);
}

// Overload without external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedTau>
void forwardDynamics_fo(const Model<Scalar>& model, Data<Scalar>& data,
  const Eigen::MatrixBase<DerivedQ>& q, 
  const Eigen::MatrixBase<DerivedQvel>& dq, 
  const Eigen::MatrixBase<DerivedTau>& tau) 
{
	forwardDynamics_fo(model, data, q, dq, tau, data.fext_zero);
}

}  // namespace TetraPGA
