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
Eigen::Matrix<typename Derived1::Scalar, 6, 3> ga_coriolis(const Eigen::MatrixBase<Derived1>& I, const Eigen::MatrixBase<Derived2>& V) {
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
		data.VI[i] = ga_coriolis(data.hI[i], data.V.col(i));
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

		const Line3D<Scalar> Eq1 = data.hI[i] * ddLi + data.VI[i] * dLi.head(3) + ga_com(data.dPi.col(i), Li);
		const Line3D<Scalar> Eq2 = Scalar(2) * data.hI[i] * dLi + data.VI[i] * Li.head(3);

		for (int idx : model.ancestor[i]) {
			data.ptau_pq(i-1, idx-1) = row_hI.dot(data.ddL.col(idx)) + row_VI.dot(data.dL.col(idx).template head<3>());
			data.ptau_pdq(i-1, idx-1) = Scalar(2) * row_hI.dot(data.dL.col(idx)) + row_VI.dot(data.L.col(idx).template head<3>());
			data.ptau_pddq(i-1, idx-1) = row_hI.dot(data.L.col(idx));
			
			data.ptau_pq(idx-1, i-1) = data.Lstar.row(idx).dot(Eq1);
			data.ptau_pdq(idx-1, i-1) = data.Lstar.row(idx).dot(Eq2);
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

// Second-order derivative of inverse dynamics with external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc, typename DerivedForce>
void inverseDynamics_so(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q,
  const Eigen::MatrixBase<DerivedQvel>& dq,
  const Eigen::MatrixBase<DerivedQacc>& ddq,
  const Eigen::Matrix<DerivedForce, 6, Eigen::Dynamic>& fext)
{
	Line3D<Scalar> Pi;
	auto calBVLC = [](const Eigen::Matrix<Scalar, 6, 3>& Bv, const Line3D<Scalar>& L) {
		const Eigen::Matrix<Scalar, 3, 3> Bv1 = Bv.template topRows<3>();
		const Eigen::Matrix<Scalar, 3, 3> Bv2 = Bv.template bottomRows<3>();
		const Eigen::Matrix<Scalar, 3, 3> w = skew(L.template head<3>());
		const Eigen::Matrix<Scalar, 3, 3> v = skew(L.template tail<3>());

		Eigen::Matrix<Scalar, 6, 3> out;
		out.template topRows<3>().noalias() = w * Bv1 - Bv1 * w;
		out.template bottomRows<3>().noalias() = w * Bv2 - Bv2 * w + v * Bv1;
		return out;
	};
	auto rowCoriolis = [](const Scalar m, const Eigen::Matrix<Scalar, 3, 3>& Rc,
	                      const Eigen::Matrix<Scalar, 3, 3>& J, const Line3D<Scalar>& v,
	                      const Eigen::Matrix<Scalar, 1, 6>& y) {
		const Eigen::Matrix<Scalar, 3, 1> r = v.template head<3>();
		const Eigen::Matrix<Scalar, 3, 1> t = v.template tail<3>();
		const Eigen::Matrix<Scalar, 3, 1> y1 = y.template head<3>().transpose();
		const Eigen::Matrix<Scalar, 3, 1> y2 = y.template tail<3>().transpose();

		const Eigen::Matrix<Scalar, 3, 1> out =
		    Scalar(2) * y1.cross(Rc * r) - Scalar(2) * m * y1.cross(t) +
		    J.transpose() * y2.cross(r) - (J.transpose() * y2).cross(r) -
		    y2.cross(J * r) - Scalar(2) * (Rc.transpose() * y2).cross(t);
		const Eigen::Matrix<Scalar, 1, 3> row = out.transpose();
		return row;
	};
	auto rowCom = [](const Eigen::Matrix<Scalar, 1, 6>& y, const Line3D<Scalar>& L) {
		const Eigen::Matrix<Scalar, 3, 1> w = L.template head<3>();
		const Eigen::Matrix<Scalar, 3, 1> v = L.template tail<3>();
		const Eigen::Matrix<Scalar, 3, 1> y1 = y.template head<3>().transpose();
		const Eigen::Matrix<Scalar, 3, 1> y2 = y.template tail<3>().transpose();

		Eigen::Matrix<Scalar, 1, 6> out;
		out.template head<3>() = (w.cross(y1) + v.cross(y2)).transpose();
		out.template tail<3>() = w.cross(y2).transpose();
		return out;
	};
	auto rowComTopLeft = [](const Eigen::Matrix<Scalar, 1, 3>& y, const Line3D<Scalar>& L) {
		const Eigen::Matrix<Scalar, 3, 1> w = L.template head<3>();
		const Eigen::Matrix<Scalar, 3, 1> yv = y.transpose();
		const Eigen::Matrix<Scalar, 1, 3> out = w.cross(yv).transpose();
		return out;
	};
	for (int idx = 0; idx < model.dof_a; ++idx) {
		data.p2tau_pqpq[idx].setZero();
		data.p2tau_pdqpq[idx].setZero();
		data.p2tau_pdqpdq[idx].setZero();
		data.p2tau_pqpddq[idx].setZero();
	}

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
				    "inverseDynamics_so: unsupported joint type '" + std::string(1, model.type[i]) +
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
		data.VI[i] = ga_coriolis(data.hI[i], data.V.col(i));
		data.dPi.col(i).noalias() = data.hI[i] * data.dV.col(i);
		data.dPi.col(i) += ga_com(Pi, data.V.col(i)) - ga_rbm(data.M.col(i), fext.col(i));
	}

	// Backward iterations
	for (int k = model.n - 1; k >= 1; --k) {
		const int k_idx = k - 1;
		const int parent = model.parent[k];
		const Eigen::Matrix<Scalar, 6, 1> Lk = data.L.col(k);
		const Eigen::Matrix<Scalar, 6, 1> dLk = data.dL.col(k);
		const Eigen::Matrix<Scalar, 6, 1> ddLk = data.ddL.col(k);
		const Eigen::Matrix<Scalar, 1, 6> row_k = data.Lstar.row(k);
		const Eigen::Matrix<Scalar, 6, 6> hIk = data.hI[k];
		const Eigen::Matrix<Scalar, 6, 3> VIk = data.VI[k];
		const Eigen::Matrix<Scalar, 6, 1> dPik = data.dPi.col(k);
		const Eigen::Matrix<Scalar, 1, 6> row_hI = row_k * hIk;
		const Eigen::Matrix<Scalar, 1, 3> row_VI = row_k * VIk;
		const Scalar hIk_m = hIk(0, 3);
		const Eigen::Matrix<Scalar, 3, 3> hIk_Rc = hIk.template block<3, 3>(3, 3);
		const Eigen::Matrix<Scalar, 3, 3> hIk_J = hIk.template block<3, 3>(3, 0);

		const Eigen::Matrix<Scalar, 6, 6> Lkc = ga_com(Lk);
		const Eigen::Matrix<Scalar, 6, 6> I_Lc = hIk * Lkc - Lkc * hIk;
		const Eigen::Matrix<Scalar, 6, 3> BdL = calBVLC(VIk, Lk) + ga_coriolis(hIk, dLk);
		const Line3D<Scalar> I_ddL = hIk * ddLk + VIk * dLk.template head<3>() + ga_com(dPik, Lk);
		const Eigen::Matrix<Scalar, 6, 3> BL = ga_coriolis(hIk, Lk);
		const Line3D<Scalar> I_dL = Scalar(2) * hIk * dLk + VIk * Lk.template head<3>();
		const Line3D<Scalar> I_L = hIk * Lk;

		int j = k;
		while (j > 0) {
			const int j_idx = j - 1;
			const Line3D<Scalar> Lj = data.L.col(j);
			const Line3D<Scalar> dLj = data.dL.col(j);
			const Line3D<Scalar> ddLj = data.ddL.col(j);
			const Eigen::Matrix<Scalar, 1, 6> row_j = data.Lstar.row(j);

			const Eigen::Matrix<Scalar, 1, 6> Eq1 = rowCom(row_hI, Lj);
			const Eigen::Matrix<Scalar, 1, 3> Eq2 = rowComTopLeft(row_VI, Lj) + rowCoriolis(hIk_m, hIk_Rc, hIk_J, dLj, row_k);
			const Eigen::Matrix<Scalar, 1, 6> Eq3 = row_j * I_Lc;
			const Eigen::Matrix<Scalar, 1, 3> Eq4 = row_j * BdL;
			const Line3D<Scalar> Eq5 = I_Lc * ddLj + BdL * dLj.template head<3>() + ga_com(I_ddL, Lj);
			const Eigen::Matrix<Scalar, 1, 3> Eq6 = rowCoriolis(hIk_m, hIk_Rc, hIk_J, Lj, row_k);
			const Eigen::Matrix<Scalar, 1, 3> Eq7 = row_j * BL;
			const Line3D<Scalar> Eq8 = Scalar(2) * I_Lc * dLj + BdL * Lj.template head<3>();
			const Line3D<Scalar> Eq9 = BL * dLj.template head<3>() + ga_com(I_dL, Lj);
			const Line3D<Scalar> Eq10 = BL * Lj.template head<3>();
			const Line3D<Scalar> Eq11 = ga_com(I_L, Lj);
			const Line3D<Scalar> Eq12 = I_Lc * Lj;

			int i = j;
			while (i > 0) {
				const int i_idx = i - 1;
				const Line3D<Scalar> Li = data.L.col(i);
				const Line3D<Scalar> dLi = data.dL.col(i);
				const Line3D<Scalar> ddLi = data.ddL.col(i);

				data.p2tau_pqpq[k_idx](j_idx, i_idx) = Eq1.dot(ddLi) + Eq2.dot(dLi.template head<3>());
				data.p2tau_pdqpq[k_idx](j_idx, i_idx) =
				    Scalar(2) * Eq1.dot(dLi) + Eq2.dot(Li.template head<3>());
				data.p2tau_pdqpdq[k_idx](j_idx, i_idx) = Eq6.dot(Li.template head<3>());

				if (j != k) {
					data.p2tau_pqpq[j_idx](k_idx, i_idx) =
					    Eq3.dot(ddLi) + Eq4.dot(dLi.template head<3>());
					data.p2tau_pdqpq[j_idx](k_idx, i_idx) =
					    Scalar(2) * Eq3.dot(dLi) + Eq4.dot(Li.template head<3>());
					data.p2tau_pdqpdq[j_idx](k_idx, i_idx) = Eq7.dot(Li.template head<3>());

					data.p2tau_pqpq[j_idx](i_idx, k_idx) = data.p2tau_pqpq[j_idx](k_idx, i_idx);
					data.p2tau_pdqpq[j_idx](i_idx, k_idx) = Eq7.dot(dLi.template head<3>());
					data.p2tau_pdqpdq[j_idx](i_idx, k_idx) = data.p2tau_pdqpdq[j_idx](k_idx, i_idx);
					data.p2tau_pqpddq[j_idx](i_idx, k_idx) = Eq3.dot(Li);
				}
				if (i != j) {
					const Eigen::Matrix<Scalar, 1, 6> row_i = data.Lstar.row(i);

					data.p2tau_pqpq[k_idx](i_idx, j_idx) = data.p2tau_pqpq[k_idx](j_idx, i_idx);
					data.p2tau_pdqpq[k_idx](i_idx, j_idx) = Eq6.dot(dLi.template head<3>());
					data.p2tau_pdqpdq[k_idx](i_idx, j_idx) = data.p2tau_pdqpdq[k_idx](j_idx, i_idx);
					data.p2tau_pqpddq[k_idx](i_idx, j_idx) = Eq1.dot(Li);

					data.p2tau_pqpq[i_idx](k_idx, j_idx) = row_i.dot(Eq5);
					data.p2tau_pdqpq[i_idx](k_idx, j_idx) = row_i.dot(Eq8);
					data.p2tau_pdqpdq[i_idx](k_idx, j_idx) = row_i.dot(Eq10);
					data.p2tau_pqpddq[i_idx](k_idx, j_idx) = row_i.dot(Eq11);

					if (j != k) {
						data.p2tau_pqpq[i_idx](j_idx, k_idx) = data.p2tau_pqpq[i_idx](k_idx, j_idx);
						data.p2tau_pdqpq[i_idx](j_idx, k_idx) = row_i.dot(Eq9);
						data.p2tau_pdqpdq[i_idx](j_idx, k_idx) = data.p2tau_pdqpdq[i_idx](k_idx, j_idx);
						data.p2tau_pqpddq[i_idx](j_idx, k_idx) = row_i.dot(Eq12);
					}
				}

				i = model.parent[i];
			}
			j = model.parent[j];
		}

		if (parent > 0) {
			data.hI[parent] += hIk;
			data.VI[parent] += VIk;
			data.dPi.col(parent) += dPik;
		}
	}
}

// Overload without external forces
template <typename Scalar, typename DerivedQ, typename DerivedQvel, typename DerivedQacc>
void inverseDynamics_so(const Model<Scalar>& model, Data<Scalar>& data,
	const Eigen::MatrixBase<DerivedQ>& q,
  const Eigen::MatrixBase<DerivedQvel>& dq,
  const Eigen::MatrixBase<DerivedQacc>& ddq)
{
	inverseDynamics_so(model, data, q, dq, ddq, data.fext_zero);
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
