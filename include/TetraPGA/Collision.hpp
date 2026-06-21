#pragma once

#include "TetraPGA/Models.hpp"

namespace TetraPGA {

template <typename Scalar = double>
struct Environment {
	int num_static_sphere = 0;                  // Number of static spheres
  std::vector<SSP<Scalar>> static_sphere;     // Static spheres in the environment, defined in the world frame

	Environment(const std::vector<SSP<Scalar>>& spheres) {
		static_sphere = spheres;
		num_static_sphere = spheres.size();
	}
};

template <typename Scalar = double>
struct EnvironmentData {
	int num_collision_pair = 0;
	std::vector<Scalar> distance;
	std::vector<Scalar> t;
	std::vector<Point3D<Scalar>> normal;
	std::vector<Eigen::Matrix<Scalar, 1, Eigen::Dynamic>> jac_dist; // 距离对关节的雅可比，1*nq 行向量

	EnvironmentData(const Model<Scalar>& model, const Environment<Scalar>& env) {
		num_collision_pair = model.num_collision_ssl * env.num_static_sphere;
		
		distance.resize(num_collision_pair);
		t.resize(num_collision_pair);
		normal.resize(num_collision_pair);
		jac_dist.resize(num_collision_pair);
		for (int i = 0; i < num_collision_pair; ++i) {
			jac_dist[i].resize(1, model.dof_a);
			jac_dist[i].setZero();
		}
	}
};

template <typename Scalar>
inline void computePointToSegment(
	const Point3D<Scalar>& X,
	const Point3D<Scalar>& A,
	const Point3D<Scalar>& B,
	Scalar& out_distance,
	Scalar& out_t,
	Point3D<Scalar>& out_normal) 
{
	Point3D<Scalar> AB = B - A;
	Point3D<Scalar> AX = X - A;
	Scalar squared_length_AB = AB.squaredNorm();

	// 保护机制：退化线段 (Capsule 变成了一个 Sphere)
	if (squared_length_AB < std::numeric_limits<Scalar>::epsilon()) {
		out_t = 0.0; // 全部权重给 A
		Point3D<Scalar> PX = X - A;
		out_distance = PX.norm();
		if (out_distance > std::numeric_limits<Scalar>::epsilon()) {
			out_normal = PX / out_distance;
		} else {
			// 目标点 X 刚好和 A 重合，随便给一个合法的法向量避免 NaN
			out_normal = Point3D<Scalar>(1.0, 0.0, 0.0, 0.0); 
		}
	} else {
		// 计算无约束的投影比例 t (利用点积)
		out_t = AX.dot(AB) / squared_length_AB;
		out_t = std::max(Scalar(0.0), std::min(Scalar(1.0), out_t));
		Point3D<Scalar> P = A + out_t * AB;

		// 计算从最近点 P 指向目标点 X 的差值向量
		Point3D<Scalar> PX = X - P;
		out_distance = PX.norm();

		// 保护机制：X 恰好在 AB 线段上 (深度穿透)
		if (out_distance > std::numeric_limits<Scalar>::epsilon()) {
			out_normal = PX / out_distance;
		} else {
			// 发生这种极端情况意味着环境球心完全扎进了胶囊体骨架里
			// 我们需要一个垂直于 AB 的任意向量来把球 "推出去"，防止梯度消失或 NaN
			const Eigen::Matrix<Scalar, 3, 1> AB3 = AB.template head<3>();
			const Eigen::Matrix<Scalar, 3, 1> unit_x(Scalar(1.0), Scalar(0.0), Scalar(0.0));

			Eigen::Matrix<Scalar, 3, 1> n_fallback = AB3.cross(unit_x);
			if (n_fallback.squaredNorm() < std::numeric_limits<Scalar>::epsilon()) {
				const Eigen::Matrix<Scalar, 3, 1> unit_y(Scalar(0.0), Scalar(1.0), Scalar(0.0));
				n_fallback = AB3.cross(unit_y);
			}

			out_normal.setZero();
			out_normal.template head<3>() = n_fallback.normalized();
		}
	}
}

// compute the collision distance
template <typename Scalar>
void computeDistance(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Environment<Scalar>& env, EnvironmentData<Scalar>& env_data) 
{
	for (int i = 0; i < model.num_collision_ssl; ++i) {
		int link_id = model.collisionSSL[i].id;
		data.SSL_A[i] = pga_rbm3(data.Mi.col(link_id), model.collisionSSL[i].endpointA);
		data.SSL_B[i] = pga_rbm3(data.Mi.col(link_id), model.collisionSSL[i].endpointB);
	}
	int idx = 0;
	for (int i = 0; i < model.num_collision_ssl; ++i) {
		for (int j = 0; j < env.num_static_sphere; ++j) {
			computePointToSegment(env.static_sphere[j].center, 
				data.SSL_A[i], 
				data.SSL_B[i], 
				env_data.distance[idx], 
				env_data.t[idx], 
				env_data.normal[idx]);

			env_data.distance[idx] -= (model.collisionSSL[i].radius + env.static_sphere[j].radius);
			idx++;
		}
	}
}

// compute the collision distance Jacobian
template <typename Scalar>
void computeDistanceJacobian(
	const Model<Scalar>& model, Data<Scalar>& data,
	const Environment<Scalar>& env, EnvironmentData<Scalar>& env_data) 
{
	for (int i = 0; i < model.num_collision_ssl; ++i) {
		int link_id = model.collisionSSL[i].id;
		data.SSL_A[i] = pga_rbm3(data.Mi.col(link_id), model.collisionSSL[i].endpointA);
		data.SSL_B[i] = pga_rbm3(data.Mi.col(link_id), model.collisionSSL[i].endpointB);
		
		data.SSL_jacA[i].setZero();
		data.SSL_jacB[i].setZero();
		if (link_id > 0) {
			data.SSL_jacA[i].col(link_id - 1) = -pga_com23(data.L.col(link_id), data.SSL_A[i]);
			data.SSL_jacB[i].col(link_id - 1) = -pga_com23(data.L.col(link_id), data.SSL_B[i]);
		}
		for (int idx : model.ancestor[link_id]) {
			if (idx > 0) {
				data.SSL_jacA[i].col(idx-1) = -pga_com23(data.L.col(idx), data.SSL_A[i]);
				data.SSL_jacB[i].col(idx-1) = -pga_com23(data.L.col(idx), data.SSL_B[i]);
			}
		}
	}
	int idx = 0;
	for (int i = 0; i < model.num_collision_ssl; ++i) {
		for (int j = 0; j < env.num_static_sphere; ++j) {
			computePointToSegment(env.static_sphere[j].center, 
				data.SSL_A[i], 
				data.SSL_B[i], 
				env_data.distance[idx], 
				env_data.t[idx], 
				env_data.normal[idx]);
			env_data.distance[idx] -= (model.collisionSSL[i].radius + env.static_sphere[j].radius);

			env_data.jac_dist[idx].noalias() =
				-env_data.normal[idx].transpose() *
				((Scalar(1.0) - env_data.t[idx]) * data.SSL_jacA[i] + env_data.t[idx] * data.SSL_jacB[i]);
			idx++;
		}
	}
}

}  // namespace TetraPGA
