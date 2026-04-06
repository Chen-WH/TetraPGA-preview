#pragma once

#include <Eigen/Core>

namespace TetraPGA {

template <typename Scalar = double>
using Sphere3D = Eigen::Matrix<Scalar, 5, 1>;

template <typename Scalar = double>
using Plane3D = Eigen::Matrix<Scalar, 4, 1>;

template <typename Scalar = double>
using Line3D = Eigen::Matrix<Scalar, 6, 1>;

template <typename Scalar = double>
using Circle3D = Eigen::Matrix<Scalar, 10, 1>;

template <typename Scalar = double>
using PointPair3D = Eigen::Matrix<Scalar, 10, 1>;

template <typename Scalar = double>
using Point3D = Eigen::Matrix<Scalar, 4, 1>;

template <typename Scalar = double>
using Motor3D = Eigen::Matrix<Scalar, 8, 1>;

template <typename Scalar = double>
using DualNum = Eigen::Matrix<Scalar, 2, 1>;

template <typename Scalar = double>
using VectorXs = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

template <typename Scalar = double>
using MatrixXs = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Scalar = double>
struct SSP {               // Sphere Swept Point: Sphere
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int id;                  // Attached parent link ID
  Scalar radius;           // Radius of the swept sphere
  Point3D<Scalar> center;  // Center point of the sphere
};

template <typename Scalar = double>
struct SSL {          // Sphere Swept Line: Capsule
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int id;             // Attached parent link ID
  Scalar radius;      // Radius of the swept sphere
  Point3D<Scalar> endpointA;  // Point A of the line segment
  Point3D<Scalar> endpointB;  // Point B of the line segment
};

}  // namespace TetraPGA
