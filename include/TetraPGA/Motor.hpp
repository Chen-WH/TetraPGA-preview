#pragma once

#include "TetraPGA/Elements3D.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <type_traits>

namespace TetraPGA {

// Geometric product of two motors (GA_mul)
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_mul(const Eigen::MatrixBase<Derived1>& M1, const Eigen::MatrixBase<Derived2>& M2)
-> Motor3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 8);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 8);
  
  const Scalar a0 = M1(0), a1 = M1(1), a2 = M1(2), a3 = M1(3);
  const Scalar a4 = M1(4), a5 = M1(5), a6 = M1(6), a7 = M1(7);
  const Scalar b0 = M2(0), b1 = M2(1), b2 = M2(2), b3 = M2(3);
  const Scalar b4 = M2(4), b5 = M2(5), b6 = M2(6), b7 = M2(7);
  Motor3D<Scalar> M;

  M(0) = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
  M(1) = a0 * b1 + a1 * b0 - a2 * b3 + a3 * b2;
  M(2) = a0 * b2 + a1 * b3 + a2 * b0 - a3 * b1;
  M(3) = a0 * b3 - a1 * b2 + a2 * b1 + a3 * b0;

  M(4) = a0 * b4 - a1 * b7 - a2 * b6 + a3 * b5 + a4 * b0 - a5 * b3 + a6 * b2 - a7 * b1;
  M(5) = a0 * b5 + a1 * b6 - a2 * b7 - a3 * b4 + a4 * b3 + a5 * b0 - a6 * b1 - a7 * b2;
  M(6) = a0 * b6 - a1 * b5 + a2 * b4 - a3 * b7 - a4 * b2 + a5 * b1 + a6 * b0 - a7 * b3;
  M(7) = a0 * b7 + a1 * b4 + a2 * b5 + a3 * b6 + a4 * b1 + a5 * b2 + a6 * b3 + a7 * b0;

  return M;
}

// Reverse (GA_rev)
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_rev(const Eigen::MatrixBase<Derived>& M)
-> Motor3D<typename Derived::Scalar>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 8);

  Motor3D<Scalar> Mt;
  Mt(0) =  M(0);
  Mt(1) = -M(1);
  Mt(2) = -M(2);
  Mt(3) = -M(3);
  Mt(4) = -M(4);
  Mt(5) = -M(5);
  Mt(6) = -M(6);
  Mt(7) =  M(7);

  return Mt;
}

// Motor * bivector (GA_prodMB)
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_prodMB(const Eigen::MatrixBase<Derived1>& M, const Eigen::MatrixBase<Derived2>& Bs)
-> Motor3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 8);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 6);

  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  const Scalar Bs0 = Bs(0), Bs1 = Bs(1), Bs2 = Bs(2), Bs3 = Bs(3), Bs4 = Bs(4), Bs5 = Bs(5);
  Motor3D<Scalar> dM;

  dM(0) = -Bs0 * M1 - Bs1 * M2 - Bs2 * M3;
  dM(1) =  Bs0 * M0 + Bs1 * M3 - Bs2 * M2;
  dM(2) = -Bs0 * M3 + Bs1 * M0 + Bs2 * M1;
  dM(3) =  Bs0 * M2 - Bs1 * M1 + Bs2 * M0;
  dM(4) = -Bs0 * M7 + Bs1 * M6 - Bs2 * M5 + Bs3 * M0 + Bs4 * M3 - Bs5 * M2;
  dM(5) = -Bs0 * M6 - Bs1 * M7 + Bs2 * M4 - Bs3 * M3 + Bs4 * M0 + Bs5 * M1;
  dM(6) =  Bs0 * M5 - Bs1 * M4 - Bs2 * M7 + Bs3 * M2 - Bs4 * M1 + Bs5 * M0;
  dM(7) =  Bs0 * M4 + Bs1 * M5 + Bs2 * M6 + Bs3 * M1 + Bs4 * M2 + Bs5 * M3;
  return dM;
}

// Bivector * motor (GA_prodBM)
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_prodBM(const Eigen::MatrixBase<Derived1>& Bb, const Eigen::MatrixBase<Derived2>& M)
-> Motor3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 8);

  const Scalar Bb0 = Bb(0), Bb1 = Bb(1), Bb2 = Bb(2), Bb3 = Bb(3), Bb4 = Bb(4), Bb5 = Bb(5);
  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  Motor3D<Scalar> dM;

  dM(0) = -Bb0 * M1 - Bb1 * M2 - Bb2 * M3;
  dM(1) =  Bb0 * M0 - Bb1 * M3 + Bb2 * M2;
  dM(2) =  Bb0 * M3 + Bb1 * M0 - Bb2 * M1;
  dM(3) = -Bb0 * M2 + Bb1 * M1 + Bb2 * M0;
  dM(4) = -Bb0 * M7 - Bb1 * M6 + Bb2 * M5 + Bb3 * M0 - Bb4 * M3 + Bb5 * M2;
  dM(5) =  Bb0 * M6 - Bb1 * M7 - Bb2 * M4 + Bb3 * M3 + Bb4 * M0 - Bb5 * M1;
  dM(6) = -Bb0 * M5 + Bb1 * M4 - Bb2 * M7 - Bb3 * M2 + Bb4 * M1 + Bb5 * M0;
  dM(7) =  Bb0 * M4 + Bb1 * M5 + Bb2 * M6 + Bb3 * M1 + Bb4 * M2 + Bb5 * M3;
  return dM;
}

// Metric on Lie algebra
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_metric(const Eigen::MatrixBase<Derived>& L)
-> Eigen::Matrix<typename Derived::Scalar, 1, 6>
{
    using Scalar = typename Derived::Scalar;
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

    Eigen::Matrix<Scalar, 1, 6> Lstar;
    Lstar(0) = L(3);
    Lstar(1) = L(4);
    Lstar(2) = L(5);
    Lstar(3) = L(0);
    Lstar(4) = L(1);
    Lstar(5) = L(2);

    return Lstar;
}

template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_metric(const Eigen::MatrixBase<Derived1>& L, const Eigen::MatrixBase<Derived2>& f)
-> std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 6);

  const Scalar l0 = L(0), l1 = L(1), l2 = L(2), l3 = L(3), l4 = L(4), l5 = L(5);
  const Scalar f0 = f(0), f1 = f(1), f2 = f(2), f3 = f(3), f4 = f(4), f5 = f(5);

  return l0 * f3 + l1 * f4 + l2 * f5 + l3 * f0 + l4 * f1 + l5 * f2;
}

// Exponential map from bivector to motor (GA_exp)
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_exp(const Eigen::MatrixBase<Derived>& B)
-> Motor3D<typename Derived::Scalar>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

  const Scalar B0 = B(0), B1 = B(1), B2 = B(2), B3 = B(3), B4 = B(4), B5 = B(5);
  const Scalar l2 = B0*B0 + B1*B1 + B2*B2;
  const Scalar m = B0*B3 + B1*B4 + B2*B5;
  Scalar c, s, t;
  Motor3D<Scalar> M;
  
  if (l2 > Scalar(1e-8)) {
    const Scalar l = std::sqrt(l2);
    c = std::cos(l);
    s = std::sin(l) / l;
    t = (m / l2) * (c - s);
  } else {
    c = Scalar(1) - l2 * Scalar(0.5);
    s = Scalar(1) - l2 * Scalar(1.0/6.0); 
    t = -m / Scalar(3);
  }
  
  M(0) = c;
  M(1) = s * B0;
  M(2) = s * B1;
  M(3) = s * B2;
  M(4) = s * B3 + t * B0;
  M(5) = s * B4 + t * B1;
  M(6) = s * B5 + t * B2;
  M(7) = m * s;

  return M;
}

// Log map from motor to bivector (GA_log)
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_log(const Eigen::MatrixBase<Derived>& M)
-> Line3D<typename Derived::Scalar>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 8);

  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  Scalar b, c;
  Line3D<Scalar> B;

  const Scalar sq = Scalar(1) - M0 * M0;
  
  if (sq > Scalar(1e-8)) {
    const Scalar a = Scalar(1) / sq;
    b = std::acos(M0) * std::sqrt(a);
    c = a * M7 * (Scalar(1) - M0 * b);
  } else {
    b = Scalar(1);
    c = Scalar(0);
  }

  B(0) = b * M1;
  B(1) = b * M2;
  B(2) = b * M3;
  B(3) = c * M1 + b * M4;
  B(4) = c * M2 + b * M5;
  B(5) = c * M3 + b * M6;

  return B;
}

// Rigid-body motion: returns 6x6 transformation matrix
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_rbm(const Eigen::MatrixBase<Derived>& M)
-> Eigen::Matrix<typename Derived::Scalar, 6, 6>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 8);

  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  Eigen::Matrix<Scalar, 6, 6> rbm;

  const Scalar M00 = M0*M0, M11 = M1*M1, M22 = M2*M2, M33 = M3*M3;
  const Scalar M01 = M0*M1, M02 = M0*M2, M03 = M0*M3;
  const Scalar M12 = M1*M2, M13 = M1*M3, M23 = M2*M3;

  rbm(0,0) = M00 + M11 - M22 - M33;
  rbm(0,1) = Scalar(2) * (M12 - M03);
  rbm(0,2) = Scalar(2) * (M02 + M13);
  rbm(1,0) = Scalar(2) * (M03 + M12);
  rbm(1,1) = M00 - M11 + M22 - M33;
  rbm(1,2) = Scalar(2) * (M23 - M01);
  rbm(2,0) = Scalar(2) * (M13 - M02);
  rbm(2,1) = Scalar(2) * (M01 + M23);
  rbm(2,2) = M00 - M11 - M22 + M33;
  
  rbm(3,0) = Scalar(2) * (M1*M4 - M0*M7 - M2*M5 - M3*M6);
  rbm(3,1) = Scalar(2) * (M1*M5 - M0*M6 + M2*M4 + M3*M7);
  rbm(3,2) = Scalar(2) * (M0*M5 + M1*M6 + M3*M4 - M2*M7);
  rbm(4,0) = Scalar(2) * (M0*M6 + M1*M5 + M2*M4 - M3*M7);
  rbm(4,1) = Scalar(2) * (M2*M5 - M0*M7 - M1*M4 - M3*M6);
  rbm(4,2) = Scalar(2) * (M1*M7 - M0*M4 + M2*M6 + M3*M5);
  rbm(5,0) = Scalar(2) * (M1*M6 - M0*M5 + M3*M4 + M2*M7);
  rbm(5,1) = Scalar(2) * (M0*M4 - M1*M7 + M2*M6 + M3*M5);
  rbm(5,2) = Scalar(2) * (M3*M6 - M0*M7 - M2*M5 - M1*M4);

  rbm.template topRightCorner<3, 3>().setZero();
  rbm.template bottomRightCorner<3, 3>() = rbm.template topLeftCorner<3, 3>();

  return rbm;
}

template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_rbm(const Eigen::MatrixBase<Derived1>& M, const Eigen::MatrixBase<Derived2>& L)
-> Line3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 8);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 6);

  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  const Scalar L0 = L(0), L1 = L(1), L2 = L(2), L3 = L(3), L4 = L(4), L5 = L(5);
  Line3D<Scalar> Lp;

  const Scalar M00 = M0*M0, M11 = M1*M1, M22 = M2*M2, M33 = M3*M3;
  const Scalar M01 = M0*M1, M02 = M0*M2, M03 = M0*M3;
  const Scalar M12 = M1*M2, M13 = M1*M3, M23 = M2*M3;

  const Scalar R00 = M00 + M11 - M22 - M33;
  const Scalar R01 = Scalar(2) * (M12 - M03);
  const Scalar R02 = Scalar(2) * (M02 + M13);
  const Scalar R10 = Scalar(2) * (M03 + M12);
  const Scalar R11 = M00 - M11 + M22 - M33;
  const Scalar R12 = Scalar(2) * (M23 - M01);
  const Scalar R20 = Scalar(2) * (M13 - M02);
  const Scalar R21 = Scalar(2) * (M01 + M23);
  const Scalar R22 = M00 - M11 - M22 + M33;

  Lp(0) = R00 * L0 + R01 * L1 + R02 * L2;
  Lp(1) = R10 * L0 + R11 * L1 + R12 * L2;
  Lp(2) = R20 * L0 + R21 * L1 + R22 * L2;
  Lp(3) = Scalar(2) * (M1*M4 - M0*M7 - M2*M5 - M3*M6) * L0 +
          Scalar(2) * (M1*M5 - M0*M6 + M2*M4 + M3*M7) * L1 +
          Scalar(2) * (M0*M5 + M1*M6 + M3*M4 - M2*M7) * L2 +
          R00 * L3 + R01 * L4 + R02 * L5;
  Lp(4) = Scalar(2) * (M0*M6 + M1*M5 + M2*M4 - M3*M7) * L0 +
          Scalar(2) * (M2*M5 - M0*M7 - M1*M4 - M3*M6) * L1 +
          Scalar(2) * (M1*M7 - M0*M4 + M2*M6 + M3*M5) * L2 +
          R10 * L3 + R11 * L4 + R12 * L5;
  Lp(5) = Scalar(2) * (M1*M6 - M0*M5 + M3*M4 + M2*M7) * L0 +
          Scalar(2) * (M0*M4 - M1*M7 + M2*M6 + M3*M5) * L1 +
          Scalar(2) * (M3*M6 - M0*M7 - M2*M5 - M1*M4) * L2 +
          R20 * L3 + R21 * L4 + R22 * L5;

  return Lp;
}

// Adjoint: returns 6x6 adjoint matrix
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_AdM(const Eigen::MatrixBase<Derived>& M)
-> Eigen::Matrix<typename Derived::Scalar, 6, 6>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 8);

  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  Eigen::Matrix<Scalar, 6, 6> AdM;

  const Scalar M00 = M0*M0, M11 = M1*M1, M22 = M2*M2, M33 = M3*M3;
  const Scalar M01 = M0*M1, M02 = M0*M2, M03 = M0*M3;
  const Scalar M12 = M1*M2, M13 = M1*M3, M23 = M2*M3;

  AdM(0,0) = M00 + M11 - M22 - M33;
  AdM(0,1) = Scalar(2) * (M03 + M12);
  AdM(0,2) = Scalar(2) * (M13 - M02);
  AdM(1,0) = Scalar(2) * (M12 - M03);
  AdM(1,1) = M00 - M11 + M22 - M33;
  AdM(1,2) = Scalar(2) * (M01 + M23);
  AdM(2,0) = Scalar(2) * (M02 + M13);
  AdM(2,1) = Scalar(2) * (M23 - M01);
  AdM(2,2) = M00 - M11 - M22 + M33;

  AdM(3,0) = Scalar(2) * (M1*M4 - M0*M7 - M2*M5 - M3*M6);
  AdM(3,1) = Scalar(2) * (M0*M6 + M1*M5 + M2*M4 - M3*M7);
  AdM(3,2) = Scalar(2) * (M1*M6 - M0*M5 + M3*M4 + M2*M7);
  AdM(4,0) = Scalar(2) * (M1*M5 - M0*M6 + M2*M4 + M3*M7);
  AdM(4,1) = Scalar(2) * (M2*M5 - M0*M7 - M1*M4 - M3*M6);
  AdM(4,2) = Scalar(2) * (M0*M4 - M1*M7 + M2*M6 + M3*M5);
  AdM(5,0) = Scalar(2) * (M0*M5 + M1*M6 + M3*M4 - M2*M7);
  AdM(5,1) = Scalar(2) * (M1*M7 - M0*M4 + M2*M6 + M3*M5);
  AdM(5,2) = Scalar(2) * (M3*M6 - M0*M7 - M2*M5 - M1*M4);

  AdM.template topRightCorner<3, 3>().setZero();
  AdM.template bottomRightCorner<3, 3>() = AdM.template topLeftCorner<3, 3>();

  return AdM;
}

// Adjoint: returns transformed bivector
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_AdM(const Eigen::MatrixBase<Derived1>& M, const Eigen::MatrixBase<Derived2>& L)
-> Line3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 8);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 6);

  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  const Scalar L0 = L(0), L1 = L(1), L2 = L(2), L3 = L(3), L4 = L(4), L5 = L(5);
  Line3D<Scalar> Lp;

  const Scalar M00 = M0*M0, M11 = M1*M1, M22 = M2*M2, M33 = M3*M3;
  const Scalar M01 = M0*M1, M02 = M0*M2, M03 = M0*M3;
  const Scalar M12 = M1*M2, M13 = M1*M3, M23 = M2*M3;

  const Scalar R00 = M00 + M11 - M22 - M33;
  const Scalar R01 = Scalar(2) * (M03 + M12);
  const Scalar R02 = Scalar(2) * (M13 - M02);
  const Scalar R10 = Scalar(2) * (M12 - M03);
  const Scalar R11 = M00 - M11 + M22 - M33;
  const Scalar R12 = Scalar(2) * (M01 + M23);
  const Scalar R20 = Scalar(2) * (M02 + M13);
  const Scalar R21 = Scalar(2) * (M23 - M01);
  const Scalar R22 = M00 - M11 - M22 + M33;

  Lp(0) = R00 * L0 + R01 * L1 + R02 * L2;
  Lp(1) = R10 * L0 + R11 * L1 + R12 * L2;
  Lp(2) = R20 * L0 + R21 * L1 + R22 * L2;
  Lp(3) = Scalar(2) * (M1*M4 - M0*M7 - M2*M5 - M3*M6) * L0 +
          Scalar(2) * (M0*M6 + M1*M5 + M2*M4 - M3*M7) * L1 +
          Scalar(2) * (M1*M6 - M0*M5 + M3*M4 + M2*M7) * L2 +
          R00 * L3 + R01 * L4 + R02 * L5;
  Lp(4) = Scalar(2) * (M1*M5 - M0*M6 + M2*M4 + M3*M7) * L0 +
          Scalar(2) * (M2*M5 - M0*M7 - M1*M4 - M3*M6) * L1 +
          Scalar(2) * (M0*M4 - M1*M7 + M2*M6 + M3*M5) * L2 +
          R10 * L3 + R11 * L4 + R12 * L5;
  Lp(5) = Scalar(2) * (M0*M5 + M1*M6 + M3*M4 - M2*M7) * L0 +
          Scalar(2) * (M1*M7 - M0*M4 + M2*M6 + M3*M5) * L1 +
          Scalar(2) * (M3*M6 - M0*M7 - M2*M5 - M1*M4) * L2 +
          R20 * L3 + R21 * L4 + R22 * L5;

  return Lp;
}

// Commutator operator: returns 6x6 matrix
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_com(const Eigen::MatrixBase<Derived>& B)
-> Eigen::Matrix<typename Derived::Scalar, 6, 6>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

  const Scalar B0 = B(0), B1 = B(1), B2 = B(2);
  const Scalar B3 = B(3), B4 = B(4), B5 = B(5);
  Eigen::Matrix<Scalar, 6, 6> res;

  res(0, 0) = Scalar(0); res(0, 1) =  B2;       res(0, 2) = -B1;
  res(1, 0) = -B2;       res(1, 1) = Scalar(0); res(1, 2) =  B0;
  res(2, 0) =  B1;       res(2, 1) = -B0;       res(2, 2) = Scalar(0);

  res(3, 0) = Scalar(0); res(3, 1) =  B5;       res(3, 2) = -B4;
  res(4, 0) = -B5;       res(4, 1) = Scalar(0); res(4, 2) =  B3;
  res(5, 0) =  B4;       res(5, 1) = -B3;       res(5, 2) = Scalar(0);

  res.template topRightCorner<3, 3>().setZero();
  res.template bottomRightCorner<3, 3>() = res.template topLeftCorner<3, 3>();

  return res;
}

// Commutator operator: returns bivector
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_com(const Eigen::MatrixBase<Derived1>& B1, const Eigen::MatrixBase<Derived2>& B2)
-> Line3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 6);

  const Scalar a0 = B1(0), a1 = B1(1), a2 = B1(2), a3 = B1(3), a4 = B1(4), a5 = B1(5);
  const Scalar b0 = B2(0), b1 = B2(1), b2 = B2(2), b3 = B2(3), b4 = B2(4), b5 = B2(5);
  Line3D<Scalar> res;

  res(0) = a2 * b1 - a1 * b2;
  res(1) = a0 * b2 - a2 * b0;
  res(2) = a1 * b0 - a0 * b1;
  res(3) = (a2 * b4 - a1 * b5) + (a5 * b1 - a4 * b2);
  res(4) = (a0 * b5 - a2 * b3) + (a3 * b2 - a5 * b0);
  res(5) = (a1 * b3 - a0 * b4) + (a4 * b0 - a3 * b1);

  return res;
}

// Lie bracket / ad operator: returns 6x6 matrix
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_adB(const Eigen::MatrixBase<Derived>& B)
-> Eigen::Matrix<typename Derived::Scalar, 6, 6>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

  const Scalar B0 = 2*B(0), B1 = 2*B(1), B2 = 2*B(2);
  const Scalar B3 = 2*B(3), B4 = 2*B(4), B5 = 2*B(5);
  Eigen::Matrix<Scalar, 6, 6> res;

  res(0, 0) = Scalar(0); res(0, 1) =  B2;       res(0, 2) = -B1;
  res(1, 0) = -B2;       res(1, 1) = Scalar(0); res(1, 2) =  B0;
  res(2, 0) =  B1;       res(2, 1) = -B0;       res(2, 2) = Scalar(0);

  res(3, 0) = Scalar(0); res(3, 1) =  B5;       res(3, 2) = -B4;
  res(4, 0) = -B5;       res(4, 1) = Scalar(0); res(4, 2) =  B3;
  res(5, 0) =  B4;       res(5, 1) = -B3;       res(5, 2) = Scalar(0);

  res.template topRightCorner<3, 3>().setZero();
  res.template bottomRightCorner<3, 3>() = res.template topLeftCorner<3, 3>();

  return res;
}

// Lie bracket / ad operator: returns bivector
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_adB(const Eigen::MatrixBase<Derived1>& B1, const Eigen::MatrixBase<Derived2>& B2)
-> Line3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 6);

  const Scalar a0 = 2*B1(0), a1 = 2*B1(1), a2 = 2*B1(2), a3 = 2*B1(3), a4 = 2*B1(4), a5 = 2*B1(5);
  const Scalar b0 = B2(0), b1 = B2(1), b2 = B2(2), b3 = B2(3), b4 = B2(4), b5 = B2(5);
  Line3D<Scalar> res;

  res(0) = a2 * b1 - a1 * b2;
  res(1) = a0 * b2 - a2 * b0;
  res(2) = a1 * b0 - a0 * b1;
  res(3) = (a2 * b4 - a1 * b5) + (a5 * b1 - a4 * b2);
  res(4) = (a0 * b5 - a2 * b3) + (a3 * b2 - a5 * b0);
  res(5) = (a1 * b3 - a0 * b4) + (a4 * b0 - a3 * b1);

  return res;
}

template <typename ScalarQ, typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto Quat_2motor(const Eigen::Quaternion<ScalarQ>& q, const Eigen::MatrixBase<Derived>& t)
-> Motor3D<std::common_type_t<ScalarQ, typename Derived::Scalar>>
{
  using Scalar = std::common_type_t<ScalarQ, typename Derived::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  const Scalar qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  const Scalar tx = t(0), ty = t(1), tz = t(2);
  Motor3D<Scalar> M;

  M(0) = qw;
  M(1) = qx;
  M(2) = qy;
  M(3) = qz;
  M(4) = Scalar(0.5) * (tx * qw + ty * qz - tz * qy);
  M(5) = Scalar(0.5) * (ty * qw - tx * qz + tz * qx);
  M(6) = Scalar(0.5) * (tz * qw + tx * qy - ty * qx);
  M(7) = Scalar(0.5) * (tx * qx + ty * qy + tz * qz);

  return M;
}

template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto SE3_2motor(const Eigen::MatrixBase<Derived>& T)
-> Motor3D<typename Derived::Scalar>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4);

  Eigen::Quaternion<Scalar> q(T.template topLeftCorner<3, 3>());
  q.normalize();
  return Quat_2motor(q, T.template topRightCorner<3, 1>());
}

// DH parameters to motor conversion
// 需要优化以提高计算效率，统一param的顺序
template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_DH2motor(const Eigen::Matrix<Scalar, 4, 1>& param, bool is_modifed = false)
-> Motor3D<Scalar>
{
  Motor3D<Scalar> M;
  Scalar d, theta, a, alpha;

  if (is_modifed) {
    a = Scalar(0.5) * param(0);
    alpha = Scalar(0.5) * param(1);
    d = Scalar(0.5) * param(2);
    theta = Scalar(0.5) * param(3);
    M(0) = std::cos(alpha) * std::cos(theta);
    M(1) = std::sin(alpha) * std::cos(theta);
    M(2) = -std::sin(alpha) * std::sin(theta);
    M(3) = std::cos(alpha) * std::sin(theta);
    M(4) = a * std::cos(alpha) * std::cos(theta) - d * std::sin(alpha) * std::sin(theta);
    M(5) = -a * std::cos(alpha) * std::sin(theta) - d * std::sin(alpha) * std::cos(theta);
    M(6) = d * std::cos(alpha) * std::cos(theta) - a * std::sin(alpha) * std::sin(theta);
    M(7) = d * std::cos(alpha) * std::sin(theta) + a * std::sin(alpha) * std::cos(theta);
  }
  else {
    d = Scalar(0.5) * param(0);
    theta = Scalar(0.5) * param(1);
    a = Scalar(0.5) * param(2);
    alpha = Scalar(0.5) * param(3);
    M(0) = std::cos(alpha) * std::cos(theta);
    M(1) = std::sin(alpha) * std::cos(theta);
    M(2) = std::sin(alpha) * std::sin(theta);
    M(3) = std::cos(alpha) * std::sin(theta);
    M(4) = a * std::cos(alpha) * std::cos(theta) - d * std::sin(alpha) * std::sin(theta);
    M(5) = a * std::cos(alpha) * std::sin(theta) + d * std::sin(alpha) * std::cos(theta);
    M(6) = d * std::cos(alpha) * std::cos(theta) - a * std::sin(alpha) * std::sin(theta);
    M(7) = d * std::cos(alpha) * std::sin(theta) + a * std::sin(alpha) * std::cos(theta);
  }
  return M;
}

// Exponential map for Revolute Joint
template <typename Derived1, typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_exp_R(const Eigen::MatrixBase<Derived1>& L, const Scalar theta)
-> Motor3D<std::common_type_t<typename Derived1::Scalar, Scalar>>
{
  using ScalarOut = std::common_type_t<typename Derived1::Scalar, Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  const ScalarOut L0 = L(0), L1 = L(1), L2 = L(2);
  const ScalarOut L3 = L(3), L4 = L(4), L5 = L(5);
  const ScalarOut c = std::cos(theta);
  const ScalarOut s = std::sin(theta);
  
  Motor3D<ScalarOut> M;
  M(0) = c;
  M(1) = s * L0;
  M(2) = s * L1;
  M(3) = s * L2;
  M(4) = s * L3;
  M(5) = s * L4;
  M(6) = s * L5;
  M(7) = ScalarOut(0);
  return M;
}

// Specialized product exp_R(L, theta) * M to avoid constructing the exponential motor
// before multiplying in hot dynamics loops.
template <typename DerivedL, typename Scalar, typename DerivedM>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_mul_exp_R(const Eigen::MatrixBase<DerivedL>& L, const Scalar theta,
                  const Eigen::MatrixBase<DerivedM>& M)
-> Motor3D<std::common_type_t<typename DerivedL::Scalar,
                              std::common_type_t<Scalar, typename DerivedM::Scalar>>>
{
  using ScalarOut = std::common_type_t<typename DerivedL::Scalar,
                                       std::common_type_t<Scalar, typename DerivedM::Scalar>>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedL, 6);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedM, 8);

  const ScalarOut c = std::cos(theta);
  const ScalarOut s = std::sin(theta);
  const ScalarOut e1 = s * L(0);
  const ScalarOut e2 = s * L(1);
  const ScalarOut e3 = s * L(2);
  const ScalarOut e4 = s * L(3);
  const ScalarOut e5 = s * L(4);
  const ScalarOut e6 = s * L(5);

  const ScalarOut b0 = M(0), b1 = M(1), b2 = M(2), b3 = M(3);
  const ScalarOut b4 = M(4), b5 = M(5), b6 = M(6), b7 = M(7);
  Motor3D<ScalarOut> out;

  out(0) = c * b0 - e1 * b1 - e2 * b2 - e3 * b3;
  out(1) = c * b1 + e1 * b0 - e2 * b3 + e3 * b2;
  out(2) = c * b2 + e1 * b3 + e2 * b0 - e3 * b1;
  out(3) = c * b3 - e1 * b2 + e2 * b1 + e3 * b0;

  out(4) = c * b4 - e1 * b7 - e2 * b6 + e3 * b5 + e4 * b0 - e5 * b3 + e6 * b2;
  out(5) = c * b5 + e1 * b6 - e2 * b7 - e3 * b4 + e4 * b3 + e5 * b0 - e6 * b1;
  out(6) = c * b6 - e1 * b5 + e2 * b4 - e3 * b7 - e4 * b2 + e5 * b1 + e6 * b0;
  out(7) = c * b7 + e1 * b4 + e2 * b5 + e3 * b6 + e4 * b1 + e5 * b2 + e6 * b3;
  return out;
}

// Exponential map for Prismatic Joint
template <typename Derived1, typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_exp_P(const Eigen::MatrixBase<Derived1>& L, const Scalar d)
-> Motor3D<std::common_type_t<typename Derived1::Scalar, Scalar>>
{
  using ScalarOut = std::common_type_t<typename Derived1::Scalar, Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  const ScalarOut L0 = L(0), L1 = L(1), L2 = L(2);
  const ScalarOut L3 = L(3), L4 = L(4), L5 = L(5);
  
  Motor3D<ScalarOut> M;
  M(0) = ScalarOut(1);
  M(1) = d * L0;
  M(2) = d * L1;
  M(3) = d * L2;
  M(4) = d * L3;
  M(5) = d * L4;
  M(6) = d * L5;
  M(7) = ScalarOut(0);
  return M;
}

// Specialized product exp_P(L, d) * M to avoid constructing the exponential motor
// before multiplying in hot dynamics loops.
template <typename DerivedL, typename Scalar, typename DerivedM>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_mul_exp_P(const Eigen::MatrixBase<DerivedL>& L, const Scalar d,
                  const Eigen::MatrixBase<DerivedM>& M)
-> Motor3D<std::common_type_t<typename DerivedL::Scalar,
                              std::common_type_t<Scalar, typename DerivedM::Scalar>>>
{
  using ScalarOut = std::common_type_t<typename DerivedL::Scalar,
                                       std::common_type_t<Scalar, typename DerivedM::Scalar>>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedL, 6);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedM, 8);

  const ScalarOut e1 = d * L(0);
  const ScalarOut e2 = d * L(1);
  const ScalarOut e3 = d * L(2);
  const ScalarOut e4 = d * L(3);
  const ScalarOut e5 = d * L(4);
  const ScalarOut e6 = d * L(5);

  const ScalarOut b0 = M(0), b1 = M(1), b2 = M(2), b3 = M(3);
  const ScalarOut b4 = M(4), b5 = M(5), b6 = M(6), b7 = M(7);
  Motor3D<ScalarOut> out;

  out(0) = b0 - e1 * b1 - e2 * b2 - e3 * b3;
  out(1) = b1 + e1 * b0 - e2 * b3 + e3 * b2;
  out(2) = b2 + e1 * b3 + e2 * b0 - e3 * b1;
  out(3) = b3 - e1 * b2 + e2 * b1 + e3 * b0;

  out(4) = b4 - e1 * b7 - e2 * b6 + e3 * b5 + e4 * b0 - e5 * b3 + e6 * b2;
  out(5) = b5 + e1 * b6 - e2 * b7 - e3 * b4 + e4 * b3 + e5 * b0 - e6 * b1;
  out(6) = b6 - e1 * b5 + e2 * b4 - e3 * b7 - e4 * b2 + e5 * b1 + e6 * b0;
  out(7) = b7 + e1 * b4 + e2 * b5 + e3 * b6 + e4 * b1 + e5 * b2 + e6 * b3;
  return out;
}

// Derivatives of the exponential map in the Motor Group (GA_dexp)
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto ga_dexp(const Eigen::MatrixBase<Derived>& r)
-> Eigen::Matrix<typename Derived::Scalar, 6, 6>
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

  const Scalar t = Scalar(2) * r.template head<3>().norm();
  const Scalar t2 = t * t;
  Scalar A, B, C3, C4;

  if (t < Scalar(1e-8)) {
      A = Scalar(0.5);
      B = Scalar(1.0/6.0);
      C3 = Scalar(1.0/24.0);
      C4 = Scalar(1.0/120.0);
  } else {
      const Scalar st = std::sin(t);
      const Scalar ct = std::cos(t);
      const Scalar t3 = t2 * t;
      const Scalar t4 = t3 * t;
      const Scalar t5 = t4 * t;

      A = (Scalar(1) - ct) / t2;
      B = (t - st) / t3;
      C3 = (Scalar(2) - Scalar(2)*ct - t*st) / (Scalar(2) * t4);
      C4 = (Scalar(2)*t - Scalar(3)*st + t*ct) / (Scalar(2) * t5);
  }

  const Eigen::Matrix<Scalar, 6, 6> ad = ga_adB(r);
  const Eigen::Matrix<Scalar, 6, 6> ad2 = ad * ad;
  const Eigen::Matrix<Scalar, 6, 6> term3 = ad2 * ad + t2 * ad;
  const Eigen::Matrix<Scalar, 6, 6> term4 = term3 * ad;

  Eigen::Matrix<Scalar, 6, 6> J;
  J.setIdentity();
  J.noalias() += A * ad + B * ad2 + C3 * term3 + C4 * term4;

  return J;
}

}  // namespace TetraPGA
