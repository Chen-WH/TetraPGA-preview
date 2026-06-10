#pragma once

#include "TetraPGA/Elements3D.hpp"
#include <type_traits>

namespace TetraPGA {

// Rigid-body motion
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto pga_rbm3(const Eigen::MatrixBase<Derived1>& M, const Eigen::MatrixBase<Derived2>& P)
-> Point3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 8);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 4);

  const Scalar M0 = M(0), M1 = M(1), M2 = M(2), M3 = M(3);
  const Scalar M4 = M(4), M5 = M(5), M6 = M(6), M7 = M(7);
  const Scalar P0 = P(0), P1 = P(1), P2 = P(2), P3 = P(3);

  const Scalar M00 = M0 * M0, M11 = M1 * M1, M22 = M2 * M2, M33 = M3 * M3;
  const Scalar M01 = M0*M1, M02 = M0*M2, M03 = M0*M3;
  const Scalar M12 = M1*M2, M13 = M1*M3, M23 = M2*M3;
  const Scalar M04 = M0*M4, M05 = M0*M5, M06 = M0*M6;
  const Scalar M15 = M1*M5, M16 = M1*M6, M17 = M1*M7;
  const Scalar M24 = M2*M4, M26 = M2*M6, M27 = M2*M7;
  const Scalar M34 = M3*M4, M35 = M3*M5, M37 = M3*M7;

  Point3D<Scalar> Pt;
  Pt(0) = P0 * (M00 + M11 - M22 - M33) + Scalar(2) * (P1 * (M12 - M03) + P2 * (M02 + M13) + P3 * (M04 + M17 + M26 - M35));
  Pt(1) = P1 * (M00 - M11 + M22 - M33) + Scalar(2) * (P0 * (M03 + M12) + P2 * (M23 - M01) + P3 * (M05 - M16 + M34 + M27));
  Pt(2) = P2 * (M00 - M11 - M22 + M33) + Scalar(2) * (P0 * (M13 - M02) + P1 * (M01 + M23) + P3 * (M06 + M15 - M24 + M37));
  Pt(3) = P3;

  return Pt;
}

// Commutator operator
template <typename Derived1, typename Derived2>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
auto pga_com23(const Eigen::MatrixBase<Derived1>& L, const Eigen::MatrixBase<Derived2>& P)
-> Point3D<std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>>
{
  using Scalar = std::common_type_t<typename Derived1::Scalar, typename Derived2::Scalar>;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 4);

  const Scalar L0 = L(0), L1 = L(1), L2 = L(2), L3 = L(3), L4 = L(4), L5 = L(5);
  const Scalar P0 = P(0), P1 = P(1), P2 = P(2), P3 = P(3);
  Point3D<Scalar> res;

  res(0) = L2 * P1 - L1 * P2 - L3 * P3;
  res(1) = L0 * P2 - L2 * P0 - L4 * P3;
  res(2) = L1 * P0 - L0 * P1 - L5 * P3;
  res(3) = 0;

  return res;
}

}  // namespace TetraPGA
