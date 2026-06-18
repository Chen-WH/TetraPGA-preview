#pragma once

#include <stdexcept>
#include <string>
#include <type_traits>

#include "TetraPGA/Motor.hpp"
#include "TetraPGA/PGA.hpp"

namespace TetraPGA::joint {

constexpr char kRevolute = 'R';
constexpr char kPrismatic = 'P';
constexpr char kPlanarRoot = 'A';  // PlAnar root joint; 'P' is reserved for prismatic.
constexpr char kFreeFlyerRoot = 'F';

enum class Type {
  kRevolute,
  kPrismatic,
  kPlanarRoot,
  kFreeFlyerRoot,
};

inline const char* name(const char joint_type) {
  switch (joint_type) {
    case kRevolute:
      return "revolute";
    case kPrismatic:
      return "prismatic";
    case kPlanarRoot:
      return "planar root";
    case kFreeFlyerRoot:
      return "free-flyer root";
    default:
      return "unknown";
  }
}

[[noreturn]] inline void throwUnsupportedType(
    const char joint_type, const int joint_index, const char* context) {
  throw std::invalid_argument(
      std::string(context) + ": unsupported joint type '" + std::string(1, joint_type) +
      "' at index " + std::to_string(joint_index));
}

[[noreturn]] inline void throwUnimplemented(
    const char joint_type, const int joint_index, const char* context) {
  throw std::logic_error(
      std::string(context) + ": " + name(joint_type) +
      " joint interface is declared but not implemented at index " +
      std::to_string(joint_index));
}

inline Type typeFromCode(const char joint_type, const int joint_index, const char* context) {
  switch (joint_type) {
    case kRevolute:
      return Type::kRevolute;
    case kPrismatic:
      return Type::kPrismatic;
    case kPlanarRoot:
      return Type::kPlanarRoot;
    case kFreeFlyerRoot:
      return Type::kFreeFlyerRoot;
    default:
      throwUnsupportedType(joint_type, joint_index, context);
  }
}

inline int dof(const char joint_type, const int joint_index, const char* context) {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return 1;
    case Type::kPlanarRoot:
      return 3;
    case Type::kFreeFlyerRoot:
      return 6;
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

inline bool isRootOnly(const char joint_type) {
  return joint_type == kPlanarRoot || joint_type == kFreeFlyerRoot;
}

inline bool isImplemented(const char joint_type) {
  return joint_type == kRevolute || joint_type == kPrismatic;
}

inline void validateRootPlacement(
    const char joint_type, const int parent_index, const int joint_index, const char* context) {
  if (isRootOnly(joint_type) && parent_index != 0) {
    throw std::invalid_argument(
        std::string(context) + ": " + name(joint_type) +
        " joint must be attached to the root body; joint index " +
        std::to_string(joint_index) + " has parent " + std::to_string(parent_index));
  }
}

template <typename DerivedAxis, typename ScalarQ, typename DerivedParent>
EIGEN_STRONG_INLINE auto transformFromParent(
    const char joint_type, const Eigen::MatrixBase<DerivedAxis>& axis,
    const ScalarQ q, const Eigen::MatrixBase<DerivedParent>& parent_transform,
    const int joint_index, const char* context)
    -> Motor3D<std::common_type_t<typename DerivedAxis::Scalar,
                                  std::common_type_t<ScalarQ, typename DerivedParent::Scalar>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
      return ga_mul_exp_R(axis, ScalarQ(0.5) * q, parent_transform);
    case Type::kPrismatic:
      return ga_mul_exp_P(axis, ScalarQ(0.5) * q, parent_transform);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedAxis, typename ScalarQ, typename DerivedPlacement>
EIGEN_STRONG_INLINE auto relativeTransform(
    const char joint_type, const Eigen::MatrixBase<DerivedAxis>& axis,
    const ScalarQ q, const Eigen::MatrixBase<DerivedPlacement>& placement,
    const int joint_index, const char* context)
    -> Motor3D<std::common_type_t<typename DerivedAxis::Scalar,
                                  std::common_type_t<ScalarQ, typename DerivedPlacement::Scalar>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
      return ga_mul_exp_R(axis, ScalarQ(0.5) * q, placement);
    case Type::kPrismatic:
      return ga_mul_exp_P(axis, ScalarQ(0.5) * q, placement);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedParent, typename DerivedAxis>
EIGEN_STRONG_INLINE auto axisFromParent(
    const char joint_type, const Eigen::MatrixBase<DerivedParent>& parent_transform,
    const Eigen::MatrixBase<DerivedAxis>& axis0,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedParent::Scalar, typename DerivedAxis::Scalar>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return ga_rbm(parent_transform, axis0);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedAxis>
EIGEN_STRONG_INLINE auto axisMetric(
    const char joint_type, const Eigen::MatrixBase<DerivedAxis>& axis,
    const int joint_index, const char* context)
    -> Eigen::Matrix<typename DerivedAxis::Scalar, 1, 6> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return ga_metric(axis);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedAxis, typename DerivedParentVelocity>
EIGEN_STRONG_INLINE auto axisDot(
    const char joint_type, const Eigen::MatrixBase<DerivedAxis>& axis,
    const Eigen::MatrixBase<DerivedParentVelocity>& parent_velocity,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedAxis::Scalar,
                                 typename DerivedParentVelocity::Scalar>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return ga_com(axis, parent_velocity);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedAxis, typename DerivedAxisDot, typename DerivedParentVelocity,
          typename DerivedParentAcceleration>
EIGEN_STRONG_INLINE auto axisDDot(
    const char joint_type, const Eigen::MatrixBase<DerivedAxis>& axis,
    const Eigen::MatrixBase<DerivedAxisDot>& axis_dot,
    const Eigen::MatrixBase<DerivedParentVelocity>& parent_velocity,
    const Eigen::MatrixBase<DerivedParentAcceleration>& parent_acceleration,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedAxis::Scalar,
                                 std::common_type_t<typename DerivedAxisDot::Scalar,
                                                    std::common_type_t<typename DerivedParentVelocity::Scalar,
                                                                       typename DerivedParentAcceleration::Scalar>>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return ga_com(axis_dot, parent_velocity) + ga_com(axis, parent_acceleration);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedParentVelocity, typename DerivedAxis, typename ScalarQd>
EIGEN_STRONG_INLINE auto spatialVelocity(
    const char joint_type, const Eigen::MatrixBase<DerivedParentVelocity>& parent_velocity,
    const Eigen::MatrixBase<DerivedAxis>& axis, const ScalarQd dq,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedParentVelocity::Scalar,
                                 std::common_type_t<typename DerivedAxis::Scalar, ScalarQd>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return parent_velocity + dq * axis;
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedParentAcceleration, typename DerivedAxis,
          typename DerivedAxisDot, typename ScalarQd, typename ScalarQdd>
EIGEN_STRONG_INLINE auto spatialAcceleration(
    const char joint_type, const Eigen::MatrixBase<DerivedParentAcceleration>& parent_acceleration,
    const Eigen::MatrixBase<DerivedAxis>& axis,
    const Eigen::MatrixBase<DerivedAxisDot>& axis_dot,
    const ScalarQd dq, const ScalarQdd ddq,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedParentAcceleration::Scalar,
                                 std::common_type_t<typename DerivedAxis::Scalar,
                                                    std::common_type_t<typename DerivedAxisDot::Scalar,
                                                                       std::common_type_t<ScalarQd, ScalarQdd>>>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return parent_acceleration + dq * axis_dot + ddq * axis;
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedAxis, typename DerivedVelocity, typename ScalarQd>
EIGEN_STRONG_INLINE auto coriolisBias(
    const char joint_type, const Eigen::MatrixBase<DerivedAxis>& axis,
    const Eigen::MatrixBase<DerivedVelocity>& velocity,
    const ScalarQd dq, const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedAxis::Scalar,
                                 std::common_type_t<typename DerivedVelocity::Scalar, ScalarQd>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return dq * ga_com(axis, velocity);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedTransform, typename DerivedParentVelocity>
EIGEN_STRONG_INLINE auto localParentVelocity(
    const char joint_type, const Eigen::MatrixBase<DerivedTransform>& relative_transform,
    const Eigen::MatrixBase<DerivedParentVelocity>& parent_velocity,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedTransform::Scalar,
                                 typename DerivedParentVelocity::Scalar>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return ga_AdM(relative_transform, parent_velocity);
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedParentVelocityLocal, typename DerivedAxis, typename ScalarQd>
EIGEN_STRONG_INLINE auto localSpatialVelocity(
    const char joint_type, const Eigen::MatrixBase<DerivedParentVelocityLocal>& parent_velocity_local,
    const Eigen::MatrixBase<DerivedAxis>& axis, const ScalarQd dq,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedParentVelocityLocal::Scalar,
                                 std::common_type_t<typename DerivedAxis::Scalar, ScalarQd>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return parent_velocity_local + dq * axis;
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

template <typename DerivedTransform, typename DerivedParentAcceleration,
          typename DerivedAxis, typename DerivedParentVelocityLocal,
          typename ScalarQd, typename ScalarQdd>
EIGEN_STRONG_INLINE auto localSpatialAcceleration(
    const char joint_type, const Eigen::MatrixBase<DerivedTransform>& relative_transform,
    const Eigen::MatrixBase<DerivedParentAcceleration>& parent_acceleration,
    const Eigen::MatrixBase<DerivedAxis>& axis,
    const Eigen::MatrixBase<DerivedParentVelocityLocal>& parent_velocity_local,
    const ScalarQd dq, const ScalarQdd ddq,
    const int joint_index, const char* context)
    -> Line3D<std::common_type_t<typename DerivedTransform::Scalar,
                                 std::common_type_t<typename DerivedParentAcceleration::Scalar,
                                                    std::common_type_t<typename DerivedAxis::Scalar,
                                                                       std::common_type_t<typename DerivedParentVelocityLocal::Scalar,
                                                                                          std::common_type_t<ScalarQd, ScalarQdd>>>>>> {
  switch (typeFromCode(joint_type, joint_index, context)) {
    case Type::kRevolute:
    case Type::kPrismatic:
      return ga_AdM(relative_transform, parent_acceleration) +
             dq * ga_com(axis, parent_velocity_local) + ddq * axis;
    case Type::kPlanarRoot:
    case Type::kFreeFlyerRoot:
      throwUnimplemented(joint_type, joint_index, context);
  }
  throwUnsupportedType(joint_type, joint_index, context);
}

}  // namespace TetraPGA::joint
