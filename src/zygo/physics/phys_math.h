#pragma once

// Math helpers of the physics engine: segment bases, inertia tensors.

#include <zygo/math/quaternion/quaternion.h>
#include <zygo/math/matrix/small_fast_matrix/mat3.h>


namespace zygo {
namespace phys {

// Orthonormal basis with the X axis along the segment p1 -> p2.
// Used for boxes (size.x = length lies along the segment).
void buildSegmentBasisX(
  Vector3 const & p1,
  Vector3 const & p2,
  Vector3 & x_axis,
  Vector3 & y_axis,
  Vector3 & z_axis
);

// Orthonormal basis with the Z axis along the segment p1 -> p2.
// Used for cylinders and capsules (their axis is the local Z, matching the GLUT/ZygoGL drawing convention).
void buildSegmentBasisZ(
  Vector3 const & p1,
  Vector3 const & p2,
  Vector3 & x_axis,
  Vector3 & y_axis,
  Vector3 & z_axis
);

// Quaternion whose rotation matrix has the given axes as columns.
Quaternion buildQuaternionFromAxes(
  Vector3 const & x_axis,
  Vector3 const & y_axis,
  Vector3 const & z_axis
);

// Rotation matrix of the quaternion, as a fixed Mat3 (mirrors Quaternion::toRotationMatrix).
Mat3 buildMat3FromQuaternion( Quaternion const & q );

// Quaternion from a rotation Mat3 (mirrors Quaternion::fromRotationMatrix, Shepperd's method).
Quaternion buildQuaternionFromMat3( Mat3 const & mat );

// Parallel-axis (Steiner) term for translating an inertia tensor by d.
Mat3 calcParallelAxisTerm( double mass, Vector3 const & d );

// Relative tolerance of the inertia-tensor validators.
//
// RELATIVE, never absolute: the determinant of a 3x3 scales as (element magnitude)^3, so a 5 g foot pad and a 3 t frame are ~18 decades apart in determinant while being equally well conditioned.
// A fixed threshold cannot serve both.
const double INERTIA_VALIDATION_REL_EPS = 1e-11;

// Asserts that mat is a valid 3x3 inertia tensor: finite, symmetric, positive definite, and satisfying the triangle inequality Ixx + Iyy >= Izz (true in any orthonormal frame).
void validateInertiaTensor( Mat3 const & mat, double rel_eps = INERTIA_VALIDATION_REL_EPS );

// The same for an INVERSE inertia tensor: everything except the triangle inequality, which does not survive inversion.
void validateInverseInertiaTensor( Mat3 const & mat, double rel_eps = INERTIA_VALIDATION_REL_EPS );

} // namespace phys
} // namespace zygo
