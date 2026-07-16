#pragma once

// Math helpers of the physics engine: segment bases, inertia tensors.

#include <zygo/math/quaternion/quaternion.h>
#include <zygo/math/matrix/matrix.h>


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

// Parallel-axis (Steiner) term for translating an inertia tensor by d.
Matrix calcParallelAxisTerm( double mass, Vector3 const & d );

// Asserts that mat is a valid 3x3 inertia tensor: finite, symmetric, SPD.
// det_eps should NOT be very small.
void validateInertiaTensor( Matrix const & mat, double symmetry_eps = SMALL_EPSILON, double det_eps = 1e-14, double det_eps_small = 1e-18 );

} // namespace phys
} // namespace zygo
