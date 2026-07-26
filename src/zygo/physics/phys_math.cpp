#include "phys_math.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <cmath>


namespace zygo {
namespace phys {

namespace
{
  // A helper axis becomes unusable when the segment is (almost) parallel to it.
  //
  // The test is on the COMPONENT of the (unit) segment direction, never on the length of the cross product: the length test only fires in a ~1e-6 rad band around the helper,
  // and by then the normalized result has already lost most of its digits - catastrophically so with Real = float, where the cross product itself is only accurate to ~1e-7 absolute.
  //
  // buildSegmentBasisX and buildSegmentBasisZ deliberately share this constant, so a box and a capsule built on the same segment switch helpers at the same angle.
  const Real SEGMENT_BASIS_MAX_HELPER_ALIGNMENT = (Real)0.9; // ~26 deg from the helper
}

void buildSegmentBasisX(
  Vector3 const & p1,
  Vector3 const & p2,
  Vector3 & x_axis,
  Vector3 & y_axis,
  Vector3 & z_axis
)
{
  x_axis = Vector3::safeNormalized( p2 - p1 );

  // Pick a helper vector not parallel to the axis (same rule as buildSegmentBasisZ).
  Vector3 helper;
  if ( std::abs( x_axis.z ) < SEGMENT_BASIS_MAX_HELPER_ALIGNMENT ) // is the segment (almost) vertical?
    helper = Vector3( 0.0, 0.0, 1.0 );
  else
    helper = Vector3( 1.0, 0.0, 0.0 ); // fall back to another helper axis.

  y_axis = Vector3::safeNormalized( helper.crossProduct( x_axis ) );
  z_axis = Vector3::safeNormalized( x_axis.crossProduct( y_axis ) );
}

void buildSegmentBasisZ(
  Vector3 const & p1,
  Vector3 const & p2,
  Vector3 & x_axis,
  Vector3 & y_axis,
  Vector3 & z_axis
)
{
  z_axis = Vector3::safeNormalized( p2 - p1 );

  // Pick a helper vector not parallel to the axis (same rule as buildSegmentBasisX).
  Vector3 helper;
  if ( std::abs( z_axis.z ) < SEGMENT_BASIS_MAX_HELPER_ALIGNMENT ) // is the segment (almost) vertical?
    helper = Vector3( 0.0, 0.0, 1.0 );
  else
    helper = Vector3( 1.0, 0.0, 0.0 );

  x_axis = Vector3::safeNormalized( helper.crossProduct( z_axis ) );
  y_axis = Vector3::safeNormalized( z_axis.crossProduct( x_axis ) );
}

Quaternion buildQuaternionFromAxes(
  Vector3 const & x_axis,
  Vector3 const & y_axis,
  Vector3 const & z_axis
)
{
  // Columns = the local basis axes in object coordinates.
  return buildQuaternionFromMat3( Mat3::fromColumns( x_axis, y_axis, z_axis ) );
}

Mat3 buildMat3FromQuaternion( Quaternion const & q )
{
  // Mirrors Quaternion::toRotationMatrix(), but builds a fixed Mat3 without heap allocations.
  Real s = q.getS();
  Vector3 v = q.getV();

  Real sx = s * v.x;
  Real sy = s * v.y;
  Real sz = s * v.z;

  Real xx = v.x * v.x;
  Real yy = v.y * v.y;
  Real zz = v.z * v.z;

  Real xy = v.x * v.y;
  Real xz = v.x * v.z;
  Real yz = v.y * v.z;

  return Mat3(
    REAL_ONE - REAL_TWO * (yy + zz),  REAL_TWO * (xy - sz),             REAL_TWO * (xz + sy),
    REAL_TWO * (xy + sz),             REAL_ONE - REAL_TWO * (xx + zz),  REAL_TWO * (yz - sx),
    REAL_TWO * (xz - sy),             REAL_TWO * (yz + sx),             REAL_ONE - REAL_TWO * (xx + yy)
  );
}

Quaternion buildQuaternionFromMat3( Mat3 const & mat )
{
  // Mirrors Quaternion::fromRotationMatrix() (Shepperd's method), but takes a Mat3.
  Real m00 = mat.m[0];
  Real m01 = mat.m[1];
  Real m02 = mat.m[2];

  Real m10 = mat.m[3];
  Real m11 = mat.m[4];
  Real m12 = mat.m[5];

  Real m20 = mat.m[6];
  Real m21 = mat.m[7];
  Real m22 = mat.m[8];

  Real trace = m00 + m11 + m22;

  Real s, x, y, z;

  if ( trace > REAL_ZERO )
  {
    Real t = std::sqrt( trace + REAL_ONE );
    s = REAL_HALF * t;

    Real inv4s = REAL_ONE / ( REAL_TWO * t ); // = 1 / (4*s)

    x = ( m21 - m12 ) * inv4s;
    y = ( m02 - m20 ) * inv4s;
    z = ( m10 - m01 ) * inv4s;
  } else
  if ( m00 > m11 && m00 > m22 )
  {
    Real t = std::sqrt( REAL_ONE + m00 - m11 - m22 );
    x = REAL_HALF * t;

    Real inv4x = REAL_ONE / ( REAL_TWO * t ); // = 1 / (4*x)

    s = ( m21 - m12 ) * inv4x;
    y = ( m01 + m10 ) * inv4x;
    z = ( m02 + m20 ) * inv4x;
  } else
  if ( m11 > m22 )
  {
    Real t = std::sqrt( REAL_ONE + m11 - m00 - m22 );
    y = REAL_HALF * t;

    Real inv4y = REAL_ONE / ( REAL_TWO * t ); // = 1 / (4*y)

    s = ( m02 - m20 ) * inv4y;
    x = ( m01 + m10 ) * inv4y;
    z = ( m12 + m21 ) * inv4y;
  } else
  {
    Real t = std::sqrt( REAL_ONE + m22 - m00 - m11 );
    z = REAL_HALF * t;

    Real inv4z = REAL_ONE / ( REAL_TWO * t ); // = 1 / (4*z)

    s = ( m10 - m01 ) * inv4z;
    x = ( m02 + m20 ) * inv4z;
    y = ( m12 + m21 ) * inv4z;
  }

  Quaternion q( s, Vector3( x, y, z ) );
  q.normalize();

  return q;
}

Mat3 calcParallelAxisTerm( double mass, Vector3 const & d )
{
  // Steiner: I_shift = m * ( (d . d) * E  -  d * d^T )
  double d2 = d.lengthSqr();

  return ( Mat3::identity() * d2 - Mat3::outerProduct( d, d ) ) * mass;
}

namespace
{
  // Everything an inertia tensor AND its inverse must satisfy: finite, symmetric, positive definite.
  // Every threshold is relative to the magnitude of the tensor.
  void validateSpdTensor( Mat3 const & mat, double rel_eps )
  {
    // 1) All elements are finite.
    ZgAssert( mat.isFinite() );

    double const s = mat.maxAbsElement();
    ZgAssert( s > 0.0 ); // a zero tensor is never a valid one

    // 2) Symmetry (relative - a rotated tensor is symmetric only to a few ulps).
    ZgAssert( mat.isSymmetric( rel_eps ) );

    // 3) Sylvester's criterion, each leading minor against its OWN power of the scale.
    //    This is what makes the check work for a 5 g pad and a 3 t frame alike:
    //    their determinants differ by ~18 decades, their CONDITION does not.
    ZgAssert( mat.m[0] > rel_eps * s );                                  // 1st minor
    ZgAssert( mat.m[4] > rel_eps * s );
    ZgAssert( mat.m[8] > rel_eps * s );

    double const det2x2 = mat.m[0] * mat.m[4] - mat.m[1] * mat.m[3];     // 2nd minor
    ZgAssert( det2x2 > rel_eps * s * s );

    double const det3x3 = mat.determinant();                             // 3rd minor
    ZgAssert( det3x3 > rel_eps * s * s * s );
  }
}

void validateInertiaTensor( Mat3 const & mat, double rel_eps )
{
  validateSpdTensor( mat, rel_eps );

  // The triangle inequality of a REAL inertia tensor.
  //
  //   Ixx + Iyy - Izz = 2 * integral( z^2 dm ) >= 0   (and cyclic)
  //
  // It holds in ANY orthonormal frame, not just the principal one, so it can be read
  // straight off the diagonal. It catches mass distributions that are SPD but
  // physically impossible - a wrong parallel-axis sign, a shape added twice,
  // millimeters left unconverted - which Sylvester alone happily accepts.
  //
  // One-sided and relative: an infinitely thin plate hits equality exactly.
  double const e = mat.maxAbsElement() * rel_eps;

  ZgAssert( mat.m[0] + mat.m[4] - mat.m[8] >= -e );
  ZgAssert( mat.m[4] + mat.m[8] - mat.m[0] >= -e );
  ZgAssert( mat.m[8] + mat.m[0] - mat.m[4] >= -e );
}

void validateInverseInertiaTensor( Mat3 const & mat, double rel_eps )
{
  // Everything except the triangle inequality: it does NOT survive inversion.
  validateSpdTensor( mat, rel_eps );
}

} // namespace phys
} // namespace zygo
