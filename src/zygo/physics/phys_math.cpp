#include "phys_math.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <cmath>


namespace zygo {
namespace phys {

void buildSegmentBasisX(
  Vector3 const & p1,
  Vector3 const & p2,
  Vector3 & x_axis,
  Vector3 & y_axis,
  Vector3 & z_axis
)
{
  x_axis = Vector3::safeNormalized( p2 - p1 );

  Vector3 world_up( 0.0, 0.0, 1.0 );
  Vector3 tmp = world_up.crossProduct( x_axis );

  if ( tmp.lengthSqr() < 1e-12 )
  {
    // The segment is (almost) vertical - fall back to another helper axis.
    Vector3 fallback( 1.0, 0.0, 0.0 );
    tmp = fallback.crossProduct( x_axis );
  }

  y_axis = Vector3::safeNormalized( tmp );
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

  // Pick a helper vector not parallel to the axis.
  Vector3 helper;
  if ( std::abs( z_axis.z ) < 0.9 )
    helper = Vector3( 0.0, 0.0, 1.0 );
  else
    helper = Vector3( 1.0, 0.0, 0.0 ); // the segment is (almost) vertical

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

void validateInertiaTensor( Mat3 const & mat, double symmetry_eps, double det_eps, double det_eps_small )
{
  // 1) All elements are finite.
  ZgAssert( mat.isFinite() );

  // 2) Symmetry.
  ZgAssert( mat.isSymmetric( symmetry_eps ) );

  // 3) Positive diagonal.
  ZgAssert( mat.m[0] > 0.0 );
  ZgAssert( mat.m[4] > 0.0 );
  ZgAssert( mat.m[8] > 0.0 );

  // 4) Sylvester's criterion for an SPD matrix.
  ZgAssert( mat.m[0] > det_eps ); // first leading minor

  double det2x2 = mat.m[0] * mat.m[4] - mat.m[1] * mat.m[3]; // second leading minor
  ZgAssert( det2x2 > det_eps );

  double det3x3 = mat.determinant();
  ZgAssert( det3x3 > det_eps_small );

  // 5) A very rough check that the matrix is not almost degenerate
  //    (the determinant threshold is scaled by the magnitude of the elements).
  double max_abs = mat.maxAbsElement();
  double coeff = std::max( 1.0, max_abs * max_abs * max_abs );
  double scaled_det_eps = det_eps_small * coeff;
  ZgAssert( std::abs( det3x3 ) > scaled_det_eps );
}

} // namespace phys
} // namespace zygo
