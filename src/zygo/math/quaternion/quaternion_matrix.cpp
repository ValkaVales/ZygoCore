#include "quaternion.h"
#include <zygo/core/assert.h>
#include <cmath> // sqrt


namespace zygo {

Matrix Quaternion::toRotationMatrix() const
{
  Matrix mat( 3, 3 );
  toRotationMatrix( mat );
  return mat;
}

void Quaternion::toRotationMatrix( Matrix & mat ) const
{
  ZgAssert( mat.dimX() == 3 && mat.dimY() == 3 );

  //Real ss = s * s;
  Real sx = s * v.x;
  Real sy = s * v.y;
  Real sz = s * v.z;

  Real xx = v.x * v.x;
  Real yy = v.y * v.y;
  Real zz = v.z * v.z;

  Real xy = v.x * v.y;
  Real xz = v.x * v.z;
  Real yz = v.y * v.z;

  // diagonal
  mat.setAt( 0, 0,  REAL_ONE - REAL_TWO * (yy + zz) );
  mat.setAt( 1, 1,  REAL_ONE - REAL_TWO * (xx + zz) );
  mat.setAt( 2, 2,  REAL_ONE - REAL_TWO * (xx + yy) );

  //
  mat.setAt( 0, 1,  REAL_TWO * (xy - sz) );
  mat.setAt( 0, 2,  REAL_TWO * (xz + sy) );

  mat.setAt( 1, 0,  REAL_TWO * (xy + sz) );
  mat.setAt( 1, 2,  REAL_TWO * (yz - sx) );

  mat.setAt( 2, 0,  REAL_TWO * (xz - sy) );
  mat.setAt( 2, 1,  REAL_TWO * (yz + sx) );
}

Quaternion Quaternion::fromRotationMatrix( Matrix const& mat )
{
  ZgAssert( mat.dimX() == 3 && mat.dimY() == 3 );

  Real m00 = mat.get( 0, 0 );
  Real m01 = mat.get( 0, 1 );
  Real m02 = mat.get( 0, 2 );

  Real m10 = mat.get( 1, 0 );
  Real m11 = mat.get( 1, 1 );
  Real m12 = mat.get( 1, 2 );

  Real m20 = mat.get( 2, 0 );
  Real m21 = mat.get( 2, 1 );
  Real m22 = mat.get( 2, 2 );

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

} // namespace zygo
