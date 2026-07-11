#include "quaternion.h"
#include <zygo/math/common/scalar.h>
#include <zygo/core/assert.h>


namespace zygo {

Quaternion::Quaternion()
  : s ( REAL_ONE ) // identity quaternion by default
{
}

Quaternion::Quaternion( Real s, Vector3 const & v )
  : s ( s )
  , v ( v )
{
}

Quaternion::Quaternion( Real s, Real x, Real y, Real z )
  : s ( s )
  , v ( x, y, z )
{
}

Quaternion::Quaternion( Quaternion const& q )
  : s ( q.s )
  , v ( q.v )
{
}

void Quaternion::set( Real s, Vector3 const& v )
{
  this->s = s;
  this->v = v;
}

void Quaternion::set( Real s, Real x, Real y, Real z )
{
  this->s = s;
  v.x = x;
  v.y = y;
  v.z = z;
}

Real Quaternion::getAngle() const
{
  if ( abs( s ) > COS_OF_1DIV2 )
  {
    Real a = REAL_TWO * std::asin( v.length() );
    if ( s < REAL_ZERO )
      return PI_MUL_2 - a;

    return a;
  }

  return REAL_TWO * std::acos( s );
}

Quaternion Quaternion::getConjugate() const
{
  return Quaternion( s, -v );
}

Quaternion Quaternion::getInverse() const
{
  Quaternion q = getConjugate();
  q.normalizeSqr();
  return q;
}

Real Quaternion::length() const
{
  return std::sqrt( lengthSqr() );
}

Real Quaternion::lengthSqr() const
{
  return sqr( s ) + v.lengthSqr();
}

void Quaternion::normalize()
{
  Real len = length();
  ZgAssert( !isZero( len ) );

  s /= len;
  v /= len;
}

void Quaternion::normalizeSqr() // used only in the getInverse() method!
{
  Real len_sqr = lengthSqr();
  ZgAssert( !isZero( len_sqr ) );

  s /= len_sqr;
  v /= len_sqr;
}

void Quaternion::setIdentity() // make identity
{
  s = REAL_ONE;
  v.set( REAL_ZERO, REAL_ZERO, REAL_ZERO );
}

Real Quaternion::cosPhiBetween( Quaternion const & q ) const
{
  Real len1 =   length();
  Real len2 = q.length();

  ZgAssert( !isZero( len1 ) && !isZero( len2 ) );

  return dotProduct( q ) / (len1 * len2);
}

bool Quaternion::isEqual( Quaternion const& q, Real epsilon ) const
{
  return eq( s, q.s, epsilon ) &&
    v.isEqual( q.v, epsilon );
}



void Quaternion::pow( Real power )
{
  // Raising to the power of 0 should return identity quaternion
  if ( isZero( power ) )
  {
    setIdentity();
    return;
  }

  Real len = length();
  //ZgAssert( !isZero( len ) );

  Real cosa = s / len;
  Real angle;

  if ( abs( cosa ) > COS_OF_1DIV2 )
  {
    // Scalar component is close to 1; using it to recover angle would lose precision.
    // Instead, we use the non-scalar components since sin() is accurate around 0.

    // Prevent a division by 0 later
    Real v_len_sqr = v.lengthSqr();

    if ( isZero( v_len_sqr ) )
    {
      // Equivalent to raising a real number to a power
      s = std::pow( s, power );
      return;
    }

    angle = std::asin( std::sqrt( v_len_sqr ) / len );
  } else
  {
    // Scalar component is small, shouldn't cause loss of precision
    angle = std::acos( cosa );
  }

  Real new_angle = angle * power;
  Real d = std::sin( new_angle ) / std::sin( angle );
  Real p = std::pow( len, power - REAL_ONE );
  Real dp = d * p;

  s = cos( new_angle ) * len * p;
  v *= dp;
}

} // namespace zygo
