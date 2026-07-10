#include "vec4.h"
#include <zygo/math/common/scalar.h>
#include <zygo/core/assert.h>


namespace zygo {

Vector4::Vector4()
  : x ( REAL_ZERO )
  , y ( REAL_ZERO )
  , z ( REAL_ZERO )
  , t ( REAL_ZERO )
{
}

Vector4::Vector4( Real x, Real y, Real z, Real t )
  : x ( x )
  , y ( y )
  , z ( z )
  , t ( t )
{
}

void Vector4::reset()
{
  x = REAL_ZERO;
  y = REAL_ZERO;
  z = REAL_ZERO;
  t = REAL_ZERO;
}

void Vector4::set( Real x, Real y, Real z, Real t )
{
  this->x = x;
  this->y = y;
  this->z = z;
  this->t = t;
}

bool Vector4::isEqual( Vector4 const & v, Real eps ) const
{
  return
    eq( x, v.x, eps ) &&
    eq( y, v.y, eps ) &&
    eq( z, v.z, eps ) &&
    eq( t, v.t, eps );
}

bool Vector4::isEqual( Real x, Real y, Real z, Real t, Real eps ) const
{
  return
    eq( x, this->x, eps ) &&
    eq( y, this->y, eps ) &&
    eq( z, this->z, eps ) &&
    eq( t, this->t, eps );
}

Real Vector4::length() const
{
  return std::sqrt( lengthSqr() );
}

Real Vector4::lengthSqr() const
{
  return sqr( x ) + sqr( y ) + sqr( z ) + sqr( t );
}

Real Vector4::distTo( Vector4 const & p ) const
{
  return std::sqrt( distToSqr( p ) );
}

Real Vector4::distToSqr( Vector4 const & p ) const
{
  return
      sqr( p.x - x ) 
    + sqr( p.y - y ) 
    + sqr( p.z - z )
    + sqr( p.t - t )
    ;
}

bool Vector4::isZeroVector() const
{
  return isZero( x ) && isZero( y ) && isZero( z ) && isZero( t );
}

void Vector4::normalize()
{
  Real len = length();
  if ( isZero( len ) )
  {
    x = REAL_ZERO;
    y = REAL_ZERO;
    z = REAL_ZERO;
    t = REAL_ONE;
    return;
  }

  x /= len;
  y /= len;
  z /= len;
  t /= len;
}


// -------------------------------------------------------------------------- operators
Vector4 & Vector4::operator += ( Vector4 const & v )
{
  x += v.x;
  y += v.y;
  z += v.z;
  t += v.t;
  return *this;
}

Vector4 & Vector4::operator -= ( Vector4 const & v )
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
  t -= v.t;
  return *this;
}

Vector4 & Vector4::operator *= ( Real d )
{
  x *= d;
  y *= d;
  z *= d;
  t *= d;
  return *this;
}

Vector4 & Vector4::operator /= ( Real d )
{
  ZgAssert( !isZero( d ) );
  const Real inv = REAL_ONE / d;
  x *= inv;
  y *= inv;
  z *= inv;
  t *= inv;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Vector4 Vector4::operator - () const
{
  return Vector4( -x, -y, -z, -t );
}

// -------------------------------------------------------------------------- binary operators
Vector4 Vector4::operator + ( Vector4 const & v ) const
{
  return Vector4(
    x + v.x,
    y + v.y,
    z + v.z,
    t + v.t
  );
}

Vector4 Vector4::operator - ( Vector4 const & v ) const
{
  return Vector4(
    x - v.x,
    y - v.y,
    z - v.z,
    t - v.t
  );
}

Vector4 Vector4::operator * ( Real d ) const
{
  return Vector4(
    x * d,
    y * d,
    z * d,
    t * d
  );
}

Vector4 Vector4::operator / ( Real d ) const
{
  ZgAssert( !isZero( d ) );
  return Vector4(
    x / d,
    y / d,
    z / d,
    t / d
  );
}

Real Vector4::operator * ( Vector4 const & v ) const
{
  return 
      x * v.x
    + y * v.y
    + z * v.z
    + t * v.t;
}

} // namespace zygo
