#include "vec2.h"
#include <zygo/math/common/scalar.h>
#include <zygo/core/assert.h>


namespace zygo {

Vector2::Vector2()
  : x ( REAL_ZERO )
  , y ( REAL_ZERO )
{
}

Vector2::Vector2( Real x, Real y )
  : x ( x )
  , y ( y )
{
}

void Vector2::set( Real x, Real y )
{
  this->x = x;
  this->y = y;
}

bool Vector2::isEqual( Vector2 const & v, Real eps ) const
{
  return
    eq( x, v.x, eps ) &&
    eq( y, v.y, eps );
}

bool Vector2::isEqual( Real x, Real y, Real eps ) const
{
  return
    eq( x, this->x, eps ) &&
    eq( y, this->y, eps );
}

Real Vector2::length() const
{
  return std::sqrt( lengthSqr() );
}

Real Vector2::lengthSqr() const
{
  return sqr( x ) + sqr( y );
}

Real Vector2::distTo( Vector2 const & p ) const
{
  return std::sqrt( distToSqr( p ) );
}

Real Vector2::distToSqr( Vector2 const & p ) const
{
  return sqr( p.x - x ) + sqr( p.y - y );
}

bool Vector2::isZeroVector() const
{
  return isZero( x ) && isZero( y );
}

Vector2 Vector2::getNormal() const
{
  return Vector2( -y, x );
}

void Vector2::normalize()
{
  Real len = length();
  if ( isZero( len ) )
  {
    x = REAL_ONE;
    y = REAL_ZERO;
    return;
  }

  x /= len;
  y /= len;
}

// -------------------------------------------------------------------------- rotation
void Vector2::rotate( Real alpha )
{
  Real sin_alpha = std::sin( alpha );
  Real cos_alpha = std::cos( alpha );

  rotate( sin_alpha, cos_alpha );
}

void Vector2::rotateSmallAngleUsingSin( Real sin_alpha )
{
  Real cos_alpha = std::sqrt( 1 - sqr( sin_alpha ) );
  rotate( sin_alpha, cos_alpha );
}

void Vector2::rotate( Real sin_alpha, Real cos_alpha )
{
  Real x2 = x * cos_alpha - y * sin_alpha;
  Real y2 = x * sin_alpha + y * cos_alpha;

  x = x2;
  y = y2;
}

// -------------------------------------------------------------------------- operators
Vector2 & Vector2::operator += ( Vector2 const & v )
{
  x += v.x;
  y += v.y;
  return *this;
}

Vector2 & Vector2::operator -= ( Vector2 const & v )
{
  x -= v.x;
  y -= v.y;
  return *this;
}

Vector2 & Vector2::operator *= ( Real d )
{
  x *= d;
  y *= d;
  return *this;
}

Vector2 & Vector2::operator /= ( Real d )
{
  ZgAssert( !isZero( d ) );
  x /= d;
  y /= d;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Vector2 Vector2::operator - () const
{
  return Vector2( -x, -y );
}

// -------------------------------------------------------------------------- binary operators
Vector2 Vector2::operator + ( Vector2 const & v ) const
{
  return Vector2( 
    x + v.x,
    y + v.y
  );
}

Vector2 Vector2::operator - ( Vector2 const & v ) const
{
  return Vector2(
    x - v.x,
    y - v.y
  );
}

Vector2 Vector2::operator * ( Real k ) const
{
  return Vector2(
    x * k,
    y * k
  );
}

Vector2 Vector2::operator / ( Real k ) const
{
  ZgAssert( !isZero( k ) );
  return Vector2(
    x / k,
    y / k
  );
}

Real Vector2::operator * ( Vector2 const & v ) const
{
  return x * v.x 
       + y * v.y;
}

Real Vector2::operator ^ ( Vector2 const & v ) const
{
  return x * v.y
       - y * v.x;
}

// -------------------------------------------------------------------------- 
Vector2 Vector2::lerpVectors( Vector2 const& a, Vector2 const& b, Real t )
{
  return Vector2(
    a.x + (b.x - a.x) * t,
    a.y + (b.y - a.y) * t
  );
}

} // namespace zygo
