#include "vec3short.h"
#include <zygo/core/assert.h>
#include <cmath>


namespace zygo {

Vector3Short::Vector3Short()
  : x ( 0 )
  , y ( 0 )
  , z ( 0 )
{
}

Vector3Short::Vector3Short( i16 x, i16 y, i16 z )
  : x ( x )
  , y ( y )
  , z ( z )
{
}

void Vector3Short::set( i16 x, i16 y, i16 z )
{
  this->x = x;
  this->y = y;
  this->z = z;
}

bool Vector3Short::isEqual( Vector3Short const & v ) const
{
  return
    x == v.x &&
    y == v.y &&
    z == v.z;
}

bool Vector3Short::isEqual( i16 x, i16 y, i16 z ) const
{
  return
    x == this->x &&
    y == this->y &&
    z == this->z;
}

bool Vector3Short::xyEquals( Vector3Short const& v ) const
{
  return
    v.x == x &&
    v.y == y;
}

inline bool Vector3Short::isZeroVector() const
{
  return x == 0 && y == 0 && z == 0;
}

bool Vector3Short::isLessByXY( Vector3Short const& v ) const
{
  return x < v.x
    || (x == v.x && y < v.y);
}

int Vector3Short::distManhattanXY( Vector3Short const& v ) const
{
  int dx = (int)v.x - (int)x;
  int dy = (int)v.y - (int)y;
  return std::abs( dx ) + std::abs( dy );
}

// -------------------------------------------------------------------------- operators
bool Vector3Short::operator == ( Vector3Short const & v ) const
{
  return isEqual( v );
}
bool Vector3Short::operator != ( Vector3Short const & v ) const
{
  return !isEqual( v );
}

Vector3Short & Vector3Short::operator += ( Vector3Short const & v )
{
  x += v.x;
  y += v.y;
  z += v.z;
  return *this;
}

Vector3Short & Vector3Short::operator -= ( Vector3Short const & v )
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
  return *this;
}

Vector3Short & Vector3Short::operator *= ( i16 d )
{
  x *= d;
  y *= d;
  z *= d;
  return *this;
}

Vector3Short & Vector3Short::operator /= ( i16 d )
{
  ZgAssert( d != 0 );
  x /= d;
  y /= d;
  z /= d;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Vector3Short Vector3Short::operator - () const
{
  return Vector3Short( -x, -y, -z );
}

// -------------------------------------------------------------------------- binary operators
Vector3Short Vector3Short::operator + ( Vector3Short const & v ) const
{
  return Vector3Short( 
    x + v.x,
    y + v.y,
    z + v.z
  );
}

Vector3Short Vector3Short::operator - ( Vector3Short const & v ) const
{
  return Vector3Short(
    x - v.x,
    y - v.y,
    z - v.z
  );
}

Vector3Short Vector3Short::operator * ( i16 k ) const
{
  return Vector3Short(
    x * k,
    y * k,
    z * k
  );
}

Vector3Short Vector3Short::operator / ( i16 k ) const
{
  ZgAssert( k != 0 );
  return Vector3Short(
    x / k,
    y / k,
    z / k
  );
}

i16 Vector3Short::operator * ( Vector3Short const & v ) const
{
  return x * v.x 
       + y * v.y
       + z * v.z;
}

} // namespace zygo
