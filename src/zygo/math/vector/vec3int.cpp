#include "vec3int.h"
#include <zygo/core/assert.h>


namespace zygo {

Vector3Int::Vector3Int()
  : x ( 0 )
  , y ( 0 )
  , z ( 0 )
{
}

Vector3Int::Vector3Int( i32 x, i32 y, i32 z )
  : x ( x )
  , y ( y )
  , z ( z )
{
}

void Vector3Int::set( i32 x, i32 y, i32 z )
{
  this->x = x;
  this->y = y;
  this->z = z;
}

bool Vector3Int::isEqual( Vector3Int const & v ) const
{
  return
    x == v.x &&
    y == v.y &&
    z == v.z;
}

bool Vector3Int::isEqual( i32 x, i32 y, i32 z ) const
{
  return
    x == this->x &&
    y == this->y &&
    z == this->z;
}

inline bool Vector3Int::isZeroVector() const
{
  return x == 0 && y == 0 && z == 0;
}

// -------------------------------------------------------------------------- operators
bool Vector3Int::operator == ( Vector3Int const & v ) const
{
  return isEqual( v );
}
bool Vector3Int::operator != ( Vector3Int const & v ) const
{
  return !isEqual( v );
}

Vector3Int & Vector3Int::operator += ( Vector3Int const & v )
{
  x += v.x;
  y += v.y;
  z += v.z;
  return *this;
}

Vector3Int & Vector3Int::operator -= ( Vector3Int const & v )
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
  return *this;
}

Vector3Int & Vector3Int::operator *= ( i32 d )
{
  x *= d;
  y *= d;
  z *= d;
  return *this;
}

Vector3Int & Vector3Int::operator /= ( i32 d )
{
  ZgAssert( d != 0 );
  x /= d;
  y /= d;
  z /= d;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Vector3Int Vector3Int::operator - () const
{
  return Vector3Int( -x, -y, -z );
}

// -------------------------------------------------------------------------- binary operators
Vector3Int Vector3Int::operator + ( Vector3Int const & v ) const
{
  return Vector3Int( 
    x + v.x,
    y + v.y,
    z + v.z
  );
}

Vector3Int Vector3Int::operator - ( Vector3Int const & v ) const
{
  return Vector3Int(
    x - v.x,
    y - v.y,
    z - v.z
  );
}

Vector3Int Vector3Int::operator * ( i32 k ) const
{
  return Vector3Int(
    x * k,
    y * k,
    z * k
  );
}

Vector3Int Vector3Int::operator / ( i32 k ) const
{
  ZgAssert( k != 0 );
  return Vector3Int(
    x / k,
    y / k,
    z / k
  );
}

i32 Vector3Int::operator * ( Vector3Int const & v ) const
{
  return x * v.x 
       + y * v.y
       + z * v.z;
}

} // namespace zygo
