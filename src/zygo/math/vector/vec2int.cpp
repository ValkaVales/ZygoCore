#include "vec2int.h"
#include <zygo/core/assert.h>


namespace zygo {

Vector2Int::Vector2Int()
  : x ( 0 )
  , y ( 0 )
{
}

Vector2Int::Vector2Int( i32 x, i32 y )
  : x ( x )
  , y ( y )
{
}

void Vector2Int::set( i32 x, i32 y )
{
  this->x = x;
  this->y = y;
}

bool Vector2Int::isEqual( Vector2Int const & v ) const
{
  return
    x == v.x &&
    y == v.y;
}

bool Vector2Int::isEqual( i32 x, i32 y ) const
{
  return
    x == this->x &&
    y == this->y;
}

bool Vector2Int::isZeroVector() const
{
  return x == 0 && y == 0;
}

// -------------------------------------------------------------------------- operators
bool Vector2Int::operator == ( Vector2Int const & v ) const
{
  return isEqual( v );
}
bool Vector2Int::operator != ( Vector2Int const & v ) const
{
  return !isEqual( v );
}

Vector2Int & Vector2Int::operator += ( Vector2Int const & v )
{
  x += v.x;
  y += v.y;
  return *this;
}

Vector2Int & Vector2Int::operator -= ( Vector2Int const & v )
{
  x -= v.x;
  y -= v.y;
  return *this;
}

Vector2Int & Vector2Int::operator *= ( i32 d )
{
  x *= d;
  y *= d;
  return *this;
}

Vector2Int & Vector2Int::operator /= ( i32 d )
{
  ZgAssert( d != 0 );
  x /= d;
  y /= d;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Vector2Int Vector2Int::operator - () const
{
  return Vector2Int( -x, -y );
}

// -------------------------------------------------------------------------- binary operators
Vector2Int Vector2Int::operator + ( Vector2Int const & v ) const
{
  return Vector2Int( 
    x + v.x,
    y + v.y
  );
}

Vector2Int Vector2Int::operator - ( Vector2Int const & v ) const
{
  return Vector2Int(
    x - v.x,
    y - v.y
  );
}

Vector2Int Vector2Int::operator * ( i32 k ) const
{
  return Vector2Int(
    x * k,
    y * k
  );
}

Vector2Int Vector2Int::operator / ( i32 k ) const
{
  ZgAssert( k != 0 );
  return Vector2Int(
    x / k,
    y / k
  );
}

i32 Vector2Int::operator * ( Vector2Int const & v ) const
{
  return x * v.x 
       + y * v.y;
}

i32 Vector2Int::operator ^ ( Vector2Int const & v ) const
{
  return x * v.y
       - y * v.x;
}

} // namespace zygo
