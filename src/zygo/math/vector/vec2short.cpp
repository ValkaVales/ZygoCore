#include "vec2short.h"
#include <zygo/core/assert.h>


namespace zygo {

Vector2Short::Vector2Short()
  : x ( 0 )
  , y ( 0 )
{
}

Vector2Short::Vector2Short( i16 x, i16 y )
  : x ( x )
  , y ( y )
{
}

void Vector2Short::set( i16 x, i16 y )
{
  this->x = x;
  this->y = y;
}

bool Vector2Short::isEqual( Vector2Short const & v ) const
{
  return
    x == v.x &&
    y == v.y;
}

bool Vector2Short::isEqual( i16 x, i16 y ) const
{
  return
    x == this->x &&
    y == this->y;
}

inline bool Vector2Short::isZeroVector() const
{
  return x == 0 && y == 0;
}

bool Vector2Short::isLessByXY( Vector2Short const& v ) const
{
  return x < v.x
    || (x == v.x && y < v.y);
}

// -------------------------------------------------------------------------- operators
bool Vector2Short::operator == ( Vector2Short const & v ) const
{
  return isEqual( v );
}
bool Vector2Short::operator != ( Vector2Short const & v ) const
{
  return !isEqual( v );
}

Vector2Short & Vector2Short::operator += ( Vector2Short const & v )
{
  x += v.x;
  y += v.y;
  return *this;
}

Vector2Short & Vector2Short::operator -= ( Vector2Short const & v )
{
  x -= v.x;
  y -= v.y;
  return *this;
}

Vector2Short & Vector2Short::operator *= ( i16 d )
{
  x *= d;
  y *= d;
  return *this;
}

Vector2Short & Vector2Short::operator /= ( i16 d )
{
  ZgAssert( d != 0 );
  x /= d;
  y /= d;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Vector2Short Vector2Short::operator - () const
{
  return Vector2Short( -x, -y );
}

// -------------------------------------------------------------------------- binary operators
Vector2Short Vector2Short::operator + ( Vector2Short const & v ) const
{
  return Vector2Short( 
    x + v.x,
    y + v.y
  );
}

Vector2Short Vector2Short::operator - ( Vector2Short const & v ) const
{
  return Vector2Short(
    x - v.x,
    y - v.y
  );
}

Vector2Short Vector2Short::operator * ( i16 k ) const
{
  return Vector2Short(
    x * k,
    y * k
  );
}

Vector2Short Vector2Short::operator / ( i16 k ) const
{
  ZgAssert( k != 0 );
  return Vector2Short(
    x / k,
    y / k
  );
}

i16 Vector2Short::operator * ( Vector2Short const & v ) const
{
  return x * v.x 
       + y * v.y;
}

i16 Vector2Short::operator ^ ( Vector2Short const & v ) const
{
  return x * v.y
       - y * v.x;
}

} // namespace zygo
