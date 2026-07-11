#include "quaternion.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {

// -------------------------------------------------------------------------- operators
Quaternion & Quaternion::operator += ( Quaternion const & q )
{
  s += q.s;
  v += q.v;
  return *this;
}

Quaternion & Quaternion::operator -= ( Quaternion const & q )
{
  s -= q.s;
  v -= q.v;
  return *this;
}

Quaternion & Quaternion::operator *= ( Real d )
{
  s *= d;
  v *= d;
  return *this;
}

Quaternion & Quaternion::operator /= ( Real d )
{
  ZgAssert( !isZero( d ) );
  s /= d;
  v /= d;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Quaternion Quaternion::operator - () const
{
  return Quaternion( -s, -v );
}

// -------------------------------------------------------------------------- static binary operators
Quaternion Quaternion::operator + ( Quaternion const & q ) const
{
  return Quaternion(
    s + q.s,
    v + q.v
  );
}

Quaternion Quaternion::operator - ( Quaternion const & q ) const
{
  return Quaternion(
    s - q.s,
    v - q.v
  );
}

Quaternion Quaternion::operator * ( Real d ) const
{
  return Quaternion(
    s * d,
    v * d
  );
}

Quaternion Quaternion::operator / ( Real d ) const
{
  ZgAssert( !isZero( d ) );
  return Quaternion(
    s / d,
    v / d
  );
}

// -------------------------------------------------------------------------- products
Real Quaternion::dotProduct( Quaternion const & q ) const
{
  return s * q.s + v * q.v;
}

Quaternion Quaternion::crossProduct( Quaternion const & q ) const
{
  return Quaternion(
    q.s * s - v * q.v,
    q.v * s + v * q.s + v.crossProduct( q.v )
  );
}

} // namespace zygo
