#include "vec3.h"
#include <zygo/math/common/scalar.h>
#include <zygo/core/assert.h>


namespace zygo {

Vector3::Vector3()
  : x ( REAL_ZERO )
  , y ( REAL_ZERO )
  , z ( REAL_ZERO )
{
}

Vector3::Vector3( Real x, Real y, Real z )
  : x ( x )
  , y ( y )
  , z ( z )
{
}

void Vector3::reset()
{
  x = REAL_ZERO;
  y = REAL_ZERO;
  z = REAL_ZERO;
}

#ifdef _DEBUG
Real Vector3::getElem( int idx ) const
{
  ZgAssert( idx >= 0 && idx < 3 );
  return (&x)[idx];
}

void Vector3::setElem( int idx, Real value )
{
  ZgAssert( idx >= 0 && idx < 3 );
  (&x)[idx] = value;
}

void Vector3::set( Real x, Real y, Real z )
{
  this->x = x;
  this->y = y;
  this->z = z;
}
#endif

bool Vector3::isEqual( Vector3 const & v, Real eps ) const
{
  return
    eq( x, v.x, eps ) &&
    eq( y, v.y, eps ) &&
    eq( z, v.z, eps );
}

bool Vector3::isEqual( Real x, Real y, Real z, Real eps ) const
{
  return
    eq( x, this->x, eps ) &&
    eq( y, this->y, eps ) &&
    eq( z, this->z, eps );
}

Real Vector3::length() const
{
  return std::sqrt( lengthSqr() );
}

Real Vector3::lengthSqr() const
{
  return sqr( x ) + sqr( y ) + sqr( z );
}

Real Vector3::distTo( Vector3 const & v ) const
{
  return std::sqrt( distToSqr( v ) );
}

Real Vector3::distToSqr( Vector3 const & v ) const
{
  return 
      sqr( v.x - x ) 
    + sqr( v.y - y ) 
    + sqr( v.z - z )
    ;
}

Real Vector3::cosPhiBetween( Vector3 const & v ) const
{
  Real len1 =   length();
  Real len2 = v.length();

  ZgAssert( !isZero( len1 ) && !isZero( len2 ) );

  Real dot_product = (*this) * v;
  return dot_product / (len1 * len2);
}

bool Vector3::isZeroVector( Real e ) const
{
  return isZero( x, e ) && isZero( y, e ) && isZero( z, e );
}

void Vector3::normalize()
{
  Real len = length();
  ZgAssert( !isZero( len ) );
  /*if ( isZero(len) )
  {
    x = REAL_ONE;
    y = REAL_ZERO;
    z = REAL_ZERO;
    return;
  }*/

  x /= len;
  y /= len;
  z /= len;
}

bool Vector3::isNormalized() const
{
  return isOne( lengthSqr(), BIG_EPSILON );
}

Vector3 Vector3::safeNormalized( Vector3 const& v, Real eps )
{
  Real len = v.length();
  if ( len <= eps )
    return Vector3{ 0.0, 0.0, 0.0 };

  return v / len;
}

Real Vector3::limitLength( Real max_len )
{
  Real len = length();
  if ( len > max_len && len > EPSILON )
  {
    Real coef = max_len / len;
    x *= coef;
    y *= coef;
    z *= coef;
  }
  return len;
}

// -------------------------------------------------------------------------- operators
Vector3 & Vector3::operator += ( Vector3 const & v )
{
  x += v.x;
  y += v.y;
  z += v.z;
  return *this;
}

Vector3 & Vector3::operator -= ( Vector3 const & v )
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
  return *this;
}

Vector3 & Vector3::operator *= ( Real d )
{
  x *= d;
  y *= d;
  z *= d;
  return *this;
}

Vector3 & Vector3::operator /= ( Real d )
{
  ZgAssert( !isZero( d ) );
  x /= d;
  y /= d;
  z /= d;
  return *this;
}

// -------------------------------------------------------------------------- unary operator
Vector3 Vector3::operator - () const
{
  return Vector3( -x, -y, -z );
}

// -------------------------------------------------------------------------- binary operators
Vector3 Vector3::operator + ( Vector3 const & v ) const
{
  return Vector3(
    x + v.x,
    y + v.y,
    z + v.z
  );
}

Vector3 Vector3::operator - ( Vector3 const & v ) const
{
  return Vector3(
    x - v.x,
    y - v.y,
    z - v.z
  );
}

Vector3 Vector3::operator * ( Real d ) const
{
  return Vector3(
    x * d,
    y * d,
    z * d
  );
}

Vector3 Vector3::operator / ( Real d ) const
{
  ZgAssert( !isZero( d ) );
  return Vector3(
    x / d,
    y / d,
    z / d
  );
}

Real Vector3::operator * ( Vector3 const & v ) const
{
  return x * v.x
       + y * v.y
       + z * v.z;
}

Vector3 Vector3::crossProduct( Vector3 const & v ) const
{
  return Vector3(
    y * v.z - z * v.y,
    z * v.x - x * v.z,
    x * v.y - y * v.x
  );
}

Vector3 Vector3::projection( Vector3 const& v ) const // calculated projection of vector V on 'this' vector
{
  Real dot_product = (*this) * v;
  Real len_sqr = lengthSqr();
  ZgAssert( !isZero( len_sqr, SMALL_EPSILON ) );

  return (*this) * (dot_product / len_sqr);
}

Vector3 Vector3::projectionOntoPlane( Vector3 const& plane_normal ) const
{
  ZgAssert( isOne( plane_normal.lengthSqr() ) );

  Real dot_product = (*this) * plane_normal;
  return (*this) - plane_normal * dot_product;
}

bool Vector3::calcNormalAndCollinearAndRotationVectors( Vector3 const& v, Vector3& normal, Vector3& collinear, Vector3& rot ) const // calculated projection of vector V on 'this' vector, and the perpendicular (normal) vector, and rotation vector
{
  ZgAssert( !isZeroVector() );
  ZgAssert( !v.isZeroVector() );

  // Calc normal
  normal = crossProduct( v );
  Real normal_len = normal.length();
  Real v_len = v.length();

  if ( isZero( normal_len ) || isZero( normal_len / v_len ) ) // vectors 'this' and 'v' are collinear
  {
    collinear = v;
    rot.reset();
    return true; // are collinear
  }

  normal /= normal_len; // normalization

  // Calc collinear vector (projection)
  collinear = projection( v );

  Real collinear_len_sqr = collinear.lengthSqr();
  Real v_len_sqr = sqr( v_len );
  Real lens_delta = v_len_sqr - collinear_len_sqr;
  if ( le( lens_delta, REAL_ZERO, SMALL_EPSILON ) )
  {
    collinear = v;
    rot.reset();
    return true; // are collinear
  }

  // Calc rotation vector
#if 1
  rot = v - collinear;
#else
  rot = crossProduct( normal ); // rotation vector
  rot.normalize();
  rot_length = sqrt( lens_delta );
  rot *= -rot_length;
#endif
  return false; // not collinear
}

void Vector3::buildOrthonormalBasisFromAxis( Vector3& b1, Vector3& b2 ) const
{
  Vector3 temp;
  if ( std::fabs( x ) < (Real)0.7 ) temp = Vector3{ REAL_ONE, REAL_ZERO, REAL_ZERO };
  else                              temp = Vector3{ REAL_ZERO, REAL_ONE, REAL_ZERO };

  b1 = safeNormalized( crossProduct( temp ) );
  b2 = safeNormalized( crossProduct( b1 ) );
}

Vector3 Vector3::lerpVectors( Vector3 const& a, Vector3 const& b, Real t )
{
  return Vector3(
    a.x + (b.x - a.x) * t,
    a.y + (b.y - a.y) * t,
    a.z + (b.z - a.z) * t
  );
}

} // namespace zygo
