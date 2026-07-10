#pragma once

#include <zygo/math/common/consts.h>


namespace zygo {

class Vector4
{
public:
  Real x;
  Real y;
  Real z;
  Real t;

public:
  Vector4();
  Vector4( Real x, Real y, Real z, Real t );
  Vector4( Vector4 const& ) = default;

  inline Real getX() const { return x; }
  inline Real getY() const { return y; }
  inline Real getZ() const { return z; }
  inline Real getT() const { return t; }

  void reset();
  void set( Real x, Real y, Real z, Real t );
  bool isEqual( Vector4 const & v, Real eps = EPSILON ) const;
  bool isEqual( Real x, Real y, Real z, Real t, Real eps = EPSILON ) const;

  Real length   () const;
  Real lengthSqr() const;

  Real distTo   ( Vector4 const & p ) const;
  Real distToSqr( Vector4 const & p ) const;

  bool isZeroVector() const;

  void normalize();

  // operators
  Vector4 & operator += ( Vector4 const & v );
  Vector4 & operator -= ( Vector4 const & v );
  Vector4 & operator *= ( Real d );
  Vector4 & operator /= ( Real d );

  // unary operator
  Vector4 operator - () const;

  // binary operators
  Vector4 operator + ( Vector4 const & v ) const;
  Vector4 operator - ( Vector4 const & v ) const;
  Vector4 operator * ( Real d ) const;
  Vector4 operator / ( Real d ) const;
  
  Real  operator *  ( Vector4 const & v ) const;
  //Vector4 crossProduct( Vector4 const & v ) const;
};

} // namespace zygo
