#pragma once

#include <zygo/math/common/consts.h> // EPSILON


namespace zygo {

class Vector2
{
public:
  Real x;
  Real y;

public:
  Vector2();
  Vector2( Real x, Real y );
  Vector2( Vector2 const& ) = default;

  inline Real getX() const { return x; }
  inline Real getY() const { return y; }

  void set( Real x, Real y );
  bool isEqual( Vector2 const & v, Real eps = EPSILON ) const;
  bool isEqual( Real x, Real y, Real eps = EPSILON ) const;

  Real length   () const;
  Real lengthSqr() const;

  Real distTo   ( Vector2 const & p ) const;
  Real distToSqr( Vector2 const & p ) const;

  bool isZeroVector() const;

  Vector2 getNormal() const;
  void normalize();

  // rotation
  void rotate( Real alpha );
  void rotateSmallAngleUsingSin( Real sin_alpha ); // Works only when cos(alpha) >= 0.
  void rotate( Real sin_alpha, Real cos_alpha );

  // operators
  Vector2 & operator += ( Vector2 const & v );
  Vector2 & operator -= ( Vector2 const & v );
  Vector2 & operator *= ( Real d );
  Vector2 & operator /= ( Real d );

  // unary operator
  Vector2 operator - () const;

  // binary operators
  Vector2 operator + ( Vector2 const & v ) const;
  Vector2 operator - ( Vector2 const & v ) const;
  Vector2 operator * ( Real k ) const;
  Vector2 operator / ( Real k ) const;

  Real operator * ( Vector2 const & v ) const;
  Real operator ^ ( Vector2 const & v ) const;

  //
  static Vector2 lerpVectors( Vector2 const& a, Vector2 const& b, Real t );
};

} // namespace zygo
