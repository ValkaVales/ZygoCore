#pragma once

#include <zygo/math/common/consts.h> // EPSILON


namespace zygo {

class Vector3
{
public:
  Real x;
  Real y;
  Real z;

public:
  Vector3();
  Vector3( Real x, Real y, Real z );
  Vector3( Vector3 const& ) = default;

  inline Real getX() const { return x; }
  inline Real getY() const { return y; }
  inline Real getZ() const { return z; }

  inline void setX( Real x ) { this->x = x; }
  inline void setY( Real y ) { this->y = y; }
  inline void setZ( Real z ) { this->z = z; }

#ifdef _DEBUG
  Real getElem( int idx ) const;
  void   setElem( int idx, Real value );
  void   set( Real x, Real y, Real z );
#else
  inline Real getElem( int idx ) const { return (&x)[idx]; }
  inline void   setElem( int idx, Real value ) { (&x)[idx] = value; }
  inline void   set( Real x, Real y, Real z ) { this->x = x;  this->y = y;  this->z = z; }
#endif

  void reset();
  bool isEqual( Vector3 const & v, Real eps = EPSILON ) const;
  bool isEqual( Real x, Real y, Real z, Real eps = EPSILON ) const;

  Real length   () const;
  Real lengthSqr() const;

  Real distTo       ( Vector3 const & v ) const;
  Real distToSqr    ( Vector3 const & v ) const;
  Real cosPhiBetween( Vector3 const & v ) const;

  bool isZeroVector( Real eps = EPSILON ) const;

  void normalize();
  bool isNormalized() const;
  static Vector3 safeNormalized( Vector3 const& v, Real eps = EPSILON );
  
  Real limitLength( Real max_len ); // returns old (unclamped) length

  // pseudo operators
  inline void addX( Real d ) { x += d; }
  inline void addY( Real d ) { y += d; }
  inline void addZ( Real d ) { z += d; }

  inline void subX( Real d ) { x -= d; }
  inline void subY( Real d ) { y -= d; }
  inline void subZ( Real d ) { z -= d; }

  inline void addX_rot( Vector3 const & v ) { y += v.z; z -= v.y; }
  inline void addY_rot( Vector3 const & v ) { x -= v.z; z += v.x; }
  inline void addZ_rot( Vector3 const & v ) { x += v.y; y -= v.x; }

  inline void subX_rot( Vector3 const & v ) { y -= v.z; z += v.y; }
  inline void subY_rot( Vector3 const & v ) { x += v.z; z -= v.x; }
  inline void subZ_rot( Vector3 const & v ) { x -= v.y; y += v.x; }

  // operators
  Vector3 & operator += ( Vector3 const & v );
  Vector3 & operator -= ( Vector3 const & v );
  Vector3 & operator *= ( Real d );
  Vector3 & operator /= ( Real d );

  // unary operator
  Vector3 operator - () const;

  // binary operators
  Vector3 operator + ( Vector3 const & v ) const;
  Vector3 operator - ( Vector3 const & v ) const;
  Vector3 operator * ( Real d ) const;
  Vector3 operator / ( Real d ) const;
  
  Real    operator *  ( Vector3 const & v ) const; // dot product
  Vector3 crossProduct( Vector3 const & v ) const;

  /// Calculates projection of vector V on 'this' vector.
  Vector3 projection         ( Vector3 const& v ) const;
  Vector3 projectionOntoPlane( Vector3 const& plane_normal ) const;

  /// Calculates projection of vector V on 'this' vector, and the perpendicular (normal) vector, and rotation vector
  /// so:
  ///   v = collinear + rot
  /// and
  ///   collinear is parallel to this.
  /// 
  /// Returns: true if vectors 'this' and 'v' are collinear.
  bool calcNormalAndCollinearAndRotationVectors( Vector3 const& v, Vector3& normal, Vector3& collinear, Vector3& rot ) const;

  //
  void buildOrthonormalBasisFromAxis( Vector3& b1, Vector3& b2 ) const;

  //
  static Vector3 lerpVectors( Vector3 const& a, Vector3 const& b, Real t );
};

} // namespace zygo
