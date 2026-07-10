#pragma once

#include <zygo/math/common/consts.h>


namespace zygo {

class Vector3Short
{
public:
  i16 x;
  i16 y;
  union
  {
    i16 z;
    i16 t;
  };

public:
  Vector3Short();
  Vector3Short( i16 x, i16 y, i16 z );
  Vector3Short( Vector3Short const& ) = default;

  void set( i16 x, i16 y, i16 z );
  bool isEqual( Vector3Short const & v ) const;
  bool isEqual( i16 x, i16 y, i16 z ) const;

  bool xyEquals( Vector3Short const& v ) const;

  bool isZeroVector() const;

  bool isLessByXY( Vector3Short const& v ) const;
  int distManhattanXY( Vector3Short const& v ) const;

  // operators
  bool operator == ( Vector3Short const & v ) const;
  bool operator != ( Vector3Short const & v ) const;

  Vector3Short & operator += ( Vector3Short const & v );
  Vector3Short & operator -= ( Vector3Short const & v );
  Vector3Short & operator *= ( i16 d );
  Vector3Short & operator /= ( i16 d );

  // unary operator
  Vector3Short operator - () const;

  // binary operators
  Vector3Short operator + ( Vector3Short const & v ) const;
  Vector3Short operator - ( Vector3Short const & v ) const;
  Vector3Short operator * ( i16 k ) const;
  Vector3Short operator / ( i16 k ) const;

  i16 operator * ( Vector3Short const & v ) const;
};

} // namespace zygo
