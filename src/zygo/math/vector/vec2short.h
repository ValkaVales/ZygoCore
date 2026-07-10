#pragma once

#include <zygo/core/types.h>


namespace zygo {

class Vector2Short
{
public:
  i16 x;
  i16 y;

public:
  Vector2Short();
  Vector2Short( i16 x, i16 y );
  Vector2Short( Vector2Short const& ) = default;

  void set( i16 x, i16 y );
  bool isEqual( Vector2Short const & v ) const;
  bool isEqual( i16 x, i16 y ) const;

  bool isZeroVector() const;

  bool isLessByXY( Vector2Short const& v ) const;

  // operators
  bool operator == ( Vector2Short const & v ) const;
  bool operator != ( Vector2Short const & v ) const;

  Vector2Short & operator += ( Vector2Short const & v );
  Vector2Short & operator -= ( Vector2Short const & v );
  Vector2Short & operator *= ( i16 d );
  Vector2Short & operator /= ( i16 d );

  // unary operator
  Vector2Short operator - () const;

  // binary operators
  Vector2Short operator + ( Vector2Short const & v ) const;
  Vector2Short operator - ( Vector2Short const & v ) const;
  Vector2Short operator * ( i16 k ) const;
  Vector2Short operator / ( i16 k ) const;

  i16 operator * ( Vector2Short const & v ) const;
  i16 operator ^ ( Vector2Short const & v ) const;
};

} // namespace zygo
