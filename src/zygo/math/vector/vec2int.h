#pragma once

#include <zygo/core/types.h>


namespace zygo {

class Vector2Int
{
public:
  i32 x;
  i32 y;

public:
  Vector2Int();
  Vector2Int( i32 x, i32 y );
  Vector2Int( Vector2Int const& ) = default;

  void set( i32 x, i32 y );
  bool isEqual( Vector2Int const & v ) const;
  bool isEqual( i32 x, i32 y ) const;

  bool isZeroVector() const;

  // operators
  bool operator == ( Vector2Int const & v ) const;
  bool operator != ( Vector2Int const & v ) const;

  Vector2Int & operator += ( Vector2Int const & v );
  Vector2Int & operator -= ( Vector2Int const & v );
  Vector2Int & operator *= ( i32 d );
  Vector2Int & operator /= ( i32 d );

  // unary operator
  Vector2Int operator - () const;

  // binary operators
  Vector2Int operator + ( Vector2Int const & v ) const;
  Vector2Int operator - ( Vector2Int const & v ) const;
  Vector2Int operator * ( i32 k ) const;
  Vector2Int operator / ( i32 k ) const;

  i32 operator * ( Vector2Int const & v ) const;
  i32 operator ^ ( Vector2Int const & v ) const;
};

} // namespace zygo
