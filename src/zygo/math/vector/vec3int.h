#pragma once

#include <zygo/math/common/consts.h>


namespace zygo {

class Vector3Int
{
public:
  i32 x;
  i32 y;
  union
  {
    i32 z;
    i32 t;
  };

public:
  Vector3Int();
  Vector3Int( i32 x, i32 y, i32 z );
  Vector3Int( Vector3Int const& ) = default;

  void set( i32 x, i32 y, i32 z );
  bool isEqual( Vector3Int const & v ) const;
  bool isEqual( i32 x, i32 y, i32 z ) const;

  bool isZeroVector() const;

  // operators
  bool operator == ( Vector3Int const & v ) const;
  bool operator != ( Vector3Int const & v ) const;

  Vector3Int & operator += ( Vector3Int const & v );
  Vector3Int & operator -= ( Vector3Int const & v );
  Vector3Int & operator *= ( i32 d );
  Vector3Int & operator /= ( i32 d );

  // unary operator
  Vector3Int operator - () const;

  // binary operators
  Vector3Int operator + ( Vector3Int const & v ) const;
  Vector3Int operator - ( Vector3Int const & v ) const;
  Vector3Int operator * ( i32 k ) const;
  Vector3Int operator / ( i32 k ) const;

  i32 operator * ( Vector3Int const & v ) const;
};

} // namespace zygo
