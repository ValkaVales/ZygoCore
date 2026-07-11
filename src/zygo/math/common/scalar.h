#pragma once

#include "consts.h"
#include <cmath>


namespace zygo {

template <typename T>
constexpr T min2( T a, T b ) noexcept
{
  return a < b ? a : b;
}

template <typename T>
constexpr T max2( T a, T b ) noexcept
{
  return a > b ? a : b;
}

template <typename T>
constexpr T min3( T a, T b, T c ) noexcept
{
  return min2( a, min2(b,c) );
}

template <typename T>
constexpr T max3( T a, T b, T c ) noexcept
{
  return max2( a, max2(b,c) );
}

template <typename T>
inline void getMinMax( T v1, T v2, T & min_v, T & max_v )
{
  if ( v1 < v2 )  { min_v = v1; max_v = v2; }
  else            { min_v = v2; max_v = v1; }
}


template <typename T>
inline bool eq( T a, T b, T e = EPSILON )
{
  return std::fabs( a - b ) <= e;
}

template <typename T>
inline bool isZero( T a, T e = EPSILON )
{
  return std::fabs( a ) <= e;
}

template <typename T>
inline bool isOne( T a, T e = EPSILON )
{
  return std::fabs( a - REAL_ONE ) <= e;
}

template <typename T>
inline bool ge( T a, T b, T e = EPSILON )
{
  return a >= b - e;
}

template <typename T>
inline bool le( T a, T b, T e = EPSILON )
{
  return a <= b + e;
}

template <typename T>
inline bool isGreater( T a, T b, T e = EPSILON )
{
  return a > b + e;
}

template <typename T>
inline bool isLess( T a, T b, T e = EPSILON )
{
  return a < b - e;
}

template <typename T>
inline bool inRange( T a, T b, T c, T e = EPSILON )
{
  return ge( a, b ) && le( a, c );
}

template <typename T>
inline T sqr( T x )
{
  return x * x;
}


template <typename T>
inline bool between01( T a, T e = EPSILON )
{
  return a >= -e && a <= 1.0 + e;
}

template <typename T>
inline bool between01sqr( T a, T norm_sqr, T e )
{
  e = sqr( e ) / norm_sqr;
  
  if ( a < 0.0 )
    return sqr( a ) <= e;

  if ( a > 1.0 )
    return sqr( a - 1.0 ) <= e;

  return true;
}

template <typename T>
inline bool between0A( T a, T A, T e = EPSILON )
{
  return inRange( a, T(0), A, e );
}

template <typename T>
inline void updMax( T & max_val, T val )
{
  if ( max_val < val )
    max_val = val;
}

template <typename T>
inline void applyMin( T & val, T min_val ) // the same as updMax
{
  if ( val < min_val )
    val = min_val;
}

template <typename T>
inline void updMin( T & min_val, T val )
{
  if ( min_val > val )
    min_val = val;
}

template <typename T>
inline void applyMax( T & val, T max_val ) // the same as updMin
{
  if ( val > max_val )
    val = max_val;
}

template <typename T>
inline void toRange( T & val, T min_val, T max_val )
{
  if ( val < min_val )
    val = min_val;
  else
  if ( val > max_val )
    val = max_val;
}

inline void to01range( float  & val )   { toRange( val, 0.0f, 1.0f ); }
inline void to01range( double & val )   { toRange( val, 0.0 , 1.0  ); }


template <typename T>
inline T remapNumber( T v, T v_min, T v_max, T out_min, T out_max )
{
  Real ratio = ((Real)v - v_min) / ((Real)v_max - v_min);
  return out_min + (T)(ratio * (out_max - out_min));
}

template <class T>
inline T lerpNumber( T a, T b, T t ) // t = 0.0 ... 1.0
{
  return a + t * (b - a);
}


Real fractionalPart       ( Real x );
Real fractionalPartSigned ( Real x );

} // namespace zygo
