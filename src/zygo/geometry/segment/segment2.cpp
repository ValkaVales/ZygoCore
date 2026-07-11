#include "segment2.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <cmath> // sqrt


namespace zygo {

Segment2::Segment2( Vector2 const & p1, Vector2 const & p2 )
  : p1 ( p1 )
  , p2 ( p2 )
{
}

// returns point on the segment by its coeff t
Vector2 Segment2::pointFromRatio( Real t ) const
{
  return p1 * (REAL_ONE - t) + p2 * t;
}

// projection of pt on the segment (coeff)
Real Segment2::ratioFromPoint( Vector2 const & p ) const
{
  ZgAssert( !p1.isEqual( p2 ) );

  Vector2 v = p2 - p1;
  Vector2 w = p - p1;

  return (w * v) / (v * v);
}

Vector2 Segment2::nearestPt( Vector2 const & p ) const
{
  Real t = ratioFromPoint( p );
  to01range( t );
  return pointFromRatio( t );
}

// расстояние от точки до отрезка
Real Segment2::distToSqr( Vector2 const & p ) const
{
  if ( p1.isEqual( p2 ) )
    return p1.distToSqr( p );

  Real t = ratioFromPoint( p );

  Vector2 pi = (t < 0.)
    ? p1
    : (t > 1.) ? p2 : pointFromRatio( t );

  return pi.distToSqr( p );
}

Real Segment2::distTo( Vector2 const & p ) const
{
  return std::sqrt( distToSqr( p ) );
}

bool Segment2::contains( Vector2 const & p, Real eps ) const
{
  return isZero( distToSqr( p ), eps * eps );
}

} // namespace zygo
