#include "segment3.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {

Segment3::Segment3( Vector3 const & p1, Vector3 const & p2 )
  : p1 ( p1 )
  , p2 ( p2 )
{
}

// returns point on the segment by its coeff t
Vector3 Segment3::pointFromRatio( Real t ) const
{
  return p1 * (REAL_ONE - t) + p2 * t;
}

// projection of pt on the segment (coeff)
Real Segment3::ratioFromPoint( Vector3 const & p ) const
{
  ZgAssert( !p1.isEqual( p2 ) );

  Vector3 v = p2 - p1;
  Vector3 w = p - p1;

  return (w * v) / (v * v);
}

Vector3 Segment3::nearestPt( Vector3 const & p ) const
{
  Real t = ratioFromPoint( p );
  to01range( t );
  return pointFromRatio( t );
}

// расстояние от точки до отрезка
Real Segment3::distToSqr( Vector3 const & p ) const
{
  if ( p1.isEqual( p2 ) )
    return p1.distToSqr( p );

  Real t = ratioFromPoint( p );

  Vector3 pi = (t < 0)
    ? p1
    : (t > 1) ? p2 : pointFromRatio( t );

  return pi.distToSqr( p );
}

Real Segment3::distTo( Vector3 const & p ) const
{
  return std::sqrt( distToSqr( p ) );
}

bool Segment3::contains( Vector3 const & p, Real eps ) const
{
  return isZero( distToSqr( p ), eps * eps );
}

} // namespace zygo
