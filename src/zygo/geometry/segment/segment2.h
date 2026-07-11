#pragma once

#include <zygo/math/vector/vec2.h>


namespace zygo {

class Segment2
{
private:
  Vector2 p1;
  Vector2 p2;

public:
  Segment2( Vector2 const & p1, Vector2 const & p2 );
  
  Vector2 pointFromRatio( Real t ) const;   // returns point on the segment by its coeff t
  Real    ratioFromPoint( Vector2 const & p ) const; // projection of pt on the segment (coeff)

  Vector2 nearestPt( Vector2 const & p ) const;

  Vector2 const &getP1() const { return p1; }
  Vector2 const &getP2() const { return p2; }

  Vector2 dir() const { return p2 - p1; }

  Real distTo   ( Vector2 const & p ) const;
  Real distToSqr( Vector2 const & p ) const; // distance from point to the segment

  bool contains( Vector2 const & p, Real eps = EPSILON ) const;
};

} // namespace zygo
