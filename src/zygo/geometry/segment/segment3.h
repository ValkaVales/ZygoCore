#pragma once

#include <zygo/math/vector/vec3.h>


namespace zygo {

class Segment3
{
private:
  Vector3 p1;
  Vector3 p2;

public:
  Segment3( Vector3 const & p1, Vector3 const & p2 );

  Vector3 pointFromRatio( Real t ) const;  // returns point on the segment by its coeff t
  Real    ratioFromPoint( Vector3 const & p ) const; // projection of pt on the segment (coeff)
  
  Vector3 nearestPt( Vector3 const & p ) const;

  Vector3 const &getP1() const { return p1; }
  Vector3 const &getP2() const { return p2; }

  Vector3 dir() const { return p2 - p1; }

  Real distTo   ( Vector3 const & p ) const;
  Real distToSqr( Vector3 const & p ) const; // distance from point to the segment
  
  bool contains( Vector3 const & p, Real eps = EPSILON ) const;
};

} // namespace zygo
