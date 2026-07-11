#pragma once

#include <zygo/geometry/segment/segment2.h>


namespace zygo {

enum class IntersectionType
{
  disjoint=0,
  intersect,
  overlap
};


class Intersections
{
public:

  IntersectionType genericIntersection
  (
    Segment2 const & s1,
    Segment2 const & s2,
    Vector2 * out1 = NULL,
    Vector2 * out2 = NULL,
    Real eps = EPSILON
  );

  
private:
  static IntersectionType genericIntersection_helper
  (
    Segment2 const & s1,
    Segment2 const & s2,
    Vector2 * out1 = NULL,
    Vector2 * out2 = NULL,
    Real eps = EPSILON
  );

  
  bool intersectionSegSeg( Segment2 const & s1, Segment2 const & s2, Real eps = EPSILON );

  bool rightTurn( Vector2 const & a, Vector2 const & b, Vector2 const & c );
  bool leftTurn ( Vector2 const & a, Vector2 const & b, Vector2 const & c );

  bool intersectionPointTriangle(
    Vector2 const & p,
    Vector2 const & p1,
    Vector2 const & p2,
    Vector2 const & p3,
    Real eps = EPSILON );

  // Is fast, but NOT right, because of intersection test absence
  bool intersectionPointQuad_fast(
    Vector2 const & p,
    Vector2 const & p1,
    Vector2 const & p2,
    Vector2 const & p3,
    Vector2 const & p4,
    Real eps = EPSILON );

  bool intersectionSegQuad(
    Segment2 const & s,
    Vector2 const & p1,
    Vector2 const & p2,
    Vector2 const & p3,
    Vector2 const & p4,
    Real eps = EPSILON );

};

} // namespace zygo
