#include "intersections.h"
#include <zygo/math/common/scalar.h>


namespace zygo {

IntersectionType Intersections::genericIntersection
(
  Segment2 const & s1,
  Segment2 const & s2,
  Vector2 * out1,
  Vector2 * out2,
  Real eps
)
{
  // check bounding boxes
  Vector2 const & p1 = s1.getP1();
  Vector2 const & p2 = s1.getP2();
  Vector2 const & p3 = s2.getP1();
  Vector2 const & p4 = s2.getP2();

  // x
  Real minx1, maxx1;
  getMinMax( p1.x, p2.x, minx1, maxx1 );

  Real minx2, maxx2;
  getMinMax( p3.x, p4.x, minx2, maxx2 );

  if ( minx1 >= maxx2 + eps || maxx1 <= minx2 - eps )
    return IntersectionType::disjoint;

  // y
  Real miny1, maxy1;
  getMinMax( p1.y, p2.y, miny1, maxy1 );

  Real miny2, maxy2;
  getMinMax( p3.y, p4.y, miny2, maxy2 );

  if ( miny1 >= maxy2 + eps || maxy1 <= miny2 - eps )
    return IntersectionType::disjoint;

  //
  return genericIntersection_helper( s1, s2, out1, out2, eps );
}

IntersectionType Intersections::genericIntersection_helper
(
  Segment2 const & s1,
  Segment2 const & s2,
  Vector2 * out1,
  Vector2 * out2,
  Real eps
)
{
  // Get segments directions (convert them to vectors)
  Vector2 v1 = s1.dir();
  Vector2 v2 = s2.dir();
  Vector2 w = s1.getP1() - s2.getP1();

  // Check segments degeneracy
  Real d1 = v1.lengthSqr();
  Real d2 = v2.lengthSqr();

  bool iszero1 = isZero( d1 ); // checking w/o eps
  bool iszero2 = isZero( d2 );

  if ( iszero1 || iszero2 )
  {
    if ( iszero1 && iszero2 ) // Both segments are points
    {
      if ( !isZero( w.length(), eps ) )
        return IntersectionType::disjoint;

      if ( out1 ) *out1 = s1.getP1();
      return IntersectionType::intersect;
    }

    if ( iszero1 ) // The first segment is a point, the second is not
    {
      if ( !s2.contains( s1.getP1(), eps ) ) // This point is NOT inside the second segment
        return IntersectionType::disjoint;

      if ( out1 ) *out1 = s1.getP1();
      return IntersectionType::intersect;
    }

    if ( iszero2 ) // The second segment is a point, the first is not
    {
      if ( !s1.contains( s2.getP1(), eps ) ) // This point is NOT inside the first segment
        return IntersectionType::disjoint;

      if ( out1 ) *out1 = s2.getP1();
      return IntersectionType::intersect;
    }
  }

  // So, these segments are NOT points

  // Check if these segments are parallel
  Real D = v1 ^ v2;

  if ( isZero( D ) ) // Parallel
  {
    if ( !isZero( v1 ^ w ) || !isZero( v2 ^ w ) ) // Not on the same line
      return IntersectionType::disjoint;

    // So, these segments are parallel and on the same line

    Real t1 = s1.ratioFromPoint( s2.getP1() );
    Real t2 = s1.ratioFromPoint( s2.getP2() );

    // make t1 <= t2
    if ( t1 >= t2 )
    {
      Real temp = t1;
      t1 = t2;
      t2 = temp;
    }

    eps /= std::sqrt( d1 );
    if ( isZero( t2, eps ) )
    {
      if ( out1 ) *out1 = s1.getP1();
      return IntersectionType::intersect;
    }

    if ( isOne( t1, eps ) )
    {
      if ( out1 ) *out1 = s1.getP2();
      return IntersectionType::intersect;
    }

    if ( t2 < REAL_ZERO || t1 > REAL_ONE ) // segments DO NOT intersect
      return IntersectionType::disjoint;

    if ( out1 ) *out1 = (t1 <= REAL_ZERO) ? s1.getP1() : s1.pointFromRatio( t1 );
    if ( out2 ) *out2 = (t2 >= REAL_ONE ) ? s1.getP2() : s1.pointFromRatio( t2 );

    return IntersectionType::overlap;
  } // Parallel segments

  // Intersection parameter for s1
  Real t1 = (v1 ^ w) / D;
  if ( !between01( t1, eps / std::sqrt( d2 ) ) )
  //if ( !between01sqr( t1, d2, eps ) )
    return IntersectionType::disjoint;

  // Intersection parameter for s2
  Real t2 = (v2 ^ w) / D;
  if ( !between01( t2, eps / std::sqrt( d1 ) ) )
  //if ( !between01sqr( t2, d1, eps ) )
    return IntersectionType::disjoint;

  if ( out1 )
  {
    if ( t1 <= REAL_ZERO ) *out1 = s1.getP1(); else
    if ( t1 >= REAL_ONE  ) *out1 = s1.getP2(); else
      *out1 = s1.pointFromRatio( t1 );
  }
  return IntersectionType::intersect;
}

bool Intersections::intersectionSegSeg( Segment2 const & s1, Segment2 const & s2, Real eps )
{
  return IntersectionType::disjoint != genericIntersection( s1, s2, NULL, NULL, eps );
}

bool Intersections::rightTurn( Vector2 const & a, Vector2 const & b, Vector2 const & c )
{
  return ((b - a) ^ (c - a)) < REAL_ZERO;
}

bool Intersections::leftTurn( Vector2 const & a, Vector2 const & b, Vector2 const & c )
{
  return ((b - a) ^ (c - a)) > REAL_ZERO;
}

bool Intersections::intersectionPointTriangle(
  Vector2 const & p,
  Vector2 const & p1,
  Vector2 const & p2,
  Vector2 const & p3,
  Real eps )
{
  Real dd = eps * eps;

  if ( Segment2( p1, p2 ).distToSqr( p ) <= dd ||
    Segment2( p2, p3 ).distToSqr( p ) <= dd ||
    Segment2( p3, p1 ).distToSqr( p ) <= dd )
  {
    return true;
  }

  // is the triangle singular?
  Real D = (p2 - p1) ^ (p3 - p1);
  if ( isZero( D ) )
    return false;

  //
  bool b1 = rightTurn( p1, p2, p );
  bool b2 = rightTurn( p2, p3, p );
  bool b3 = rightTurn( p3, p1, p );

  return (b1 && b2 && b3) ||
    (!b1 && !b2 && !b3);
}

// Is fast, but NOT right, because of intersection test absence
bool Intersections::intersectionPointQuad_fast(
  Vector2 const & p,
  Vector2 const & p1,
  Vector2 const & p2,
  Vector2 const & p3,
  Vector2 const & p4,
  Real eps )
{
  Real dd = eps * eps;

  if ( Segment2( p1, p2 ).distToSqr( p ) <= dd ||
    Segment2( p2, p3 ).distToSqr( p ) <= dd ||
    Segment2( p3, p4 ).distToSqr( p ) <= dd ||
    Segment2( p4, p1 ).distToSqr( p ) <= dd )
  {
    return true;
  }

  /*
  // is the rectangle singular?
      Real D = (p2 - p1) ^ (p4 - p2);
    if ( is_zero(D) )
    return false;
  */

  //
  bool b1 = rightTurn( p1, p2, p );
  bool b2 = rightTurn( p2, p3, p );
  bool b3 = rightTurn( p3, p4, p );
  bool b4 = rightTurn( p4, p1, p );

  return (b1 && b2 && b3 && b4) ||
    (!b1 && !b2 && !b3 && !b4);
}

bool Intersections::intersectionSegQuad(
  Segment2 const & s,
  Vector2 const & p1,
  Vector2 const & p2,
  Vector2 const & p3,
  Vector2 const & p4,
  Real eps )
{
  Segment2 s1( p1, p2 );
  Segment2 s2( p2, p3 );
  Segment2 s3( p3, p4 );
  Segment2 s4( p4, p1 );

  // Check is the quad self-intersected
  Vector2 pp;

  // s1 and s3
  IntersectionType isect_type = genericIntersection( s1, s3, &pp );
  if ( isect_type == IntersectionType::overlap )
    goto CheckSegs;

  if ( isect_type == IntersectionType::intersect )
  {
    if ( intersectionPointTriangle( s.getP1(), p1, p4, pp, eps ) ) return true;
    if ( intersectionPointTriangle( s.getP2(), p3, p2, pp, eps ) ) return true;
    goto CheckSegs;
  }

  // s2 and s4
  isect_type = genericIntersection( s2, s4, &pp );
  if ( isect_type == IntersectionType::overlap )
    goto CheckSegs;

  if ( isect_type == IntersectionType::intersect )
  {
    if ( intersectionPointTriangle( s.getP1(), p1, p2, pp, eps ) ) return true;
    if ( intersectionPointTriangle( s.getP2(), p3, p4, pp, eps ) ) return true;
    goto CheckSegs;
  }

  // Check points in quad
  if ( intersectionPointQuad_fast( s.getP1(), p1, p2, p3, p4, eps ) ) return true;
  if ( intersectionPointQuad_fast( s.getP2(), p1, p2, p3, p4, eps ) ) return true;

CheckSegs:
  // Check Seg-Seg intersection
  if ( intersectionSegSeg( s1, s, eps ) ) return true;
  if ( intersectionSegSeg( s2, s, eps ) ) return true;
  if ( intersectionSegSeg( s3, s, eps ) ) return true;
  if ( intersectionSegSeg( s4, s, eps ) ) return true;

  return false;
}

} // namespace zygo
