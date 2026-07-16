#include "rigid_body.h"
#include <zygo/physics/phys_math.h>
#include <zygo/physics/phys_consts.h>
#include <zygo/physics/shape/shape_box.h>
#include <zygo/physics/shape/shape_capsule.h>
#include <zygo/physics/shape/shape_cylinder.h>
#include <zygo/physics/shape/shape_sphere.h>

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <memory>


namespace zygo {
namespace phys {

void RigidBody::addShape( Shape * shape )
{
  shapes.push_back( std::unique_ptr<Shape>( shape ) );
}

void RigidBody::addBoxBySegment(
  double mass_grams,
  Vector3 p1_mm,
  Vector3 p2_mm,
  double width_mm,
  double height_mm
)
{
  double  mass   = mass_grams / GRAMS_IN_KG;
  Vector3 p1     = p1_mm      / MILLIMETERS_IN_METER;
  Vector3 p2     = p2_mm      / MILLIMETERS_IN_METER;
  double  width  = width_mm   / MILLIMETERS_IN_METER;
  double  height = height_mm  / MILLIMETERS_IN_METER;

  Vector3 center = (p1 + p2) * 0.5;
  double length = (p2 - p1).length();

  ZgAssert( length > BIG_EPSILON );
  ZgAssert( width  > BIG_EPSILON );
  ZgAssert( height > BIG_EPSILON );

  Vector3 x_axis, y_axis, z_axis;
  buildSegmentBasisX( p1, p2, x_axis, y_axis, z_axis );

  Vector3 size( length, width, height );

  Quaternion q = buildQuaternionFromAxes( x_axis, y_axis, z_axis );

  addShape( new ShapeBox( mass, center, size, q, color ) );
}

void RigidBody::addCapsuleBySegment(
  double mass_grams,
  Vector3 p1_mm,
  Vector3 p2_mm,
  double diameter_mm
)
{
  double  mass   = mass_grams / GRAMS_IN_KG;
  Vector3 p1     = p1_mm      / MILLIMETERS_IN_METER;
  Vector3 p2     = p2_mm      / MILLIMETERS_IN_METER;
  double  radius = diameter_mm * 0.5 / MILLIMETERS_IN_METER;

  Vector3 center = (p1 + p2) * 0.5;
  double full_len = (p2 - p1).length();
  double cylinder_len = full_len - 2.0 * radius;

  ZgAssert( full_len > BIG_EPSILON );
  ZgAssert( radius > BIG_EPSILON );
  ZgAssert( cylinder_len > BIG_EPSILON );

  // The capsule axis is the local Z, like the cylinder.
  Vector3 x_axis, y_axis, z_axis;
  buildSegmentBasisZ( p1, p2, x_axis, y_axis, z_axis );

  Quaternion q = buildQuaternionFromAxes( x_axis, y_axis, z_axis );

  addShape( new ShapeCapsule( mass, center, cylinder_len, radius, q, color ) );
}

void RigidBody::addBoxByDiagonal(
  double mass_grams,
  Vector3 p1_mm,
  Vector3 p2_mm
)
{
  double  mass = mass_grams / GRAMS_IN_KG;
  Vector3 p1   = p1_mm      / MILLIMETERS_IN_METER;
  Vector3 p2   = p2_mm      / MILLIMETERS_IN_METER;

  Vector3 center = (p1 + p2) * 0.5;
  double length = std::abs( p2.x - p1.x );
  double width  = std::abs( p2.y - p1.y );
  double height = std::abs( p2.z - p1.z );

  ZgAssert( length > BIG_EPSILON );
  ZgAssert( width  > BIG_EPSILON );
  ZgAssert( height > BIG_EPSILON );

  Vector3 size( length, width, height );

  Quaternion q; // axis-aligned

  addShape( new ShapeBox( mass, center, size, q, color ) );
}

void RigidBody::addVerticalPlate(
  double mass_grams,
  Vector3 p1_mm,
  Vector3 p2_mm,
  double width_mm
)
{
  double  mass  = mass_grams / GRAMS_IN_KG;
  Vector3 p1    = p1_mm      / MILLIMETERS_IN_METER;
  Vector3 p2    = p2_mm      / MILLIMETERS_IN_METER;
  double  width = width_mm   / MILLIMETERS_IN_METER;

  ZgAssert( width > BIG_EPSILON );

  Vector3 a = p1;
  Vector3 b = p2;

  if ( a.z > b.z )
    std::swap( a, b );

  Vector3 d = b - a;

  double height = d.z;

  Vector3 horiz_v( d.x, d.y, 0.0 );
  double horiz_len = horiz_v.length();

  // For a vertical plate the face diagonal must have both a vertical
  // and a horizontal component.
  ZgAssert( height    > BIG_EPSILON );
  ZgAssert( horiz_len > BIG_EPSILON );

  // Local axes of the box:
  // ex - along the horizontal side of the plate;
  // ey - up;
  // ez - along the plate thickness.
  Vector3 ex = horiz_v / horiz_len;
  Vector3 ez( 0.0, 0.0, 1.0 );
  Vector3 ey = ez.crossProduct( ex );

  ey = Vector3::safeNormalized( ey );

  // The face diagonal = the horizontal component + the vertical component,
  // so the rectangle sides are simply:
  Vector3 size( horiz_len, width, height );

  Vector3 center = (a + b) * 0.5;

  Quaternion q = buildQuaternionFromAxes( ex, ey, ez );

  addShape( new ShapeBox( mass, center, size, q, color ) );
}

void RigidBody::addCylinderBySegment(
  double mass_grams,
  Vector3 p1_mm,
  Vector3 p2_mm,
  double diameter_mm
)
{
  double  mass     = mass_grams  / GRAMS_IN_KG;
  Vector3 p1       = p1_mm       / MILLIMETERS_IN_METER;
  Vector3 p2       = p2_mm       / MILLIMETERS_IN_METER;
  double  diameter = diameter_mm / MILLIMETERS_IN_METER;

  Vector3 axis = p2 - p1;
  double height = axis.length();

  ZgAssert( height > BIG_EPSILON );
  ZgAssert( diameter > BIG_EPSILON );

  Vector3 ex, ey, ez;
  buildSegmentBasisZ( p1, p2, ex, ey, ez );

  Quaternion q = buildQuaternionFromAxes( ex, ey, ez );
  Vector3 center = (p1 + p2) * 0.5;

  Vector3 size( height, diameter * 0.5, 0.0 );
  addShape( new ShapeCylinder( mass, center, size, q, color ) );
}

void RigidBody::addSphere(
  double mass_grams,
  Vector3 center_mm,
  double diameter_mm
)
{
  double  mass     = mass_grams  / GRAMS_IN_KG;
  Vector3 center   = center_mm   / MILLIMETERS_IN_METER;
  double  diameter = diameter_mm / MILLIMETERS_IN_METER;

  Vector3 size( diameter * 0.5, 0.0, 0.0 );

  addShape( new ShapeSphere(
    mass,
    center,
    size,
    Quaternion(), // orientation does not matter for a sphere
    color
  ) );
}

} // namespace phys
} // namespace zygo
