#include "shape_cylinder.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

ShapeCylinder::ShapeCylinder( double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color )
  : Shape( ShapeType::CYLINDER, mass, local_pos, size, local_rot, color )
{
}

Matrix ShapeCylinder::calcLocalInertiaTensorForPart() const
{
  Matrix I( 3, 3 );
  I.makeAllZero();

  // size.x = height, size.y = radius.
  // RigidBody::addCylinderBySegment() builds local_rot so that the cylinder axis is the local Z axis.
  const double m = mass;
  const double L = size.x;
  const double r = size.y;

  ZgAssert( m > BIG_EPSILON );
  ZgAssert( L > BIG_EPSILON );
  ZgAssert( r > BIG_EPSILON );

  const double r2 = sqr( r );
  const double L2 = sqr( L );

  // Uniform solid cylinder with the axis along Z:
  // Izz - about its own longitudinal axis;
  // Ixx, Iyy - about the transverse axes through the center of mass.
  const double Ixx = m * ( 3.0 * r2 + L2 ) / 12.0;
  const double Iyy = Ixx;
  const double Izz = 0.5 * m * r2;

  I.setAt( 0, Ixx );
  I.setAt( 4, Iyy );
  I.setAt( 8, Izz );

  return I;
}

void ShapeCylinder::draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const
{
  Vector3 world_center = calcWorldCenterOfMass( obj_world_pos, obj_world_rot );
  Quaternion world_rot = calcWorldRotation( obj_world_rot );

  // size.x = height, size.y = radius
  drawer.cylinder( world_center, world_rot, size.y, size.x, color );
}

} // namespace phys
} // namespace zygo
