#include "shape_cylinder.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

ShapeCylinder::ShapeCylinder( double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color )
  : Shape( ShapeType::CYLINDER, mass, local_pos, size, local_rot, color )
{
}

Mat3 ShapeCylinder::calcLocalInertiaTensorForPart() const
{
  // size.x = height, size.y = radius.
  // RigidBody::addCylinderBySegment() builds local_rot so that the cylinder axis is the local Z axis.
  ZgAssert( mass   > BIG_EPSILON );
  ZgAssert( size.x > BIG_EPSILON );
  ZgAssert( size.y > BIG_EPSILON );

  return Mat3::inertiaCylinderZ( mass, size.y /*radius*/, size.x /*length*/ );
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
