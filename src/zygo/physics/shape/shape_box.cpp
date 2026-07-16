#include "shape_box.h"
#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

ShapeBox::ShapeBox( double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color )
  : Shape( ShapeType::BOX, mass, local_pos, size, local_rot, color )
{
}

Mat3 ShapeBox::calcLocalInertiaTensorForPart() const
{
  // Uniform box.
  return Mat3::inertiaBox( mass, size.x, size.y, size.z );
}

void ShapeBox::draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const
{
  Vector3 world_center = calcWorldCenterOfMass( obj_world_pos, obj_world_rot );
  Quaternion world_rot = calcWorldRotation( obj_world_rot );

  drawer.box( world_center, world_rot, size.x, size.y, size.z, color );
}

} // namespace phys
} // namespace zygo
