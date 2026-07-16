#include "shape_sphere.h"


namespace zygo {
namespace phys {

ShapeSphere::ShapeSphere( double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color )
  : Shape( ShapeType::SPHERE, mass, local_pos, size, local_rot, color )
{
}

Mat3 ShapeSphere::calcLocalInertiaTensorForPart() const
{
  // Uniform solid sphere: I = 2/5 * m * r^2 about any axis.
  return Mat3::inertiaSphere( mass, size.x );
}

void ShapeSphere::draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const
{
  // Orientation does not matter for a sphere.
  Vector3 world_center = calcWorldCenterOfMass( obj_world_pos, obj_world_rot );
  drawer.sphere( world_center, size.x, color );
}

} // namespace phys
} // namespace zygo
