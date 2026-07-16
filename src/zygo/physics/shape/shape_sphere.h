#pragma once

#include <zygo/physics/shape/shape.h>


namespace zygo {
namespace phys {

class ShapeSphere : public Shape
{
public:
  ShapeSphere( double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color = DEFAULT_SHAPE_COLOR );

  Mat3 calcLocalInertiaTensorForPart() const override;

  void draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const override;
};

} // namespace phys
} // namespace zygo
