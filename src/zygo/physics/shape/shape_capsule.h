#pragma once

#include <zygo/physics/shape/shape.h>


namespace zygo {
namespace phys {

class ShapeCapsule : public Shape
{
public:
  ShapeCapsule( double mass, Vector3 const & local_pos, double cylinder_len, double radius, Quaternion const & local_rot, uint color = DEFAULT_SHAPE_COLOR );

  Matrix calcLocalInertiaTensorForPart() const override;

  void draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const override;
};

} // namespace phys
} // namespace zygo
