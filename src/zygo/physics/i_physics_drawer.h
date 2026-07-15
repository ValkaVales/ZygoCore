#pragma once

// Debug-draw interface of the physics engine.
//
// The engine itself does not depend on any graphics library: bodies, joints,
// terrains and the world draw themselves through this interface.
// A ZygoGL implementation lives outside ZygoCore: zygogl/debug/phys_draw.h.
//
// Default implementations do nothing, so an implementation overrides only what it needs.

#include <zygo/core/types.h>
#include <zygo/math/vector/vec3.h>
#include <zygo/math/quaternion/quaternion.h>
#include <vector>


namespace zygo {
namespace phys {

class IPhysicsDrawer
{
public:
  virtual ~IPhysicsDrawer() {}

  virtual void line( Vector3 const & /*p1*/, Vector3 const & /*p2*/, uint /*color*/ ) const {}

  virtual void box     ( Vector3 const & /*center*/, Quaternion const & /*rot*/, double /*x_size*/, double /*y_size*/, double /*z_size*/, uint /*color*/ ) const {}
  virtual void sphere  ( Vector3 const & /*center*/, double /*radius*/, uint /*color*/ ) const {}
  virtual void cylinder( Vector3 const & /*center*/, Quaternion const & /*rot*/, double /*radius*/, double /*height*/, uint /*color*/ ) const {}
  virtual void capsule ( Vector3 const & /*center*/, Quaternion const & /*rot*/, double /*radius*/, double /*cylinder_length*/, uint /*color*/ ) const {}

  virtual void planeXY( double /*z*/, double /*x_min*/, double /*x_max*/, double /*y_min*/, double /*y_max*/, uint /*color*/ ) const {}
  virtual void gridXY ( double /*z*/, double /*x_min*/, double /*x_max*/, double /*y_min*/, double /*y_max*/, double /*step*/, uint /*color*/ ) const {}

  virtual void heightFieldXY( double /*origin_x*/, double /*origin_y*/, double /*cell*/, int /*nx*/, int /*ny*/, std::vector<double> const & /*heights*/, uint /*fill_color*/, uint /*wire_color*/ ) const {}
};

} // namespace phys
} // namespace zygo
