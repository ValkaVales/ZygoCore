#pragma once

// Contact geometry of the world: registered foot spheres and per-substep contact points.

#include <zygo/math/vector/vec3.h>


namespace zygo {
namespace phys {

class RigidBody;


// A foot sphere registered as contact geometry.
struct ContactSphere
{
  RigidBody * body = nullptr;

  Vector3 local_center; // relative to the body center of mass, body frame, meters
  double  radius = 0.0;

  // Accumulated impulses for warm-starting between frames.
  double accumulated_normal_impulse = 0.0;
  double accumulated_t1_impulse     = 0.0;
  double accumulated_t2_impulse     = 0.0;

  bool was_in_contact = false;

  Vector3 prev_normal = Vector3( 0.0, 0.0, 1.0 ); // up by default (flat horizontal terrain)
};


// An active contact of the current substep (rebuilt every substep).
struct ContactPoint
{
  ContactSphere * sphere = nullptr;

  Vector3 point;       // the contact point in the world
  Vector3 normal;      // unit, out of the ground
  Vector3 t1, t2;      // the tangent basis

  double penetration = 0.0;
  double vn0         = 0.0; // the normal approach speed at detection time

  // Inverse effective masses along n, t1, t2.
  double kn  = 0.0;
  double kt1 = 0.0;
  double kt2 = 0.0;
};

} // namespace phys
} // namespace zygo
