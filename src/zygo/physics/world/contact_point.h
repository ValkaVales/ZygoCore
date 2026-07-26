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
  double accumulated_spin_impulse   = 0.0; // angular, about the contact normal

  // Stored in WORLD space so a rebuilt (t1,t2) basis cannot reinterpret two old scalar components as a different friction impulse.
  // It is projected onto the current tangent plane whenever the contact normal changes.
  Vector3 accumulated_tangent_impulse;

  bool was_in_contact = false;

  // The normal approach speed sampled on the substep the contact FIRST appeared, before any response.
  // Restitution uses this rather than the current speed:
  // with speculative contacts the sphere is braked as it approaches, so by the time it actually touches the instantaneous speed is no longer the speed it arrived with.
  double impact_vn = 0.0;

  Vector3 prev_normal = Vector3( 0.0, 0.0, 1.0 ); // up by default (flat horizontal terrain)
};


// An active contact of the current substep (rebuilt every substep).
struct ContactPoint
{
  ContactSphere * sphere = nullptr;

  Vector3 point;       // the contact point in the world
  Vector3 normal;      // unit, out of the ground
  Vector3 t1, t2;      // the tangent basis

  double penetration = 0.0; // > 0 => overlapping; clamped at 0
  double separation  = 0.0; // signed gap; > 0 => a speculative contact, not touching yet
  double vn0         = 0.0; // the normal approach speed at detection time

  // Inverse effective masses along n, t1, t2, and about n (torsional).
  double kn  = 0.0;
  double kt1 = 0.0;
  double kt2 = 0.0;
  double k_spin = 0.0;
};

} // namespace phys
} // namespace zygo
