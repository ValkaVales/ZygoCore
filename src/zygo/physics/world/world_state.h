#pragma once

// A snapshot of everything the simulation needs to continue from where it was.
//
// The point of it is reinforcement learning: an episode reset has to be cheap and it has to
// be EXACT. Rebuilding the world instead is neither - it reallocates every body and shape,
// and it silently drops the solver's accumulated impulses, so the first step after a reset
// behaves differently from the same step reached by simulating.
//
// What is stored is the DYNAMIC state only:
//
//   bodies          position, orientation, linear and angular velocity, sleep flag
//   joints          the accumulated constraint impulses (the warm-start state) and the
//                   motor command currently in force
//   assemblies      sleep flag and idle timer
//   contact spheres the accumulated contact impulses and the latched impact velocity
//
// What is NOT stored is the structure: masses, shapes, inertia tensors, anchors, axes, which
// body is connected to which. Those never change after construction, so a snapshot is only
// valid for the world it came from - restoreState() checks the counts and refuses a mismatch.
//
// The contact accumulators matter more than they look. Leave them out and a restored robot
// starts its first substep with cold contacts, which takes several substeps to build the
// normal impulse back up - the foot sinks, and the "same" state produces a different
// trajectory. That is exactly the kind of irreproducibility that makes an RL run untrainable.

#include <zygo/math/vector/vec3.h>
#include <zygo/math/quaternion/quaternion.h>

#include <vector>


namespace zygo {
namespace phys {

struct RigidBodyState
{
  Vector3    center_of_mass_pos;
  Quaternion rotation_quaternion;

  Vector3 speed;
  Vector3 angular_speed;

  bool is_sleeping = false;
};


struct HingeJointState
{
  // Warm-start state: the impulses the joint applied over the last substep.
  Vector3 accumulated_anchor_impulse;
  Vector3 accumulated_axis_impulse;

  double accumulated_motor_impulse       = 0.0;
  double accumulated_lower_limit_impulse = 0.0;
  double accumulated_upper_limit_impulse = 0.0;

  double accumulated_impulse_dt = 0.0;
  double last_substep_dt        = 0.0;

  // The command currently in force.
  //
  // Stored because a snapshot has to be self-contained: a controller that only issues a
  // command when its target changes would otherwise find a restored joint still driving
  // toward whatever it was told long after the reset.
  int    motor_mode            = 0; // MotorMode, kept as int so this header stays independent
  double motor_target_velocity = 0.0;
  double motor_target_angle    = 0.0;
  double motor_target_torque   = 0.0;
  double motor_max_torque      = 0.0;
  double motor_max_velocity    = 0.0;

  bool wake_requested = false;
};


struct ArticulatedBodyState
{
  bool   is_sleeping = false;
  double idle_time   = 0.0;
};


struct ContactSphereState
{
  double  accumulated_normal_impulse = 0.0;
  double  accumulated_spin_impulse   = 0.0;
  Vector3 accumulated_tangent_impulse;

  bool    was_in_contact = false;
  double  impact_vn      = 0.0;
  Vector3 prev_normal    = Vector3( 0.0, 0.0, 1.0 );
};


// The whole snapshot. Reuse one of these across saves: the vectors keep their capacity, so
// after the first call saving is a copy of a few hundred bytes with no allocation at all.
struct WorldState
{
  std::vector<RigidBodyState>       bodies;
  std::vector<HingeJointState>      joints;
  std::vector<ArticulatedBodyState> assemblies;
  std::vector<ContactSphereState>   contact_spheres;

  bool valid = false; // set by saveState(), checked by restoreState()

  void clear()
  {
    bodies         .clear();
    joints         .clear();
    assemblies     .clear();
    contact_spheres.clear();
    valid = false;
  }
};

} // namespace phys
} // namespace zygo
