#pragma once

// Runtime-tunable settings of the physics engine, grouped by domain.
//
// PhysicsWorld owns one SolverSettings instance (world.settings); every HingeJoint
// of that world reads it through a const pointer, so changes take effect on the
// next simulation step. All defaults reproduce the previous hard-coded constants,
// so a default-constructed SolverSettings changes nothing.
//
// True constants (unit conversions, float-precision epsilons) stay in phys_consts.h.

#include <zygo/math/vector/vec3.h>
#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

// ------------------------------------------------------------------ gravity
struct GravitySettings
{
  // Gravity is OFF by default (as before); the vector is pre-set, so switching
  // it on is one flag: world.settings.gravity.enabled = true;
  bool enabled = false;

  Vector3 g = Vector3( 0.0, 0.0, -9.81 ); // m/s^2
};


// ------------------------------------------------------------------ stepping
struct StepSettings
{
  int substeps            = 5;
  int velocity_iterations = 30; // per substep; joints and contacts share the loop
  int position_iterations = 10; // per substep, with contact re-detection
};


// ------------------------------------------------------------------ hinge joints
struct JointSettings
{
  // ---- warm starting ----
  // Re-apply last substep's accumulated anchor/axis impulse before the Gauss-Seidel
  // loop starts. A joint under a steady load carries almost the same impulse from one
  // substep to the next, so this hands the solver a nearly converged starting point
  // and the iterations only have to fix the residual.
  //
  // Turn it off to A/B against the old cold-start behaviour.
  bool warm_starting = true;

  // How much of the stored impulse is actually re-applied. 1.0 is the textbook value and
  // is right once the solver gets enough iterations to settle. Below ~8 iterations the
  // warm start lands before the neighbouring constraints have had a chance to react, and
  // damping it trades a little stiffness for a much smaller transient - the same knob
  // Bullet exposes as m_warmstartingFactor.
  double warm_start_factor = 0.85;

  // ---- velocity solver (Baumgarte bias) ----
  double position_beta = 0.37;  // Baumgarte share for the anchor position error
  double angular_beta  = 0.2;   // Baumgarte share for the axis tilt error
  double softness      = 0.0;
  double max_impulse   = 1e5;   // explosion protection

  double max_bias_speed      = 1.0; // m/s, bias speed cap
  double max_axis_bias_speed = 5.0; // rad/s

  // Deadbands: the bias is applied only when the error exceeds the slop,
  // otherwise numerical jitter would produce parasite impulses.
  double linear_slop  = 1e-5;            // 0.01 mm
  double angular_slop = DEG2RAD( 0.05 ); // ~0.05 deg

  // ---- position solver ----
  double position_solver_activation_error = 5e-4; // 0.5 mm
  double position_solver_beta             = 0.2;

  double max_position_linear_correction  = 0.001;          // <= 1 mm per correction
  double max_position_angular_correction = DEG2RAD( 5.0 );

  // ---- convergence thresholds ("still has error") ----
  // Impulses below these do not count as an error, so the solver loop can stop
  // early once everything settles. Looser = faster, tighter = more precise.
  double min_error_for_j               = 1e-4;
  double min_error_for_angular_impulse = 1e-4;
  double min_error_for_impulse         = 1e-5;
};


// ------------------------------------------------------------------ joint motors
struct MotorSettings
{
  // ERP-like coefficient for the position motor: the fraction of the angle error
  // we try to remove per simulation step. Good values for DT = 0.01: 0.03 .. 0.15.
  double position_erp = 0.07;

  double softness = 0.0;
};


// ------------------------------------------------------------------ joint angle limits
struct LimitSettings
{
  double beta        = 0.1;
  double slop        = DEG2RAD( 0.5 );
  double softness    = 1e-8;
  double max_impulse = 20.0;

  double min_error_for_impulse = 1e-4; // convergence threshold
};


// ------------------------------------------------------------------ contacts
struct ContactSettings
{
  double friction_mu = 0.75;
  double restitution = 0.0;  // feet must not bounce, hence 0

  double margin = 1e-4; // 0.1 mm: a sphere this close already counts as a contact

  // ---- speculative contacts ----
  // A fixed 0.1 mm margin is far too small for a moving foot: landing at 2 m/s with a
  // 2 ms substep it travels 4 mm per substep, so it is never seen approaching - only
  // afterwards, already 4 mm inside the ground, and Baumgarte then shoves it back out.
  //
  // With this on, the query margin grows to whatever the sphere can actually cover in
  // one substep, so the contact is found BEFORE the surface is crossed. Such a contact
  // does not stop the foot in mid-air: it only forbids it from ending the substep below
  // the surface, i.e. the allowed approach speed is exactly the one that closes the gap.
  //
  // The result is a foot that lands ON the ground instead of inside it.
  bool speculative_contacts = true;

  // Cap on the velocity-derived margin. Guards against a single explosive velocity
  // turning every foot into a ground-wide contact query.
  double speculative_margin_max = 0.05; // 50 mm

  // ---- torsional (spin) friction ----
  // A sphere touches a plane at one point, so the two tangential rows resist sliding but
  // nothing resists rotation ABOUT the contact normal - a planted foot spins freely and
  // the robot slowly yaws away with no external torque. A real foot has a finite contact
  // patch; this is the standard single-row approximation of it:
  //
  //   |spin impulse| <= spin_friction_mu * sphere_radius * normal impulse
  //
  // Dimensionless, like friction_mu; the sphere radius supplies the lever arm.
  // Set to 0 to disable.
  double spin_friction_mu = 0.35;

  // ---- velocity solver ----
  double baumgarte_beta = 0.2;  // penetration share removed per substep
  double slop           = 1e-4; // allowed penetration, ~0.1 mm
  double max_bias_speed = 1.0;  // m/s, push-out speed cap

  // Friction accumulators are reset when the contact normal turns further than this
  // (the old tangent plane no longer matches the new one).
  double normal_reset_dot = 0.98;

  // "Push into the ground" speed below this threshold produces no restitution (kills jitter).
  double restitution_velocity_threshold = 0.1; // m/s

  // ---- position solver ----
  double position_beta           = 0.2;
  double position_slop           = 1e-4;  // 0.1 mm
  double max_position_correction = 0.005; // 5 mm per iteration
  double min_position_correction = 1e-5;

  double max_position_angular_correction = DEG2RAD( 5.0 ); // rotation cap of one positional pseudo-impulse

  // ---- convergence thresholds ("still has error") ----
  double min_error_for_collision_impulse = 1e-3;
  double min_error_for_friction_impulse  = 1e-3;
};


// ------------------------------------------------------------------ sleeping
struct SleepSettings
{
  // OFF by default, and deliberately so.
  //
  // A sleeping assembly is skipped entirely - no gravity, no solve, no integration - so
  // a standing robot stops costing anything. The catch is that nothing in the engine can
  // guess when it should wake up again: the world only knows about gravity and terrain
  // contacts, so an assembly that fell asleep will stay asleep until something tells it
  // otherwise.
  //
  // Two things wake it:
  //   - a motor command on any of its joints (setMotorPosition/Velocity/Torque,
  //     disableMotor, enableAngleLimit) - handled automatically;
  //   - ArticulatedBody::wakeUp(), which you must call yourself after applying an
  //     impulse, teleporting a body, or changing the terrain under it.
  //
  // If a controller drives the assembly through anything other than the joint motors,
  // leave this off.
  bool enabled = false;

  // An assembly is a sleep candidate while EVERY one of its bodies stays under both
  // thresholds. One body above either of them resets the timer for the whole assembly.
  double linear_velocity_threshold  = 0.01; // m/s
  double angular_velocity_threshold = 0.05; // rad/s

  // How long it has to stay that quiet before it actually falls asleep.
  double time_to_sleep = 0.5; // s
};


// ------------------------------------------------------------------ the aggregate
struct SolverSettings
{
  GravitySettings gravity;
  StepSettings    step;
  JointSettings   joints;
  MotorSettings   motor;
  LimitSettings   limits;
  ContactSettings contacts;
  SleepSettings   sleep;
};

} // namespace phys
} // namespace zygo
