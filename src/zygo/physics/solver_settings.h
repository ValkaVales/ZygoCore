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


// ------------------------------------------------------------------ the aggregate
struct SolverSettings
{
  GravitySettings gravity;
  StepSettings    step;
  JointSettings   joints;
  MotorSettings   motor;
  LimitSettings   limits;
  ContactSettings contacts;
};

} // namespace phys
} // namespace zygo
