#pragma once

// Solver tuning constants.
// All values are SI: meters, seconds, radians. See physics.h for the units convention.

#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

// Unit conversion for the construction API (grams/millimeters -> SI).
const double GRAMS_IN_KG          = 1000.0;
const double MILLIMETERS_IN_METER = 1000.0;


// ------------------------------------------------------------------ generic epsilons
const double PHYS_EPSILON     = 1e-15;
const double PHYS_EPSILON_SQR = sqr( PHYS_EPSILON );
const double MIN_ERROR        = 1e-12;


// ------------------------------------------------------------------ "still has error" thresholds
// An impulse smaller than the threshold does not count as an error,
// so the solver loop can stop early once everything settles below them.
const double MIN_ERROR_FOR_J               = 1e-4;
const double MIN_ERROR_FOR_ANGULAR_IMPULSE = 1e-4;
const double MIN_MOTOR_IMPULSE             = 1e-5;
const double MIN_ERROR_FOR_IMPULSE         = 1e-5;

const double MIN_ERROR_FOR_COLLISION_IMPULSE = 1e-3;
const double MIN_ERROR_FOR_FRICTION_IMPULSE  = 1e-3;


// ------------------------------------------------------------------ joint velocity solver
const double POSITION_BETA = 0.37;  // Baumgarte share for the anchor position error
const double ANGULAR_BETA  = 0.2;   // Baumgarte share for the axis tilt error
const double SOFTNESS      = 0.0;
const double MAX_IMPULSE   = 1e5;   // explosion protection

const double MAX_BIAS_SPEED      = 1.0; // m/s, bias speed cap
const double MAX_AXIS_BIAS_SPEED = 5.0; // rad/s

// Deadbands: the bias is applied only when the error exceeds the slop,
// otherwise numerical jitter would produce parasite impulses.
const double POSITION_LINEAR_SLOP  = 1e-5;            // 0.01 mm
const double POSITION_ANGULAR_SLOP = DEG2RAD( 0.05 ); // ~0.05 deg


// ------------------------------------------------------------------ joint position solver
const double POSITION_ACTIVATION_ERROR = 5e-4; // 0.5 mm
const double POSITION_SOLVER_BETA      = 0.2;

const double MAX_POSITION_LINEAR_CORRECTION  = 0.001;         // <= 1 mm per correction
const double MAX_POSITION_ANGULAR_CORRECTION = DEG2RAD( 5.0 );


// ------------------------------------------------------------------ contact solver
const double CONTACT_MARGIN         = 1e-4; // 0.1 mm: a sphere this close already counts as a contact
const double CONTACT_BAUMGARTE_BETA = 0.2;  // penetration share removed per substep
const double CONTACT_SLOP           = 1e-4; // allowed penetration, ~0.1 mm
const double MAX_CONTACT_BIAS_SPEED = 1.0;  // m/s, push-out speed cap

// Friction accumulators are reset when the contact normal turns further than this
// (the old tangent plane no longer matches the new one).
const double CONTACT_NORMAL_RESET_DOT = 0.98;

const double CONTACT_POSITION_BETA           = 0.2;
const double CONTACT_POSITION_SLOP           = 1e-4;  // 0.1 mm
const double MAX_CONTACT_POSITION_CORRECTION = 0.005; // 5 mm per iteration
const double MIN_CONTACT_POSITION_CORRECTION = 1e-5;

// "Push into the ground" speed below this threshold produces no restitution (kills jitter).
const double RESTITUTION_VELOCITY_THRESHOLD = 0.1; // m/s

} // namespace phys
} // namespace zygo
