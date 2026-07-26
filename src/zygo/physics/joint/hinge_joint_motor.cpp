#include "hinge_joint.h"
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/phys_consts.h>

#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

void HingeJoint::setMotorMode( MotorMode mode )
{
  if ( motor_mode != mode )
  {
    motor_mode = mode;
    accumulated_motor_impulse = 0;
  }
}

void HingeJoint::setMotorVelocity( double target_velocity_rad, double max_torque )
{
  requestWake();

  setMotorMode( MOTOR_VELOCITY );

  ZgAssert( std::isfinite( max_torque ) );
  ZgAssert( max_torque > 0.0 );

  motor_target_velocity = target_velocity_rad;
  motor_max_torque      = max_torque;
}

void HingeJoint::setMotorPosition( double target_angle_rad, double max_torque, double max_velocity_rad )
{
  requestWake();

  setMotorMode( MOTOR_POSITION );

  ZgAssert( std::isfinite( max_torque ) );
  ZgAssert( max_torque > 0.0 );

  motor_target_angle  = target_angle_rad;
  motor_max_torque    = max_torque;
  motor_max_velocity  = std::abs( max_velocity_rad );
}

// Idea of this mode:   (see solveMotorVelocityConstraint)
// new_accumulated_impulse = target_torque * dt;
// lambda = new_accumulated_impulse - accumulated_motor_impulse;
// accumulated_motor_impulse = new_accumulated_impulse;
// applyImpulse( lambda );
void HingeJoint::setMotorTorque( double torque )
{
  requestWake();

  ZgAssert( std::isfinite( torque ) );
  ZgAssert( torque > 0.0 );

  setMotorMode( MOTOR_TORQUE );
  motor_target_torque = torque;
}

void HingeJoint::disableMotor()
{
  requestWake();

  setMotorMode( MOTOR_OFF );

  motor_target_velocity     = 0.0;
  motor_target_angle        = 0.0;
  motor_target_torque       = 0.0;
  motor_max_torque          = 0.0;
  motor_max_velocity        = 0.0;
  accumulated_motor_impulse = 0.0;
}

void HingeJoint::prepareVelocitySolve( double dt )
{
  // The motor and the limits are re-derived from scratch every substep
  // (they are one-sided / torque-capped, so a stale accumulator would be a wrong clamp, not a good guess).
  // The anchor and axis accumulators are NOT touched here - warm starting them across substeps is the whole point.
  accumulated_motor_impulse       = 0.0;
  accumulated_lower_limit_impulse = 0.0;
  accumulated_upper_limit_impulse = 0.0;

  cache_valid = false;

  if ( objA == nullptr || objB == nullptr || settings == nullptr )
    return;

  // Constants of the substep, shared by the anchor, axis, motor and limit solvers.
  cached_axis_A      = worldAxisA();
  cached_hinge_angle = currentHingeAngle();

  //ZgAssertRelease( cached_axis_A.isNormalized() ); // this works, but is commented just for performance
  ZgAssert( cached_axis_A.isNormalized() );

  prepareAnchorConstraint( dt );
  prepareAxisConstraint  ( dt );

  cache_valid = true;
}

bool HingeJoint::solveMotorVelocityConstraint( double dt )
{
  if ( motor_mode == MOTOR_OFF )
    return false;

  // Constant during the velocity loop - see prepareVelocitySolve().
  Vector3 const & axis = cached_axis_A;

  if ( motor_mode == MOTOR_TORQUE )
  {
    // A torque source is not a constraint: over the whole step it must inject exactly torque*dt of angular impulse, no matter how the solver behaves around it.
    //
    // But this function is called once per Gauss-Seidel ITERATION, so injecting the impulse directly here would multiply it by velocity_iterations.
    // Reusing the same accumulator as the other modes solves that: the total is pinned to torque*dt, the first iteration applies all of it and every later one applies a zero delta.
    // prepareVelocitySolve() clears the accumulator once per step, so the budget is fresh every time.
    double const old_impulse = accumulated_motor_impulse;

    accumulated_motor_impulse = motor_target_torque * dt;

    double const lambda = accumulated_motor_impulse - old_impulse;

    Vector3 const torque_impulse = axis * lambda;

    objA->applyAngularImpulse( -torque_impulse );
    objB->applyAngularImpulse(  torque_impulse );

    // No target velocity, so nothing to report as an error.
    return false;
  }

  double Cdot = curAngleVelocity();

  double rhs = 0.0;

  if ( motor_mode == MOTOR_VELOCITY )
  {
    // We want: Cdot == motor_target_velocity.
    rhs = motor_target_velocity - Cdot;
  } else
  if ( motor_mode == MOTOR_POSITION )
  {
    // Constraint:
    // C = currentAngle - targetAngle = 0
    double C = cached_hinge_angle - motor_target_angle;

    // Baumgarte/ERP: remove a fraction of the error per step.
    double bias = settings->motor.position_erp * C / dt;

    // The bias has the meaning of a target velocity, so cap it by the max velocity.
    toRange( bias, -motor_max_velocity, motor_max_velocity );

    // We want: Cdot + bias == 0.
    rhs = -( Cdot + bias );
  }

  double K = hingeAngularMassInv() + settings->motor.softness;
  if ( K < PHYS_EPSILON )
    return false;

  double lambda = rhs / K;

  // The torque cap per physics step:
  double max_impulse = motor_max_torque * dt;

  double old_impulse = accumulated_motor_impulse;
  double new_impulse = old_impulse + lambda;

  toRange( new_impulse, -max_impulse, max_impulse );

  accumulated_motor_impulse = new_impulse;
  lambda = new_impulse - old_impulse;

  Vector3 angular_impulse = axis * lambda;

  objA->applyAngularImpulse( -angular_impulse );
  objB->applyAngularImpulse(  angular_impulse );

  // The motor is torque-limited: when saturated, some velocity error always remains,
  // and reporting it would keep the solver iterating at the cap forever.
  // So the motor never reports an error and lets the other constraints decide
  // when the solve has settled.
  return false;
}

double HingeJoint::hingeAngularMassInv() const
{
  // Public API: recomputed rather than read from the cache, so that it stays correct for callers outside the solver
  // (for example: MotorJointController::angularMassInv(), diagnostics).
  Vector3 axis = worldAxisA();

  Vector3 IA = objA->inertia_tensor_world_inv * axis;
  Vector3 IB = objB->inertia_tensor_world_inv * axis;

  return axis * IA + axis * IB;
}

} // namespace phys
} // namespace zygo
