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

// Open-loop torque is applied once per substep by applyExternalActuatorImpulse().
// Keeping it outside the iterative solver makes the injected angular impulse exactly torque*dt, independent of velocity_iterations.
void HingeJoint::setMotorTorque( double torque )
{
  requestWake();

  ZgAssert( std::isfinite( torque ) );

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

void HingeJoint::applyExternalActuatorImpulse( double dt )
{
  if ( motor_mode != MOTOR_TORQUE )
    return;

  ZgAssertRelease( cache_valid );
  ZgAssertRelease( std::isfinite( dt ) && dt > 0.0 );

  // Positive torque increases currentHingeAngle(): objB receives +axis*J and objA receives the equal-and-opposite reaction.
  Vector3 const angular_impulse = cached_axis_A * (motor_target_torque * dt);

  objA->applyAngularImpulse( -angular_impulse );
  objB->applyAngularImpulse(  angular_impulse );
}

void HingeJoint::prepareVelocitySolve( double dt )
{
  // The servo motor and the limits are re-derived from scratch every substep
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

  // MOTOR_TORQUE is handled once per substep by applyExternalActuatorImpulse().
  // Only the velocity/position servo modes belong to the iterative constraint solve.
  if ( motor_mode == MOTOR_TORQUE )
    return false;

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
