#include "hinge_joint.h"
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/phys_consts.h>

#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

void HingeJoint::enableAngleLimit( JointLimits limits )
{
  requestWake();

  this->limits = limits;
  limit_enabled = true;
}

Vector3 HingeJoint::worldRefA() const
{
  return Vector3::safeNormalized( objA->localDirToWorld( local_ref_A ) );
}

Vector3 HingeJoint::worldRefB() const
{
  return Vector3::safeNormalized( objB->localDirToWorld( local_ref_B ) );
}

double HingeJoint::calcSignedAngleAroundAxis(
  Vector3 const & a,
  Vector3 const & b,
  Vector3 const & axis_unit
)
{
  // Project both vectors onto the plane perpendicular to the axis.
  Vector3 aa = a - axis_unit * ( a * axis_unit );
  Vector3 bb = b - axis_unit * ( b * axis_unit );

  double la2 = aa.lengthSqr();
  double lb2 = bb.lengthSqr();
  if ( la2 < PHYS_EPSILON || lb2 < PHYS_EPSILON )
    return 0.0;

  aa = Vector3::safeNormalized( aa );
  bb = Vector3::safeNormalized( bb );

  double c = aa * bb;
  c = max2( -1.0, min2( 1.0, c ) );

  double angle = acos( c );
  double s = axis_unit * aa.crossProduct( bb );

  if ( s < 0.0 )
    angle = -angle;

  return angle;
}

double HingeJoint::currentHingeAngle() const
{
  Vector3 axis = worldAxisA();

  Vector3 refA = worldRefA();
  Vector3 refB = worldRefB();

  return calcSignedAngleAroundAxis( refA, refB, axis );
}

double HingeJoint::curAngleVelocity() const
{
  Vector3 axis = worldAxisA();
  Vector3 wRel = objB->angular_speed - objA->angular_speed;
  return wRel * axis;
}

bool HingeJoint::solveAngleLimitVelocity( double dt )
{
  if ( !limit_enabled )
    return false;

  // Geometry is constant during the velocity loop; velocities are not.
  Vector3 const & axis  = cached_axis_A;
  double  const   angle = cached_hinge_angle;

  // Use one generic inequality C >= 0.
  //
  // lower: C = angle - min, Cdot = +angleDot
  // upper: C = max - angle, Cdot = -angleDot
  double C = 0.0;
  double Cdot = 0.0;
  double target_Cdot = 0.0;

  Vector3 JwA, JwB;
  double * accumulated = nullptr;

  double const angle_dot = (objB->angular_speed - objA->angular_speed) * axis;

  double const lower_C = angle - limits.min_angle_rad;
  double const upper_C = limits.max_angle_rad - angle;

  bool const lower_violated = (lower_C < -settings->limits.slop);
  bool const upper_violated = (upper_C < -settings->limits.slop);

  // Speculative/predictive activation:
  // while still inside the legal interval, activate the row if the current angular velocity would cross the boundary by the end of this substep.
  // The allowed approach speed is exactly the speed that reaches the limit.
  bool const lower_predicted = (lower_C >= 0.0) && (lower_C + angle_dot * dt < 0.0);
  bool const upper_predicted = (upper_C >= 0.0) && (upper_C - angle_dot * dt < 0.0);

  // Once a speculative row has accumulated an impulse, keep solving that SAME row for the rest of the velocity loop.
  // Otherwise the first correction would make the prediction false, the accumulator would be forgotten, and a neighbouring constraint could make it fire repeatedly with an incorrect total impulse.
  bool const lower_active = lower_violated || lower_predicted || (accumulated_lower_limit_impulse > 0.0);
  bool const upper_active = upper_violated || upper_predicted || (accumulated_upper_limit_impulse > 0.0);

  if ( lower_active && !upper_active )
  {
    C = lower_C;
    Cdot = angle_dot;

    JwA = -axis;
    JwB =  axis;

    accumulated = &accumulated_lower_limit_impulse;
    accumulated_upper_limit_impulse = 0.0;

    target_Cdot = -C / dt; // Still inside: allow approaching, but not fast enough to cross the limit.
    if ( lower_violated )
      target_Cdot *= settings->limits.beta; // Baumgarte recovery after an actual violation.
  } else
  if ( upper_active && !lower_active )
  {
    C = upper_C;
    Cdot = -angle_dot;

    JwA =  axis;
    JwB = -axis;

    accumulated = &accumulated_upper_limit_impulse;
    accumulated_lower_limit_impulse = 0.0; // // The lower limit is definitely inactive now

    target_Cdot = -C / dt; // Still inside: allow approaching, but not fast enough to cross the limit.
    if ( upper_violated )
      target_Cdot *= settings->limits.beta; // Baumgarte recovery after an actual violation.
  } else
  {
    // Both active is only possible for invalid/crossed limits;
    // neither active is the normal inside-range case.
    // In both cases do not inject an ambiguous impulse.
    accumulated_lower_limit_impulse = 0.0;
    accumulated_upper_limit_impulse = 0.0;
    return false;
  }

  Vector3 const IAJwA = objA->inertia_tensor_world_inv * JwA;
  Vector3 const IBJwB = objB->inertia_tensor_world_inv * JwB;

  double const K = JwA * IAJwA + JwB * IBJwB + settings->limits.softness;
  if ( std::abs( K ) < PHYS_EPSILON )
    return false;

  double lambda = (target_Cdot - Cdot) / K;

  // One-sided constraint: the accumulated impulse is clamped to [0, max].
  double const old_accumulated = *accumulated;
  double new_accumulated = old_accumulated + lambda;
  toRange( new_accumulated, 0.0, settings->limits.max_impulse );
  *accumulated = new_accumulated;

  lambda = new_accumulated - old_accumulated;

  Vector3 impulseA = JwA * lambda;
  Vector3 impulseB = JwB * lambda;

  objA->applyAngularImpulse( impulseA );
  objB->applyAngularImpulse( impulseB );

  // A negative delta means the solver is releasing an over-correction;
  // its magnitude still counts as work left for convergence.
  return std::abs(lambda) > settings->limits.min_error_for_impulse;
}

} // namespace phys
} // namespace zygo
