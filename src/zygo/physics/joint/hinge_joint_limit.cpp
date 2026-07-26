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

  // Both are constants of the substep - see prepareVelocitySolve().
  Vector3 const & axis  = cached_axis_A;
  double  const   angle = cached_hinge_angle;

  // Solving the inequality C >= 0:
  // lower limit: angle >= min_angle  ->  C = angle - min_angle
  // upper limit: angle <= max_angle  ->  C = max_angle - angle
  double C = 0.0;

  // Jacobian over the angular velocities:
  // Cdot = JwA*wA + JwB*wB
  Vector3 JwA, JwB;

  double * accumulated = nullptr;

  if ( angle < limits.min_angle_rad - settings->limits.slop )
  {
    // lower: angle >= min
    // C = angle - min >= 0
    C = angle - limits.min_angle_rad;   // < 0 when violated

    // angleDot = -(wRel*axis) = -(wA-wB)*axis
    //  => Cdot = angleDot
    //  => JwA = -axis, JwB = +axis
    JwA = -axis;
    JwB =  axis;

    accumulated = &accumulated_lower_limit_impulse;

    // The upper limit is definitely inactive now.
    accumulated_upper_limit_impulse = 0.0;
  } else
  if ( angle > limits.max_angle_rad + settings->limits.slop )
  {
    // upper: angle <= max
    // C = max - angle >= 0
    C = limits.max_angle_rad - angle;   // < 0 when violated

    // C = max - angle
    // => Cdot = -angleDot = +(wA-wB)*axis
    // => JwA = +axis, JwB = -axis
    JwA =  axis;
    JwB = -axis;

    accumulated = &accumulated_upper_limit_impulse;

    // The lower limit is definitely inactive now.
    accumulated_lower_limit_impulse = 0.0;
  } else
  {
    // Inside the allowed range - both one-sided constraints are inactive.
    accumulated_lower_limit_impulse = 0.0;
    accumulated_upper_limit_impulse = 0.0;

    return false;
  }

  Vector3 wA = objA->angular_speed;
  Vector3 wB = objB->angular_speed;

  double Cdot = JwA * wA + JwB * wB;

  // The bias must push toward C -> 0.
  double bias = settings->limits.beta * C / dt;

  Vector3 IAJwA = objA->inertia_tensor_world_inv * JwA;
  Vector3 IBJwB = objB->inertia_tensor_world_inv * JwB;

  double K = JwA * IAJwA + JwB * IBJwB + settings->limits.softness;
  if ( std::abs( K ) < PHYS_EPSILON )
    return false;

  double lambda = -(Cdot + bias) / K;

  // One-sided constraint: the accumulated impulse is clamped to [0, max].
  double old_accumulated = *accumulated;
  double new_accumulated = old_accumulated + lambda;
  toRange( new_accumulated, 0.0, settings->limits.max_impulse );
  *accumulated = new_accumulated;

  lambda = new_accumulated - old_accumulated;

  Vector3 impulseA = JwA * lambda;
  Vector3 impulseB = JwB * lambda;

  objA->applyAngularImpulse( impulseA );
  objB->applyAngularImpulse( impulseB );

  // lambda is the SIGNED delta of the accumulated impulse: it is negative whenever the solver is releasing a limit it over-pushed on the previous iteration.
  // That is just as much "still has error" as pushing, so the magnitude is what matters here.
  return std::abs( lambda ) > settings->limits.min_error_for_impulse;
}

} // namespace phys
} // namespace zygo
