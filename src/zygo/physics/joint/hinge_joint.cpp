#include "hinge_joint.h"
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/phys_consts.h>
#include <zygo/math/common/color.h>


namespace zygo {
namespace phys {

HingeJoint::HingeJoint(
  RigidBody * a,
  RigidBody * b,
  Vector3 anchor_mm,
  Vector3 axis_world,
  SolverSettings const * settings
)
  : objA( a )
  , objB( b )
  , settings( settings )
{
  ZgAssert( settings != nullptr );

  Vector3 anchor = anchor_mm / MILLIMETERS_IN_METER;

  local_anchor_A = objA->worldPointToLocal( anchor );
  local_anchor_B = objB->worldPointToLocal( anchor );

  // axis
  axis_world = Vector3::safeNormalized( axis_world );
  local_axis_A = Vector3::safeNormalized( objA->worldVectorToLocal( axis_world ) );
  local_axis_B = Vector3::safeNormalized( objB->worldVectorToLocal( axis_world ) );

  // hinge limit reference: any world vector perpendicular to the axis.
  Vector3 ref_world, tmp;
  axis_world.buildOrthonormalBasisFromAxis( ref_world, tmp );

  local_ref_A = Vector3::safeNormalized( objA->worldVectorToLocal( ref_world ) );
  local_ref_B = Vector3::safeNormalized( objB->worldVectorToLocal( ref_world ) );
}

Vector3 HingeJoint::worldAnchorA() const { return objA->localPointToWorld( local_anchor_A ); }
Vector3 HingeJoint::worldAnchorB() const { return objB->localPointToWorld( local_anchor_B ); }

Vector3 HingeJoint::worldAxisA() const { return Vector3::safeNormalized( objA->localDirToWorld( local_axis_A ) ); }
Vector3 HingeJoint::worldAxisB() const { return Vector3::safeNormalized( objB->localDirToWorld( local_axis_B ) ); }


// Effective mass matrix K = (1/mA + 1/mB) * I + [rA]x * invIA * [rA]x^T + [rB]x * invIB * [rB]x^T
// for the anchor constraint (3 linear rows).
Mat3 HingeJoint::computeAnchorEffectiveMass(
  double inv_mass_A,
  double inv_mass_B,
  Vector3 const & rA,
  Vector3 const & rB,
  Mat3 const & inv_IA,
  Mat3 const & inv_IB,
  double softness
)
{
  Mat3 rAx = Mat3::skew( rA );
  Mat3 rBx = Mat3::skew( rB );

  double coef = inv_mass_A + inv_mass_B;
  Mat3 K = Mat3::identity() * coef;

  K += ( rAx * inv_IA ).mulTransposedRight( rAx );
  K += ( rBx * inv_IB ).mulTransposedRight( rBx );

  K.m[0] += softness;
  K.m[4] += softness;
  K.m[8] += softness;

  return K;
}

// -------------------------------------------------------------------------------------------------
void HingeJoint::draw( IPhysicsDrawer const& drawer, double axis_length ) const
{
  Vector3 p1 = worldAnchorA();
  Vector3 p2 = worldAnchorB();

  drawer.line( p1, p1 + worldAxisA() * axis_length, makeColorLighter( objA->getColor() ) );
  drawer.line( p2, p2 + worldAxisB() * axis_length, makeColorLighter( objB->getColor() ) );
}

// ------------------------------------------------------------------------- reaction loads
//
// The accumulated impulses ARE the reaction, integrated over the substep.
// Dividing by the substep length turns them back into a force and a torque.
//
// Nothing here is measured or estimated: these are the exact numbers the solver applied.

namespace
{
  // Guards the division on the very first call, before any substep has run.
  inline double safeInvDt( double dt )
  {
    return (dt > 0.0) ? (1.0 / dt) : 0.0;
  }
}

Vector3 HingeJoint::reactionForce() const
{
  return accumulated_anchor_impulse * safeInvDt( last_substep_dt );
}

Vector3 HingeJoint::reactionTorque() const
{
  return accumulated_axis_impulse * safeInvDt( last_substep_dt );
}

double HingeJoint::motorTorque() const
{
  return accumulated_motor_impulse * safeInvDt( last_substep_dt );
}

double HingeJoint::limitTorque() const
{
  // Both accumulators are one-sided and clamped to [0, max], so the SIGN lives in the Jacobian, not in the stored value:
  //   the lower limit pushes the angle up   ( JwB = +axis )
  //   the upper limit pushes the angle down ( JwB = -axis )
  // Only one of them can be loaded at a time, so the difference is the signed torque, in the same convention as motorTorque().
  
  return (accumulated_lower_limit_impulse - accumulated_upper_limit_impulse) * safeInvDt( last_substep_dt );
}

// ------------------------------------------------------------------------ state snapshot
void HingeJoint::saveState( HingeJointState & out ) const
{
  out.accumulated_anchor_impulse       = accumulated_anchor_impulse;
  out.accumulated_axis_impulse         = accumulated_axis_impulse;
  out.accumulated_motor_impulse        = accumulated_motor_impulse;
  out.accumulated_lower_limit_impulse  = accumulated_lower_limit_impulse;
  out.accumulated_upper_limit_impulse  = accumulated_upper_limit_impulse;
  out.accumulated_impulse_dt           = accumulated_impulse_dt;
  out.last_substep_dt                  = last_substep_dt;

  out.motor_mode            = (int)motor_mode;
  out.motor_target_velocity = motor_target_velocity;
  out.motor_target_angle    = motor_target_angle;
  out.motor_target_torque   = motor_target_torque;
  out.motor_max_torque      = motor_max_torque;
  out.motor_max_velocity    = motor_max_velocity;

  out.wake_requested = wake_requested;
}

void HingeJoint::restoreState( HingeJointState const & in )
{
  accumulated_anchor_impulse       = in.accumulated_anchor_impulse;
  accumulated_axis_impulse         = in.accumulated_axis_impulse;
  accumulated_motor_impulse        = in.accumulated_motor_impulse;
  accumulated_lower_limit_impulse  = in.accumulated_lower_limit_impulse;
  accumulated_upper_limit_impulse  = in.accumulated_upper_limit_impulse;
  accumulated_impulse_dt           = in.accumulated_impulse_dt;
  last_substep_dt                  = in.last_substep_dt;

  motor_mode            = (MotorMode)in.motor_mode;
  motor_target_velocity = in.motor_target_velocity;
  motor_target_angle    = in.motor_target_angle;
  motor_target_torque   = in.motor_target_torque;
  motor_max_torque      = in.motor_max_torque;
  motor_max_velocity    = in.motor_max_velocity;

  wake_requested = in.wake_requested;

  // The per-substep cache belongs to the substep that built it, and the bodies have just
  // been moved underneath it. Invalidating is enough - prepareVelocitySolve() rebuilds it
  // before anything reads it.
  cache_valid  = false;
  anchor_valid = false;
  axis_valid   = false;
}

} // namespace phys
} // namespace zygo
