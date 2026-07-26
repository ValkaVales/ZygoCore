#include "hinge_joint.h"
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/phys_consts.h>


namespace zygo {
namespace phys {

#ifdef USE_POSITION_SOLVER
bool HingeJoint::solvePositionConstraint()
{
  bool has_error = false;
  if ( solveAnchorPosition() ) has_error = true;
  //if ( solveAxisPosition  () ) has_error = true; // parked: the velocity solver handles the axis tilt well enough
  return has_error;
}

bool HingeJoint::solveAnchorPosition()
{
  Vector3 pA = worldAnchorA();
  Vector3 pB = worldAnchorB();

  Vector3 err = pA - pB;
  double err_len = err.length();
  if ( err_len < settings->joints.position_solver_activation_error )
    return false;

  double correction_len = err_len * settings->joints.position_solver_beta;
  applyMax( correction_len, settings->joints.max_position_linear_correction );

  Vector3 correction = err * (correction_len / err_len);

  // As in the velocity solver, a shared point avoids introducing a spurious force couple.
  Vector3 p = (pA + pB) * 0.5;

  Vector3 rA = p - objA->center_of_mass_pos;
  Vector3 rB = p - objB->center_of_mass_pos;

  Mat3 K = computeAnchorEffectiveMass(
    objA->inv_mass,
    objB->inv_mass,
    rA,
    rB,
    objA->inertia_tensor_world_inv,
    objB->inertia_tensor_world_inv,
    settings->joints.softness
  );

  // We need to reduce the error, hence RHS = -correction.
  // K is symmetric positive definite - see the note in solveAnchorVelocity().
  Vector3 impulse;
  //if ( !K.trySolveLDLT( -correction, impulse ) )
  if ( !K.trySolve( -correction, impulse ) )
    return false; // degenerate effective mass - nothing to correct

  double impulse_len = impulse.limitLength( settings->joints.max_impulse );

  double max_angular_correction = settings->joints.max_position_angular_correction;

  objA->applyPositionImpulseAtWorldPoint(  impulse, p, max_angular_correction );
  objB->applyPositionImpulseAtWorldPoint( -impulse, p, max_angular_correction );

  return impulse_len > settings->joints.min_error_for_impulse;
}

bool HingeJoint::solveAxisPosition()
{
  Vector3 aA = worldAxisA();
  Vector3 aB = worldAxisB();

  ZgAssert( aA.isNormalized() );
  ZgAssert( aB.isNormalized() );

  if ( aA * aB < 0.0 )
    aB = -aB;

  Vector3 err = aA.crossProduct( aB );
  double err_len = err.length();
  if ( err_len < settings->joints.angular_slop )
    return false;

  Vector3 dir = err / err_len;

  double correction_angle = err_len * settings->joints.position_solver_beta;
  applyMax( correction_angle, settings->joints.max_position_angular_correction );

  Vector3 correction = dir * correction_angle;

  double wA = 0.0;
  double wB = 0.0;

  if ( !objA->isStatic() )
    wA = dir * ( objA->inertia_tensor_world_inv * dir );

  if ( !objB->isStatic() )
    wB = dir * ( objB->inertia_tensor_world_inv * dir );

  double sum_w = wA + wB;
  if ( sum_w < PHYS_EPSILON )
    return false;

  objA->applyOrientationCorrection(  correction * (wA / sum_w) );
  objB->applyOrientationCorrection( -correction * (wB / sum_w) );

  return true;
}
#endif

} // namespace phys
} // namespace zygo
