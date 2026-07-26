#include "hinge_joint.h"
#include <zygo/physics/body/rigid_body.h>


namespace zygo {
namespace phys {

#ifdef USE_VELOCITY_SOLVER
#ifdef DEBUG_CONSERVATION_CHECKS
Vector3 HingeJoint::calcTotalL() const
{
  Vector3 system_com =
    (objA->center_of_mass_pos * objA->total_mass +
      objB->center_of_mass_pos * objB->total_mass)
    / (objA->total_mass + objB->total_mass);

  Vector3 PA = objA->speed * objA->total_mass;
  Vector3 PB = objB->speed * objB->total_mass;

  Vector3 LA_orb = (objA->center_of_mass_pos - system_com).crossProduct( PA );
  Vector3 LB_orb = (objB->center_of_mass_pos - system_com).crossProduct( PB );

  Vector3 L_spin =
    objA->inertia_tensor_world * objA->angular_speed +
    objB->inertia_tensor_world * objB->angular_speed;

  return LA_orb + LB_orb + L_spin;
}
#endif

bool HingeJoint::solveVelocityConstraint( double dt ) // returns true, if still has error
{
  bool has_error = false;
  if ( solveMotorVelocityConstraint ( dt ) ) has_error = true;
  if ( solveAnchorVelocity          ( dt ) ) has_error = true;
  if ( solveAxisVelocity            ( dt ) ) has_error = true;
  if ( solveAngleLimitVelocity      ( dt ) ) has_error = true;
  return has_error;
}


#if 1
bool HingeJoint::solveAnchorVelocity( double /*dt*/ ) // returns true, if still has error
{
  // The geometry, the bias and the inverse effective mass were built once, in prepareAnchorConstraint().
  // Degenerate effective mass (e.g. both bodies static) leaves anchor_valid false and there is nothing to solve.
  if ( !anchor_valid )
    return false;

  Vector3 const uA = objA->speed + objA->angular_speed.crossProduct( anchor_rA );
  Vector3 const uB = objB->speed + objB->angular_speed.crossProduct( anchor_rB );

  Vector3 const relV = uA - uB;
  Vector3 const rhs  = -( relV + anchor_bias );

  Vector3 const J = anchor_K_inv * rhs;

  // Accumulate, clamp the TOTAL, apply only the delta.
  //
  // Clamping the total rather than each increment is what makes the accumulator a
  // meaningful quantity: at the end of the substep it holds the whole impulse this
  // constraint applied, which is exactly what the next substep warm-starts from.
  Vector3 const old_impulse = accumulated_anchor_impulse;

  Vector3 new_impulse = old_impulse + J;
  new_impulse.limitLength( settings->joints.max_impulse );

  accumulated_anchor_impulse = new_impulse;

  Vector3 const delta = new_impulse - old_impulse;

#ifdef DEBUG_CONSERVATION_CHECKS
  // Angular momentum is only conserved between two DYNAMIC bodies: a static one
  // silently absorbs whatever is applied to it, which is the whole point of it.
  bool const check_L = !objA->isStatic() && !objB->isStatic();

  Vector3 L_before;
  if ( check_L )
    L_before = calcTotalL();
#endif

  objA->applyImpulseAtWorldPoint(  delta, anchor_point );
  objB->applyImpulseAtWorldPoint( -delta, anchor_point );

#ifdef DEBUG_CONSERVATION_CHECKS
  if ( check_L )
  {
    Vector3 dL = calcTotalL() - L_before;
    ZgAssert( dL.isZeroVector( BIG_EPSILON ) );
  }
#endif

  // The remaining error is the size of THIS correction, not of the accumulated impulse:
  // a fully converged joint under load carries a large impulse and zero correction.
  return delta.length() > settings->joints.min_error_for_j;
}

bool HingeJoint::solveAxisVelocity( double /*dt*/ ) // returns true, if still has error
{
  // Rows, bias and the inverse 2x2 effective mass come from prepareAxisConstraint().
  if ( !axis_valid )
    return false;

  Vector3 const wRel = objA->angular_speed - objB->angular_speed;

  double const rhs1 = -( wRel * axis_g1 + axis_bias1 );
  double const rhs2 = -( wRel * axis_g2 + axis_bias2 );

  double const lambda1 = axis_inv_K11 * rhs1 + axis_inv_K12 * rhs2;
  double const lambda2 = axis_inv_K21 * rhs1 + axis_inv_K22 * rhs2;

  // Accumulate in WORLD space, not as (lambda1, lambda2): the tangent basis is rebuilt
  // every substep and buildOrthonormalBasisFromAxis() switches helper vector at
  // |axis.x| = 0.7, so a pair of lambdas can silently change meaning mid-flight - which
  // is exactly what happens to an abduction hinge when the robot tips past 45 degrees.
  Vector3 const old_impulse = accumulated_axis_impulse;

  Vector3 new_impulse = old_impulse + axis_g1 * lambda1 + axis_g2 * lambda2;
  new_impulse.limitLength( settings->joints.max_impulse );

  accumulated_axis_impulse = new_impulse;

  Vector3 const delta = new_impulse - old_impulse;

#ifdef DEBUG_CONSERVATION_CHECKS
  // See solveAnchorVelocity(): only meaningful when both bodies are dynamic.
  bool const check_L = !objA->isStatic() && !objB->isStatic();

  Vector3 L_before;
  if ( check_L )
    L_before = calcTotalL();
#endif

  objA->applyAngularImpulse(  delta );
  objB->applyAngularImpulse( -delta );

#ifdef DEBUG_CONSERVATION_CHECKS
  if ( check_L )
  {
    Vector3 dL = calcTotalL() - L_before;
    ZgAssert( dL.isZeroVector( BIG_EPSILON ) );
  }
#endif

  return delta.length() > settings->joints.min_error_for_angular_impulse;
}

#else

bool HingeJoint::solveAnchorVelocity( double dt ) // returns true, if still has error
{
  // The geometry, the bias and the inverse effective mass were built once, in prepareAnchorConstraint().
  // Degenerate effective mass (e.g. both bodies static) leaves anchor_valid false and there is nothing to solve.
  if ( !anchor_valid )
    return false;

  Vector3 pA = worldAnchorA();
  Vector3 pB = worldAnchorB();

  // One shared application point for +J and -J:
  // zero net impulse/torque => the system momentum is conserved exactly.
  Vector3 p = (pA + pB) * 0.5;
  Vector3 pos_error = pA - pB;

  pA = p;
  pB = p;

  Vector3 rA = pA - objA->center_of_mass_pos;
  Vector3 rB = pB - objB->center_of_mass_pos;

  Vector3 uA = objA->speed + objA->angular_speed.crossProduct( rA );
  Vector3 uB = objB->speed + objB->angular_speed.crossProduct( rB );

  Vector3 relV = uA - uB;

  // A small positional bias to pull the points together once they have drifted apart.
  // Deadband: applied only when the error exceeds the slop, otherwise numerical
  // jitter would produce parasite impulses.
  Vector3 bias;
  double err_len = pos_error.length();
  if ( err_len > settings->joints.linear_slop )
  {
    double effective_error = err_len - settings->joints.linear_slop;

    bias = pos_error * (settings->joints.position_beta * effective_error / (err_len * dt));
  }

  bias.limitLength( settings->joints.max_bias_speed );

  Vector3 rhs = -(relV + bias);

  Mat3 K = computeAnchorEffectiveMass(
    objA->inv_mass,
    objB->inv_mass,
    rA,
    rB,
    objA->inertia_tensor_world_inv,
    objB->inertia_tensor_world_inv,
    settings->joints.softness
  );

  // K is symmetric positive definite.
  // trySolveLDLT() is the faster and more accurate path here (sqrt-free Cholesky, and it verifies positive definiteness instead of silently returning garbage).
  // Flip the two lines to A/B it.
  // trySolveSPD() is the old sqrt-based Cholesky: measurably slower.
  Vector3 J;
  //if ( !K.trySolveLDLT( rhs, J ) )
  if ( !K.trySolve( rhs, J ) )
    return false; // degenerate effective mass (e.g. both bodies static) - nothing to solve

  double J_len = J.limitLength( settings->joints.max_impulse );

#ifdef DEBUG_CONSERVATION_CHECKS
  // Angular momentum is only conserved between two DYNAMIC bodies:
  // a static one silently absorbs whatever is applied to it, which is the whole point of it.
  bool const check_L = !objA->isStatic() && !objB->isStatic();

  Vector3 L_before;
  if ( check_L )
    L_before = calcTotalL();
#endif

  objA->applyImpulseAtWorldPoint(  J, pA );
  objB->applyImpulseAtWorldPoint( -J, pB );

#ifdef DEBUG_CONSERVATION_CHECKS
  if ( check_L )
  {
    Vector3 L_after = calcTotalL();
    Vector3 dL = L_after - L_before;
    ZgAssert( dL.isZeroVector( BIG_EPSILON ) );
  }
#endif

  return J_len > settings->joints.min_error_for_j;
}

bool HingeJoint::solveAxisVelocity( double dt ) // returns true, if still has error
{
  Vector3 aA = worldAxisA();
  Vector3 aB = worldAxisB();

  ZgAssert( aA.isNormalized() );
  ZgAssert( aB.isNormalized() );

  // For a hinge, +a and -a are equivalent as an axis.
  // Bring both into one hemisphere.
  if ( (aA * aB) < 0.0 )
    aB = -aB;

  // Two world vectors perpendicular to the A axis.
  Vector3 t1, t2;
  aA.buildOrthonormalBasisFromAxis( t1, t2 );

  Vector3 wRel = objA->angular_speed - objB->angular_speed;

  // The two rows of the angular constraint.
  Vector3 g1 = t1.crossProduct( aB );
  Vector3 g2 = t2.crossProduct( aB );

  double g1_len_sqr = g1.lengthSqr();
  double g2_len_sqr = g2.lengthSqr();
  if ( g1_len_sqr < PHYS_EPSILON && g2_len_sqr < PHYS_EPSILON )
    return false;

  // The velocity error.
  double Cdot1 = wRel * g1;
  double Cdot2 = wRel * g2;

  // The positional axis tilt error: how far aB left the (t1,t2) plane.
  // = the projections of aB on t1,t2 (ideally both zero, since aB should be along aA).
  double C1 = t1 * aB;
  double C2 = t2 * aB;

  // Baumgarte with a deadband: the bias only fires when the tilt really exceeds the threshold.
  // Without it, C1,C2 on numerical jitter produce a parasite torque => a drift out of the plane.
  double bias1 = 0.0;
  double bias2 = 0.0;

  double c_len = sqrt( C1*C1 + C2*C2 );
  if ( c_len > settings->joints.angular_slop )
  {
    double scale = settings->joints.angular_beta * (c_len - settings->joints.angular_slop) / (c_len * dt);
    bias1 = C1 * scale;
    bias2 = C2 * scale;

    double bias_len = sqrt( sqr(bias1) + sqr(bias2) );

    if ( bias_len > settings->joints.max_axis_bias_speed )
    {
      double s = settings->joints.max_axis_bias_speed / bias_len;
      bias1 *= s;
      bias2 *= s;
    }
  }

  double rhs1 = -(Cdot1 + bias1);
  double rhs2 = -(Cdot2 + bias2);

  Vector3 Ig1A = objA->inertia_tensor_world_inv * g1;
  Vector3 Ig2A = objA->inertia_tensor_world_inv * g2;
  Vector3 Ig1B = objB->inertia_tensor_world_inv * g1;
  Vector3 Ig2B = objB->inertia_tensor_world_inv * g2;

  double K11 = g1 * (Ig1A + Ig1B) + settings->joints.softness;
  double K12 = g1 * (Ig2A + Ig2B);
  double K21 = g2 * (Ig1A + Ig1B);
  double K22 = g2 * (Ig2A + Ig2B) + settings->joints.softness;

  double det = K11 * K22 - K12 * K21;
  if ( std::abs( det ) < PHYS_EPSILON )
    return false;

  double inv_det = 1.0 / det;

  double lambda1 = inv_det * (  K22 * rhs1 - K12 * rhs2 );
  double lambda2 = inv_det * ( -K21 * rhs1 + K11 * rhs2 );

  Vector3 angular_impulse = g1 * lambda1 + g2 * lambda2;
  double angular_impulse_len = angular_impulse.limitLength( settings->joints.max_impulse );

#ifdef DEBUG_CONSERVATION_CHECKS
  // See solveAnchorVelocity(): only meaningful when both bodies are dynamic.
  bool const check_L = !objA->isStatic() && !objB->isStatic();

  Vector3 L_before;
  if ( check_L )
    L_before = calcTotalL();
#endif

  objA->applyAngularImpulse(  angular_impulse );
  objB->applyAngularImpulse( -angular_impulse );

#ifdef DEBUG_CONSERVATION_CHECKS
  if ( check_L )
  {
    Vector3 L_after = calcTotalL();
    Vector3 dL = L_after - L_before;
    ZgAssert( dL.isZeroVector( BIG_EPSILON ) );
  }
#endif

  return angular_impulse_len > settings->joints.min_error_for_angular_impulse;
}

#endif // if 1
#endif // USE_VELOCITY_SOLVER

} // namespace phys
} // namespace zygo
