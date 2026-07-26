#include "hinge_joint.h"
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/phys_consts.h>


namespace zygo {
namespace phys {

// ------------------------------------------------------------------------------------
// Per-substep preparation.
//
// Everything here is a constant of the substep: the velocity loop changes velocities only,
// so the anchor points, the Jacobian rows, the effective mass and the Baumgarte bias never move while it runs.
// ------------------------------------------------------------------------------------
void HingeJoint::prepareAnchorConstraint( double dt )
{
  anchor_valid = false;

  Vector3 const pA = worldAnchorA();
  Vector3 const pB = worldAnchorB();

  // One shared application point for +J and -J:
  // zero net impulse/torque => the system momentum is conserved exactly.
  anchor_point = (pA + pB) * 0.5;

  anchor_rA = anchor_point - objA->center_of_mass_pos;
  anchor_rB = anchor_point - objB->center_of_mass_pos;

  // A small positional bias to pull the points together once they have drifted apart.
  // Deadband: applied only when the error exceeds the slop, otherwise numerical jitter would produce parasite impulses.
  Vector3 const pos_error = pA - pB;

  anchor_bias.reset();

  double const err_len = pos_error.length();
  if ( err_len > settings->joints.linear_slop )
  {
    double const effective_error = err_len - settings->joints.linear_slop;

    anchor_bias = pos_error * (settings->joints.position_beta * effective_error / (err_len * dt));
  }

  anchor_bias.limitLength( settings->joints.max_bias_speed );

  Mat3 const K = computeAnchorEffectiveMass(
    objA->inv_mass,
    objB->inv_mass,
    anchor_rA,
    anchor_rB,
    objA->inertia_tensor_world_inv,
    objB->inertia_tensor_world_inv,
    settings->joints.softness
  );

  // Invert once per substep instead of solving once per iteration: the loop then costs a 3x3 matrix-vector product per iteration instead of a full solve.
  // K is symmetric positive definite - trySolveLDLT()/tryInverse() both apply.
  anchor_valid = K.tryInverse( anchor_K_inv );
}

void HingeJoint::prepareAxisConstraint( double dt )
{
  axis_valid = false;

  Vector3 const aA = cached_axis_A;
  Vector3       aB = worldAxisB();

  // For a hinge, +a and -a are equivalent as an axis. Bring both into one hemisphere.
  if ( (aA * aB) < 0.0 )
    aB = -aB;

  // Two world vectors perpendicular to the A axis.
  Vector3 t1, t2;
  aA.buildOrthonormalBasisFromAxis( t1, t2 );

  // The two rows of the angular constraint.
  axis_g1 = t1.crossProduct( aB );
  axis_g2 = t2.crossProduct( aB );

  double g1_len_sqr = axis_g1.lengthSqr();
  double g2_len_sqr = axis_g2.lengthSqr();
  if ( g1_len_sqr < PHYS_EPSILON && g2_len_sqr < PHYS_EPSILON )
    return;

  // The positional axis tilt error: how far aB left the (t1,t2) plane.
  // = the projections of aB on t1,t2 (ideally both zero, since aB should be along aA).
  double const C1 = t1 * aB;
  double const C2 = t2 * aB;

  // Baumgarte with a deadband: the bias only fires when the tilt really exceeds the
  // threshold. Without it, C1,C2 on numerical jitter produce a parasite torque.
  axis_bias1 = 0.0;
  axis_bias2 = 0.0;

  double const c_len = sqrt( sqr(C1) + sqr(C2) );
  if ( c_len > settings->joints.angular_slop )
  {
    double const scale = settings->joints.angular_beta * (c_len - settings->joints.angular_slop) / (c_len * dt);
    axis_bias1 = C1 * scale;
    axis_bias2 = C2 * scale;

    double const bias_len = sqrt( sqr(axis_bias1) + sqr(axis_bias2) );

    if ( bias_len > settings->joints.max_axis_bias_speed )
    {
      double const s = settings->joints.max_axis_bias_speed / bias_len;
      axis_bias1 *= s;
      axis_bias2 *= s;
    }
  }

  Vector3 const Ig1A = objA->inertia_tensor_world_inv * axis_g1;
  Vector3 const Ig2A = objA->inertia_tensor_world_inv * axis_g2;
  Vector3 const Ig1B = objB->inertia_tensor_world_inv * axis_g1;
  Vector3 const Ig2B = objB->inertia_tensor_world_inv * axis_g2;

  double const K11 = axis_g1 * (Ig1A + Ig1B) + settings->joints.softness;
  double const K12 = axis_g1 * (Ig2A + Ig2B);
  double const K21 = axis_g2 * (Ig1A + Ig1B);
  double const K22 = axis_g2 * (Ig2A + Ig2B) + settings->joints.softness;

  double const det = K11 * K22 - K12 * K21;

  // Relative test - K scales with the inverse inertia, which is ~1e4 for a light link
  // and ~1e-2 for a heavy one, so a fixed threshold means different things for each.
  double const scale = max2( std::abs(K11), max2( std::abs(K12), max2( std::abs(K21), std::abs(K22) ) ) );

  if ( !(std::abs(det) > PHYS_EPSILON * sqr(scale)) )
    return;

  double const inv_det = 1.0 / det;

  axis_inv_K11 =  K22 * inv_det;
  axis_inv_K12 = -K12 * inv_det;
  axis_inv_K21 = -K21 * inv_det;
  axis_inv_K22 =  K11 * inv_det;

  axis_valid = true;
}

void HingeJoint::warmStartVelocitySolve( double dt )
{
  if ( !cache_valid )
    return;

  if ( !settings->joints.warm_starting )
  {
    accumulated_anchor_impulse.reset();
    accumulated_axis_impulse.reset();
    accumulated_impulse_dt = 0.0;
    return;
  }

  // An impulse is force*dt.
  // A stored one only means the same thing at the same dt, so it is rescaled when the substep length changes - and dropped on the first substep, where there is nothing to scale from.
  double const dt_ratio = (accumulated_impulse_dt > 0.0) ? (dt / accumulated_impulse_dt) : 0.0;
  accumulated_impulse_dt = dt;

  double const ratio = dt_ratio * settings->joints.warm_start_factor;

  accumulated_anchor_impulse *= ratio;
  accumulated_axis_impulse *= ratio;

  if ( anchor_valid && !accumulated_anchor_impulse.isZeroVector( PHYS_EPSILON ) )
  {
    objA->applyImpulseAtWorldPoint(  accumulated_anchor_impulse, anchor_point );
    objB->applyImpulseAtWorldPoint( -accumulated_anchor_impulse, anchor_point );
  }

  if ( axis_valid )
  {
    // The axis impulse lives in the plane perpendicular to the hinge axis, and the axis has turned a little since the impulse was stored.
    // Project it back, otherwise a slowly growing component ALONG the axis would leak in and fight the motor.
    accumulated_axis_impulse -= cached_axis_A * (accumulated_axis_impulse * cached_axis_A);

    if ( !accumulated_axis_impulse.isZeroVector( PHYS_EPSILON ) )
    {
      objA->applyAngularImpulse(  accumulated_axis_impulse );
      objB->applyAngularImpulse( -accumulated_axis_impulse );
    }
  }
};

} // namespace phys
} // namespace zygo
