#include "hinge_joint.h"
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/phys_consts.h>

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <zygo/math/common/color.h>


namespace zygo {
namespace phys {

namespace
{
  // Effective mass matrix K = (1/mA + 1/mB) * I + [rA]x * invIA * [rA]x^T + [rB]x * invIB * [rB]x^T
  // for the anchor constraint (3 linear rows).
  Mat3 computeAnchorEffectiveMass(
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
}


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

bool HingeJoint::solveAnchorVelocity( double dt ) // returns true, if still has error
{
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

  // K is symmetric positive definite - solve with the fast Cholesky path.
  Vector3 J;
  //if ( !K.trySolveSPD( rhs, J ) )
  if ( !K.trySolve( rhs, J ) )
    return false; // degenerate effective mass (e.g. both bodies static) - nothing to solve

  double J_len = J.limitLength( settings->joints.max_impulse );

#ifdef DEBUG_CONSERVATION_CHECKS
  Vector3 L_before = calcTotalL();
#endif

  objA->applyImpulseAtWorldPoint(  J, pA );
  objB->applyImpulseAtWorldPoint( -J, pB );

#ifdef DEBUG_CONSERVATION_CHECKS
  Vector3 L_after = calcTotalL();

  Vector3 dL = L_after - L_before;
  ZgAssert( dL.isZeroVector( BIG_EPSILON ) );
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
  Vector3 L_before = calcTotalL();
#endif

  objA->applyAngularImpulse(  angular_impulse );
  objB->applyAngularImpulse( -angular_impulse );

#ifdef DEBUG_CONSERVATION_CHECKS
  Vector3 L_after = calcTotalL();

  Vector3 dL = L_after - L_before;
  ZgAssert( dL.isZeroVector( BIG_EPSILON ) );
#endif

  return angular_impulse_len > settings->joints.min_error_for_angular_impulse;
}
#endif


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
  // K is symmetric positive definite - solve with the fast Cholesky path.
  Vector3 impulse;
  //if ( !K.trySolveSPD( -correction, impulse ) )
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


// -------------------------------------------------------------------------------------------------
void HingeJoint::draw( IPhysicsDrawer const& drawer, double axis_length ) const
{
  Vector3 p1 = worldAnchorA();
  Vector3 p2 = worldAnchorB();

  drawer.line( p1, p1 + worldAxisA() * axis_length, makeColorLighter( objA->getColor() ) );
  drawer.line( p2, p2 + worldAxisB() * axis_length, makeColorLighter( objB->getColor() ) );
}

} // namespace phys
} // namespace zygo
