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

} // namespace phys
} // namespace zygo
