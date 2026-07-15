#include "rigid_body.h"
#include <zygo/physics/phys_math.h>
#include <zygo/physics/phys_consts.h>

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

RigidBody::RigidBody( uint color )
  : initialized ( false )

  , total_mass  ( 0.0 )
  , inv_mass    ( 0.0 )
  , color       ( color )

  , inertia_tensor_local    ( 3, 3 )
  , inertia_tensor_local_inv( 3, 3 )
  , inertia_tensor_world    ( 3, 3 )
  , inertia_tensor_world_inv( 3, 3 )
{
  inertia_tensor_local    .makeAllZero();
  inertia_tensor_local_inv.makeAllZero();
  inertia_tensor_world    .makeAllZero();
  inertia_tensor_world_inv.makeAllZero();
}

void RigidBody::clearGeometry()
{
  shapes.clear();

  total_mass = 0.0;
  inv_mass = 0.0;

  inertia_tensor_local    .makeAllZero();
  inertia_tensor_local_inv.makeAllZero();
  inertia_tensor_world    .makeAllZero();
  inertia_tensor_world_inv.makeAllZero();

  center_of_mass_pos.reset();
  speed             .reset();
  angular_speed     .reset();

  rotation_quaternion = Quaternion();
}

void RigidBody::normalizeQuaternion()
{
  rotation_quaternion.normalize();
}

void RigidBody::updateWorldInertia()
{
  Matrix R = rotation_quaternion.toRotationMatrix();
  Matrix Rt = R.transpose();

  inertia_tensor_world     = R * inertia_tensor_local     * Rt;
  inertia_tensor_world_inv = R * inertia_tensor_local_inv * Rt;

  syncAngularMomentumFromAngularSpeed();
}

Vector3 RigidBody::localPointToWorld( Vector3 const & point_local ) const
{
  return center_of_mass_pos + rotation_quaternion.rotateVector3( point_local );
}

Vector3 RigidBody::worldPointToLocal( Vector3 const & point_world ) const
{
  return rotation_quaternion.rotateInverseVector3( point_world - center_of_mass_pos );
}

Vector3 RigidBody::localDirToWorld( Vector3 const & dir_local ) const
{
  return rotation_quaternion.rotateVector3( dir_local );
}

Vector3 RigidBody::pointVelocityWorld( Vector3 const & point_world ) const
{
  Vector3 r = point_world - center_of_mass_pos;
  return speed + angular_speed.crossProduct( r );
}

Vector3 RigidBody::worldVectorToLocal( Vector3 const & v ) const
{
  return rotation_quaternion.getConjugate().rotateVector3( v );
}

Vector3 RigidBody::localVectorToWorld( Vector3 const & v ) const
{
  return rotation_quaternion.rotateVector3( v );
}

void RigidBody::applyPositionImpulseAtWorldPoint( Vector3 const & impulse, Vector3 const & world_point )
{
  if ( isStatic() )
    return;

  Vector3 r = world_point - center_of_mass_pos;

  // A positional pseudo-impulse: moves the position but does NOT change the speed.
  center_of_mass_pos += impulse * inv_mass;

  Vector3 moment_of_impulse = r.crossProduct( impulse );
  Vector3 angular_correction = inertia_tensor_world_inv.multiplyByVector3( moment_of_impulse );

  angular_correction.limitLength( MAX_POSITION_ANGULAR_CORRECTION );

  applyOrientationCorrection( angular_correction );
}

void RigidBody::applyImpulseAtWorldPoint( Vector3 const & impulse, Vector3 const & world_point )
{
  if ( isStatic() )
    return;

  speed += impulse * inv_mass;

  Vector3 r = world_point - center_of_mass_pos;
  Vector3 angular_impulse = r.crossProduct( impulse );

  applyAngularImpulse( angular_impulse );
}

void RigidBody::applyOrientationCorrection( Vector3 const & small_angle )
{
  if ( isStatic() )
    return;

  double angle = small_angle.length();
  if ( angle < PHYS_EPSILON )
    return;

  Vector3 axis = small_angle / angle;

  Quaternion dq = Quaternion::calcRotationQuaternion( axis, angle );

  rotation_quaternion = dq.crossProduct( rotation_quaternion );
  rotation_quaternion.normalize();

  updateWorldInertia();
}

void RigidBody::applyAngularImpulse( Vector3 const & angular_impulse )
{
  if ( isStatic() )
    return;

  Vector3 mul = inertia_tensor_world_inv.multiplyByVector3( angular_impulse );
  angular_speed += mul;

  syncAngularMomentumFromAngularSpeed();
}

void RigidBody::addAngularSpeed( Vector3 const & angular_speed_addon )
{
  ZgAssert( initialized );

  angular_speed += angular_speed_addon;
  syncAngularMomentumFromAngularSpeed();
}

void RigidBody::syncAngularMomentumFromAngularSpeed()
{
  // The angular momentum is not stored explicitly, so there is nothing to sync.
  // The hook is kept so that switching to an L-based representation stays a local change.
}

void RigidBody::applyGravity( Vector3 const & gravity, double dt )
{
  if ( isStatic() )
    return;

  speed += gravity * dt;
}

double RigidBody::invEffectiveMassAlong( Vector3 const & world_point, Vector3 const & dir ) const
{
  if ( isStatic() )
    return 0.0;

  Vector3 r     = world_point - center_of_mass_pos;
  Vector3 rxd   = r.crossProduct( dir );
  Vector3 I_rxd = inertia_tensor_world_inv.multiplyByVector3( rxd );

  return inv_mass + rxd * I_rxd; // dot
}

void RigidBody::calcMassAndLocalCenterOfMass()
{
  total_mass = 0.0;
  center_of_mass_pos.reset();

  double mass_sum = 0.0;

  for ( auto const & shape : shapes )
  {
    if ( shape.get() == nullptr )
      continue;

    double m = shape->getMass();
    Vector3 shape_com = shape->calcLocalCenterOfMass();

    total_mass += m;
    center_of_mass_pos += shape_com * m;
    mass_sum += m;
  }

  ZgAssert( mass_sum > BIG_EPSILON );

  center_of_mass_pos /= mass_sum;

  inv_mass = 1.0 / total_mass;
  if ( inv_mass < SMALL_EPSILON )
    inv_mass = 0.0; // the mass is so huge that the body is effectively static
}

void RigidBody::rebuildPhysicalParameters_afterAllShapesAdded() // should be called only once, after all shapes have been added
{
  ZgAssert( !initialized );
  initialized = true;

  total_mass  = 0.0;
  inv_mass    = 0.0;

  inertia_tensor_local    .makeAllZero();
  inertia_tensor_local_inv.makeAllZero();
  inertia_tensor_world    .makeAllZero();
  inertia_tensor_world_inv.makeAllZero();

  if ( shapes.empty() )
    return;

  // 1. The mass and the local center of mass.
  calcMassAndLocalCenterOfMass();

  // 2. Shift all shapes so that the center of mass of the body lands at (0,0,0).
  Vector3 minus_center_of_mass_pos = -center_of_mass_pos;

  for ( auto & shape : shapes )
  {
    if ( shape.get() != nullptr )
      shape->addToLocalPos( minus_center_of_mass_pos );
  }

  // 3. The total inertia tensor.
  for ( auto & shape : shapes )
  {
    if ( shape.get() == nullptr )
      continue;

    Matrix I_part_local = shape->calcLocalInertiaTensorForPart();

    // The rotation of the shape inside the body.
    Matrix R = shape->localRot().toRotationMatrix();
    Matrix Rt = R.transpose();

    // Rotate the tensor into the local frame of the body.
    Matrix I_part_rotated = R * I_part_local * Rt;

    // Add the parallel-axis (Steiner) translation.
    Vector3 d = shape->localPos(); // after the shift this is already the vector from the body COM
    Matrix I_shift = calcParallelAxisTerm( shape->getMass(), d );

    inertia_tensor_local += I_part_rotated;
    inertia_tensor_local += I_shift;
  }

  inertia_tensor_local_inv = inertia_tensor_local.inverse3x3();

  validateInertiaTensor( inertia_tensor_local );
  validateInertiaTensor( inertia_tensor_local_inv );

  // 4. The world tensor.
  updateWorldInertia();
}

Vector3 RigidBody::calcOmegaBodyDerivative( Vector3 const & omega_body, Vector3 const & torque_body ) const
{
  Vector3 Iw = inertia_tensor_local.multiplyByVector3( omega_body );
  Vector3 gyro = omega_body.crossProduct( Iw );
  Vector3 rhs = torque_body - gyro;
  return inertia_tensor_local_inv.multiplyByVector3( rhs );
}

void RigidBody::calcMainPhysicalParameters(
  Vector3 const & total_center_of_mass_pos,
  Vector3 & momentum,
  Vector3 & angular_momentum,
  double & kinetic_energy ) const
{
  // 1. Momentum.
  momentum = speed * total_mass;

  // 2. Angular momentum about the center of mass of the WHOLE system.
  Vector3 r = center_of_mass_pos - total_center_of_mass_pos;

  Vector3 orbital_angular_momentum = r.crossProduct( momentum );
  Vector3 spin_angular_momentum = inertia_tensor_world.multiplyByVector3( angular_speed );

  angular_momentum = orbital_angular_momentum + spin_angular_momentum;

  // 3. Kinetic energy.
  double translational_energy = 0.5 * total_mass * speed.lengthSqr();

  Vector3 Iw = inertia_tensor_world.multiplyByVector3( angular_speed );
  double rotational_energy = 0.5 * (angular_speed * Iw); // dot product

  kinetic_energy = translational_energy + rotational_energy;
}

void RigidBody::draw( IPhysicsDrawer const& drawer ) const
{
  for ( auto const & shape : shapes )
    shape->draw( drawer, center_of_mass_pos, rotation_quaternion );
}

} // namespace phys
} // namespace zygo
