#include "rigid_body.h"
#include <zygo/math/common/scalar.h>

// Integrator selection. USE_EULER_BODY_INTEGRATION is the production one:
// a midpoint step of the Euler rotation equations in the body frame,
// which handles the gyroscopic term correctly.
//#define USE_SIMPLE_INTEGRATION
#define USE_EULER_BODY_INTEGRATION


namespace zygo {
namespace phys {

#ifdef USE_SIMPLE_INTEGRATION

void RigidBody::integrateVelocities( double dt )
{
  if ( isStatic() )
    return;

  center_of_mass_pos += speed * dt;

  Quaternion dq = Quaternion::calcRotationQuaternion_fromAngularVelocity( angular_speed, dt );
  rotation_quaternion.rotateByRotationQuaternion( dq );
}

#else

#ifdef USE_EULER_BODY_INTEGRATION
void RigidBody::integrateVelocities( double dt )
{
  if ( isStatic() )
    return;

  center_of_mass_pos += speed * dt;

  // 1. The current omega in the local frame of the body.
  Vector3 omega_body_0 = worldVectorToLocal( angular_speed );

  // 2. For a free body the external torque is zero.
  Vector3 torque_body( 0.0, 0.0, 0.0 );

  // 3. A half step of the Euler equations in the body frame.
  Vector3 domega0 = calcOmegaBodyDerivative( omega_body_0, torque_body );
  Vector3 omega_body_mid = omega_body_0 + domega0 * (dt * 0.5);

  // 4. Rotate the body using omega_mid, but already in world space.
  Vector3 omega_world_mid = localVectorToWorld( omega_body_mid );

  Quaternion dq = Quaternion::calcRotationQuaternion_fromAngularVelocity( omega_world_mid, dt );
  rotation_quaternion.rotateByRotationQuaternion( dq );
  rotation_quaternion.normalize();

  // 5. The full omega_body step through the midpoint RHS.
  Vector3 domega_mid = calcOmegaBodyDerivative( omega_body_mid, torque_body );
  Vector3 omega_body_1 = omega_body_0 + domega_mid * dt;

  // 6. After the orientation update, refresh the world inertia.
  updateWorldInertia();

  // 7. Return angular_speed to the world frame.
  angular_speed = localVectorToWorld( omega_body_1 );
  syncAngularMomentumFromAngularSpeed();
}
#else
// Angular-momentum-based midpoint integrator: L is kept constant over the step,
// omega is re-derived from L and the rotating inertia tensor.
void RigidBody::integrateVelocities( double dt )
{
  if ( isStatic() )
    return;

  center_of_mass_pos += speed * dt;

  Vector3 L_spin = inertia_tensor_world * angular_speed;

  Quaternion q_old = rotation_quaternion;
  Mat3 I_old_inv = inertia_tensor_world_inv;

  // omega at the start of the step
  Vector3 omega0 = I_old_inv * L_spin;

  // a trial half step
  Quaternion dq_half = Quaternion::calcRotationQuaternion_fromAngularVelocity( omega0, dt * 0.5 );
  rotation_quaternion = q_old;
  rotation_quaternion.rotateByRotationQuaternion( dq_half );
  rotation_quaternion.normalize();

  updateWorldInertia();

  // omega at the middle of the step
  Vector3 omega_mid = inertia_tensor_world_inv * L_spin;

  // the full step, now using the midpoint omega
  rotation_quaternion = q_old;
  Quaternion dq_full = Quaternion::calcRotationQuaternion_fromAngularVelocity( omega_mid, dt );
  rotation_quaternion.rotateByRotationQuaternion( dq_full );
  rotation_quaternion.normalize();

  updateWorldInertia();

  // make omega consistent with the new tensor
  angular_speed = inertia_tensor_world_inv * L_spin;
  syncAngularMomentumFromAngularSpeed();
}
#endif

#endif // USE_SIMPLE_INTEGRATION

} // namespace phys
} // namespace zygo
