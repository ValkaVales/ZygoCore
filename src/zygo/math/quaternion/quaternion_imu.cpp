#include "quaternion.h"


namespace zygo {

Quaternion Quaternion::calcJacobian( Vector3 const& acc ) const // acc - acceleration
{
  Real s2 =   s * REAL_TWO;
  Real x2 = v.x * REAL_TWO;
  Real y2 = v.y * REAL_TWO;
  Real z2 = v.z * REAL_TWO;

  Real x4 = x2 * REAL_TWO;
  Real y4 = y2 * REAL_TWO;

  // calc objective function
  Real fx =              x2 * v.z - s2 * v.y - acc.x;
  Real fy =              s2 * v.x + y2 * v.z - acc.y;
  Real fz = REAL_ONE - x2 * v.x - y2 * v.y - acc.z;

  // calc Jacobian (gradient)
  return Quaternion(
       -y2 * fx + x2 * fy,
        z2 * fx + s2 * fy - x4 * fz,
       -s2 * fx + z2 * fy - y4 * fz,
        x2 * fx + y2 * fy
    );
}

Real Quaternion::calcRungeKuttaDeltas( Vector3 const& gyro, Real dt, Vector3& dv ) const // returns ds
{
  dt *= 0.5;

  Real ds   = (-v.x * gyro.x  -  v.y * gyro.y  -  v.z * gyro.z) * dt;
  dv.x      = (   s * gyro.x  +  v.y * gyro.z  -  v.z * gyro.y) * dt;
  dv.y      = (   s * gyro.y  -  v.x * gyro.z  +  v.z * gyro.x) * dt;
  dv.z      = (   s * gyro.z  +  v.x * gyro.y  -  v.y * gyro.x) * dt;

  return ds;
}

void Quaternion::updateQuaternionUsingRungeKutta( Vector3 const& gyro, Real dt )
{
  Vector3 dv;
  Real ds = calcRungeKuttaDeltas( gyro, dt, dv );

  s += ds;
  v += dv;
  
  normalize();
}

void Quaternion::updateQuaternionUsingRungeKutta2( Vector3 const& gyro, Quaternion const& estimated_gyro_error_direction, Real dt )
{
  Vector3 dv;
  Real ds = calcRungeKuttaDeltas( gyro, dt, dv );

  s += ds - estimated_gyro_error_direction.s * dt;
  v += dv - estimated_gyro_error_direction.v * dt;
  
  normalize();
}

Quaternion Quaternion::accelToQuaternion( Vector3 accel )
{
  Real acc_len = accel.length();
  accel *= REAL_ONE / acc_len;

  Vector3 down_vector( REAL_ZERO, REAL_ZERO, REAL_ONE );
  Quaternion q = calcRotationFromVectorToVector( down_vector, accel );

  return q;
}

} // namespace zygo
