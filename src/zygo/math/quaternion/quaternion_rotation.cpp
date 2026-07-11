#include "quaternion.h"
#include <zygo/math/common/scalar.h>


namespace zygo {

// -------------------------------------------------------------------------- rotation
Quaternion Quaternion::calcRotationQuaternion( Vector3 const& normal, Real angle )
{
  Real a2 = angle * REAL_HALF;
  Real cosa = std::cos( a2 );
  Real sina = std::sin( a2 );

  return Quaternion( cosa, normal * sina );
}

Quaternion Quaternion::calcRotationQuaternion_fromAngularVelocity( Vector3 const& omega, Real dt )
{
  Real len = omega.length();
  Real angle = len * dt;

  if ( angle < EPSILON )
    return Quaternion{ 1.0, 0.0, 0.0, 0.0 };

  Vector3 normal = omega / len; // axis

  return calcRotationQuaternion( normal, angle );
}

Quaternion Quaternion::calcRotationFromVectorToVector( Vector3 const& v0, Vector3 const& v1 )
{
  // Based on Stan Melax's article in Game Programming Gems, 1 article
  Quaternion res;

  Real cos_theta = v0 * v1;
  if ( ge( cos_theta, REAL_ONE ) ) // If dot == 1, vectors are the same
  {
    res.setIdentity();
    return res;
  }

  if ( le( cos_theta, -REAL_ONE ) )
  {
    // special case when vectors are in opposite directions: there is no "ideal" rotation axis.
    // So guess one; any will do as long as it's perpendicular to start.
    // This implementation favors a rotation around the Up axis (Z), since it's often what you want to do.

    Vector3 axis( REAL_ZERO, REAL_ZERO, REAL_ONE );
    axis = axis.crossProduct( v0 );

    if ( isZero( axis.lengthSqr() ) )
    {
      axis.set( REAL_ZERO, REAL_ONE, REAL_ZERO );
      axis = axis.crossProduct( v0 );
    }

    axis.normalize();
    res.s = REAL_ZERO;
    res.v = axis;
    return res;
  }

  //
  Real s = std::sqrt( (1 + cos_theta) * REAL_TWO );
  Vector3 axis2 = v0.crossProduct( v1 ) / s;
  
  // do NOT normalize axis2

  res.s = s * REAL_HALF;
  res.v = axis2;
  return res;
}

void Quaternion::rotateInPlane( Vector3 const& normal, Real angle )
{
  Quaternion q = calcRotationQuaternion( normal, angle );
  //q.normalize();

  *this = crossProduct( q );
}

Vector3 Quaternion::rotateVector3( Vector3 const& v ) const
{
  Quaternion q_conj = getConjugate();
  Quaternion q_target( 0, v );

  Quaternion q_res = crossProduct( q_target ).crossProduct( q_conj );

  Vector3 v_res = q_res.v;
  //v_res.normalize();
  return v_res;
}

Vector3 Quaternion::rotateInverseVector3( Vector3 const& v ) const
{
  Quaternion q = getInverse();
  return q.rotateVector3( v );
}

void Quaternion::rotateCameraVectors( Vector3 const & normal, Real angle, Vector3 & pos, Vector3 & view, Vector3 & up )
{
  Quaternion rot = calcRotationQuaternion( normal, angle );
  rot.normalize();

  pos   = rot.rotateVector3( pos  );
  view  = rot.rotateVector3( view );
  up    = rot.rotateVector3( up   );
}

void Quaternion::rotateByRotationQuaternion( Quaternion const& rotation_quaternion )
{
  //normalize();
  //rotation_quaternion.normalize();

  *this = rotation_quaternion.crossProduct( *this );
  normalize();
}

Vector3 Quaternion::rotateVector3_byRotationQuaternion_usingMatrix( Vector3 const& v ) const
{
  Matrix rot_matrix = toRotationMatrix();
  return rot_matrix.multiplyByVector3( v );
}

void Quaternion::rotateByAngularSpeedVector( Vector3 const& angular_speed, Real dt )
{
  // Convert angular speed into a quaternion, which represent the rotation increment
  Quaternion delta_rotation( REAL_ONE, angular_speed * (dt * REAL_HALF) );
  delta_rotation.normalize();
  rotateByRotationQuaternion( delta_rotation );
}

} // namespace zygo
