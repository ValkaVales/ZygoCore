#pragma once

#include <zygo/math/vector/vec3.h>
#include <zygo/math/matrix/matrix.h>


namespace zygo {

class Quaternion
{
private:
  Real s;
  Vector3 v;

public:
  Quaternion(); // identity quaternion by default
  Quaternion( Real s, Vector3 const & v );
  Quaternion( Real s, Real x, Real y, Real z );
  Quaternion( Quaternion const& q );

  void set( Real s, Vector3 const & v );
  void set( Real s, Real x, Real y, Real z );

  Quaternion getConjugate () const;
  Quaternion getInverse   () const;

  Real getAngle() const;

  Real length   () const;
  Real lengthSqr() const;

  inline Real getS() const { return s; }
  inline Vector3 getV() const { return v; }

  void setIdentity();
  void normalize();

  Real cosPhiBetween( Quaternion const & q ) const;
  bool isEqual( Quaternion const& q, Real epsilon = EPSILON ) const;

  // rotation
  static Quaternion calcRotationQuaternion_fromAngularVelocity( Vector3 const& omega, Real dt );
  static Quaternion calcRotationQuaternion                    ( Vector3 const& normal, Real angle ); // vector should be normalized
  static Quaternion calcRotationFromVectorToVector            ( Vector3 const& from, Vector3 const& to ); // both vectors should be normalized

  void rotateInPlane( Vector3 const& normal, Real angle );

  Vector3 rotateVector3       ( Vector3 const& v ) const; // v can be any vector (even not normalized)
  Vector3 rotateInverseVector3( Vector3 const& v ) const;

  Vector3 rotateVector3_byRotationQuaternion_usingMatrix( Vector3 const& v ) const;

  void rotateByRotationQuaternion( Quaternion const& rotation_quaternion ); // both quaternions should be normalized
  void rotateByAngularSpeedVector( Vector3 const& angular_speed, Real dt );

  static void rotateCameraVectors( Vector3 const & normal, Real angle, Vector3 & pos, Vector3 & view, Vector3 & up );

  // operators
  Quaternion & operator += ( Quaternion const & q );
  Quaternion & operator -= ( Quaternion const & q );
  Quaternion & operator *= ( Real d );
  Quaternion & operator /= ( Real d );

  // unary operator
  Quaternion operator - () const;

  // binary operators
  Quaternion operator + ( Quaternion const & q ) const;
  Quaternion operator - ( Quaternion const & q ) const;
  Quaternion operator * ( Real d ) const;
  Quaternion operator / ( Real d ) const;
  
  //
  Real       dotProduct   ( Quaternion const & q ) const;
  Quaternion crossProduct ( Quaternion const & q ) const;

  void pow( Real power );

  // Euler angles
  void fromEulerAnglesDeg ( Real  roll, Real  pitch, Real  yaw );
  void fromEulerAngles    ( Real  roll, Real  pitch, Real  yaw );
  void toEulerAngles      ( Real& roll, Real& pitch, Real& yaw ) const;

  //
  static Quaternion linearInterpolation   ( Quaternion const& q1, Quaternion const& q2, Real ratio );
  static Quaternion sphericalInterpolation( Quaternion const& q1, Quaternion const& q2, Real ratio, int spin_count = 0 );

  // matrix
  Matrix toRotationMatrix() const;
  void toRotationMatrix( Matrix & mat ) const;
  static Quaternion fromRotationMatrix( Matrix const& mat );

  // imu
  Quaternion calcJacobian( Vector3 const& acc ) const; // acc - acceleration

  void updateQuaternionUsingRungeKutta ( Vector3 const& gyro, Real dt );
  void updateQuaternionUsingRungeKutta2( Vector3 const& gyro, Quaternion const& estimated_gyro_error_direction, Real dt );

  static Quaternion accelToQuaternion( Vector3 accel );

private:
  void normalizeSqr();

  Real calcRungeKuttaDeltas( Vector3 const& gyro, Real dt, Vector3& dv ) const; // returns ds
};

} // namespace zygo
