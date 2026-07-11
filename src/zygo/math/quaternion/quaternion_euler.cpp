#include "quaternion.h"
#include <zygo/math/common/scalar.h>


namespace zygo {

void Quaternion::fromEulerAnglesDeg( Real roll, Real pitch, Real yaw )
{
  fromEulerAngles( DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw) );
}

void Quaternion::fromEulerAngles( Real roll, Real pitch, Real yaw )
{
  const Real COEFF = REAL_HALF;
  
  //if ( roll  > 180.0 ) roll  -= 360.0;
  //if ( pitch > 180.0 ) pitch -= 360.0;
  //if ( yaw   > 180.0 ) yaw   -= 360.0;

  roll  *= COEFF;
  pitch *= COEFF;
  yaw   *= COEFF;

  // First , rotate around Y axis (roll).
  // Second, rotate around X axis (pitch)
  // Third , rotate around Z axis (yaw)

  Real cos_x = std::cos( pitch );
  Real cos_y = std::cos( roll  );
  Real cos_z = std::cos( yaw   );

  Real sin_x = std::sin( pitch );
  Real sin_y = std::sin( roll  );
  Real sin_z = std::sin( yaw   );

  s = cos_x * cos_y * cos_z + sin_x * sin_y * sin_z;

  v.setX( sin_x * cos_y * cos_z + cos_x * sin_y * sin_z );
  v.setY( cos_x * sin_y * cos_z - sin_x * cos_y * sin_z );
  v.setZ( cos_x * cos_y * sin_z - sin_x * sin_y * cos_z );

/*
  // Inav:
  v.setX( cos_x * sin_y * cos_z - sin_x * cos_y * sin_z ); // the same, as my version, but v.x and v.y are swapped
  v.setY( sin_x * cos_y * cos_z + cos_x * sin_y * sin_z );
  v.setZ( cos_x * cos_y * sin_z - sin_x * sin_y * cos_z );
*/
}

void Quaternion::toEulerAngles( Real & roll, Real & pitch, Real & yaw ) const
{
  Real x = v.x;
  Real y = v.y;
  Real z = v.z;

  Real s2 = s * s;
  Real x2 = x * x;
  Real y2 = y * y;
  Real z2 = z * z;

  Real t0 = REAL_TWO * (s * y + x * z);
  Real t1 = s2 + x2 - z2 - y2;
  roll = std::atan2( t0, t1 );

  Real t2 = REAL_TWO * (s * x + z * y);
  Real t3 = s2 - x2 - z2 + y2;
  pitch = std::atan2( t2, t3 );

  Real t4 = REAL_TWO * (s * z - x * y);
  toRange( t4, -REAL_ONE, REAL_ONE );
  yaw = std::asin( t4 );
}

} // namespace zygo
