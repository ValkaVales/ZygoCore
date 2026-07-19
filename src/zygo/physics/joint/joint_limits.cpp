#include "joint_limits.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <zygo/math/common/angle.h>


namespace zygo {
namespace phys {

// --------------------------------------------------------------------------------- JointLimits
JointLimits::JointLimits()
  : min_angle_rad ( -PI )
  , max_angle_rad (  PI )
{
}

JointLimits::JointLimits( double min_angle_deg, double max_angle_deg )
  : min_angle_rad ( DEG2RAD( min_angle_deg ) )
  , max_angle_rad ( DEG2RAD( max_angle_deg ) )
{
  ZgAssert( isGreater( max_angle_rad, min_angle_rad ) );
  ZgAssert( checkAngle180( min_angle_rad ) ); // angle >= -PI && angle <= PI
  ZgAssert( checkAngle180( max_angle_rad ) ); // angle >= -PI && angle <= PI
}

bool JointLimits::insideLimits( double angle ) const
{
  return
       ge( angle, min_angle_rad )   // greater or equal
    && le( angle, max_angle_rad );  // less or equal
}

double JointLimits::angleError( double angle ) const
{
  if ( insideLimits( angle ) )
    return 0.0;

  double d1 = angleDistance( min_angle_rad, angle );
  double d2 = angleDistance( max_angle_rad, angle );

  return min2( d1, d2 );
}

// --------------------------------------------------------------------------------- JointAngles
JointAngles::JointAngles()
  : q0 ( 0.0 )
  , q1 ( 0.0 )
  , q2 ( 0.0 )
{
}

JointAngles::JointAngles( double q0_rad, double q1_rad, double q2_rad )
  : q0 ( q0_rad )
  , q1 ( q1_rad )
  , q2 ( q2_rad )
{
  ZgAssert( checkAngle180( q0 ) );
  ZgAssert( checkAngle180( q1 ) );
  ZgAssert( checkAngle180( q2 ) );
}

JointAngles JointAngles::fromDeg( double q0_deg, double q1_deg, double q2_deg )
{
  return fromRad( DEG2RAD(q0_deg), DEG2RAD(q1_deg), DEG2RAD(q2_deg) );
}

JointAngles JointAngles::fromRad( double q0_rad, double q1_rad, double q2_rad )
{
  return JointAngles( q0_rad, q1_rad, q2_rad );
}

} // namespace phys
} // namespace zygo
