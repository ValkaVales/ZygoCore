#include "quaternion.h"
#include <zygo/core/assert.h>
#include <cmath>


namespace zygo {

Quaternion Quaternion::linearInterpolation( Quaternion const & q1, Quaternion const & q2, Real ratio )
{
  ZgAssert( ratio >= REAL_ZERO && ratio <= REAL_ONE );

  Real ratio1 = REAL_ONE - ratio;
  Quaternion res = (q1 * ratio1) + (q2 * ratio);
  return res;
}


Quaternion Quaternion::sphericalInterpolation( Quaternion const& q1, Quaternion const& q2_, Real ratio, int spin_count ) // , bool use_short_rotation
{
  ZgAssert( ratio >= REAL_ZERO && ratio <= REAL_ONE );

  Real cosa = q1.dotProduct( q2_ ); // fast version

  // If cosa < 0, the interpolation will take the long way around the sphere.
  // Make sure we use the short rotation.
  Quaternion q2 = q2_;
  if ( cosa < REAL_ZERO ) // && use_short_rotation
  {
    q2   = -q2;
    cosa = -cosa;
  }

  // Perform a linear interpolation when cosa is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
  if ( cosa > (REAL_ONE - QUATERNION_SPHERICAL_INTERPOLATION_THRESHOLD_EPSILON) )
    return linearInterpolation( q1, q2, ratio );

  // Spherical interpolation (Essential Mathematics, page 467. Or Graphics Gems III, page 96)
  Real angle    = std::acos( cosa );
  Real phi      = angle + PI * spin_count;
  Real inv_sina = REAL_ONE / std::sin( angle ); // sina may be calculated from cosa (but it will be slower)

  Real scale      = std::sin( angle - phi * ratio ) * inv_sina;
  Real inv_scale  = std::sin(         phi * ratio ) * inv_sina;

  Quaternion res = (q1 * scale) + (q2 * inv_scale);
  return res;
}

} // namespace zygo
