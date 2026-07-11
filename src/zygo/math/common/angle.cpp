#include "angle.h"
#include "consts.h"
#include <zygo/core/assert.h>
#include <cmath>


namespace zygo {

Real normalizeAnglePi( Real angle )
{
  angle = std::fmod( angle + PI, PI_MUL_2 );

  if ( angle < 0.0 )
    angle += PI_MUL_2;

  angle -= PI;
/*
  if ( angle_rad > PI )
    angle_rad -= PI_MUL_2;
  else
  if ( angle_rad < -PI )
    angle_rad += PI_MUL_2;
*/
  ZgAssert( angle <=  PI );
  ZgAssert( angle >= -PI );

  return angle;

}

// Кратчайшая разница между углами.
// Результат в диапазоне [-pi; +pi]
Real angleDelta( Real from, Real to )
{
  return normalizeAnglePi( to - from );
}

// Расстояние между углами.
// Результат в диапазоне [0; pi]
Real angleDistance( Real a, Real b )
{
  return std::abs( angleDelta( a, b ) );
}

} // namespace zygo
