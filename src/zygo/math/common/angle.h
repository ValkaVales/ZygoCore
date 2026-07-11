#pragma once

#include <zygo/core/types.h>
#include "consts.h"


namespace zygo {

template <typename T>
inline bool checkAngle180( T angle, T e = EPSILON )
{
  return ge( angle, -PI ) && le( angle, PI );
}

Real normalizeAnglePi( Real angle_rad );
Real angleDelta( Real from, Real to );
Real angleDistance( Real a, Real b );

} // namespace zygo
