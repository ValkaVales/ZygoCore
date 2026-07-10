#pragma once

#include <zygo/core/types.h>
#include "consts.h"


namespace zygo {

template <typename T>
inline bool checkAngle180( T angle, T e = EPSILON )
{
  return ge( angle, -PI ) && le( angle, PI );
}

double normalizeAnglePi( Real angle_rad );
double angleDelta( double from, double to );
double angleDistance( double a, double b );

} // namespace zygo
