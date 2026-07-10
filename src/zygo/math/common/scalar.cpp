#include "scalar.h"


namespace zygo {

Real fractionalPart( Real x )
{
  x -= std::floor( x );

  if ( x < REAL_ZERO )
    x += REAL_ONE;

  return x;
}

Real fractionalPartSigned( Real x )
{
  x = fmod( x + REAL_HALF, REAL_ONE );

  if ( x < REAL_ZERO )
    x += REAL_ONE;

  return x - REAL_HALF;
}

} // namespace zygo
