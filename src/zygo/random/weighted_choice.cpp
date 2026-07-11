#include "weighted_choice.h"
#include "random.h"
#include <zygo/core/assert.h>


namespace zygo
{

int getRandomIndex_BasedOnProbabilities( Real const* weights, int size ) // weightedChoiceIndex
{
  ZgAssert( weights != nullptr );
  ZgAssert( size > 0 );

  Real sum = REAL_ZERO;
  Real const* p = weights;
  int k = size;

  while ( k-- > 0 )
  {
    Real w = *p++;
    ZgAssert( w >= REAL_ZERO );

    sum += w;
  }

  ZgAssert( sum > 0 );

  Real d = Random::rand( REAL_ZERO, sum );

  int res = 0;
  p = weights;

  while ( res < size - 1 )
  {
    d -= *p++;

    if ( d <= REAL_ZERO )
      return res;

    ++res;
  }

  //ZgAssert( res < size );
  return res;
}

} // namespace zygo
