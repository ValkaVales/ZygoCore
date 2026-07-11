#include "matrix.h"
#include <zygo/core/assert.h>
#include <zygo/random/random.h> // not very good dependency


namespace zygo {

void Matrix::makeAllIntegerRandom( int from, int to_inclusive )
{
  ZgAssert( to_inclusive > from );
  Real* p = ptr();

  for ( int i = 0; i < size; ++i )
    *p++ = (Real)(from + (int)Random::randInt( (u32)(to_inclusive - from + 1) ));
}

void Matrix::makeAllLinearRandom( Real from, Real to )
{
  Real* p = ptr();

  for ( int i = 0; i < size; ++i )
    *p++ = Random::rand( from, to );
}

void Matrix::makeAllGaussRandom( Real mean, Real sigma )
{
  Real* p = ptr();

  for ( int i = 0; i < size; ++i )
    *p++ = Random::gauss( mean, sigma );
}

} // namespace zygo
