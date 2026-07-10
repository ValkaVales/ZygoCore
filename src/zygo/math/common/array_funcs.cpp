#include "array_funcs.h"
#include <zygo/core/assert.h>
#include <cmath> // std::exp


namespace zygo {

Real calcSquaresSum( Real const * arr, int size ) noexcept // was calcSqrsSum
{
  Real sum = REAL_ZERO;

  for ( int i = 0; i < size; ++i )
    sum += arr[i] * arr[i];

  return sum;
}

int argMax( Real const* arr, int size ) noexcept // was maxValueIndex
{
  ZgAssert( arr != nullptr );
  ZgAssert( size > 0 );

  int best = 0;
  Real best_value = arr[0];

  for ( int i = 1; i < size; ++i )
  {
    if ( arr[i] > best_value )
    {
      best_value = arr[i];
      best = i;
    }
  }

  return best;
}

std::vector<Real> calcSoftMax( Real const * arr, int size )
{
  ZgAssert( arr != nullptr );
  ZgAssert( size > 0 );

  std::vector<Real> res( (size_t)size );

  int max_item_idx = argMax( arr, size );
  Real max_v = arr[max_item_idx];

  Real sum = (Real)0;

  for ( int i = 0; i < size; ++i )
  {
    Real e = std::exp( arr[i] - max_v );
    res[i] = e;
    sum += e;
  }

  ZgAssert( sum >= REAL_ONE );

  // sum >= 1 always: the max element contributes exp(0) == 1,
  // so division by zero is impossible by construction
  for ( int i = 0; i < size; ++i )
    res[i] /= sum;

  return res;
}

} // namespace zygo
