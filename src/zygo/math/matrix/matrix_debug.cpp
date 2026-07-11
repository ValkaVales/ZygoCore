#include "matrix.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {

#ifdef DEBUG_MATRIX
void Matrix::setFromVector( std::vector<Real> const& vv )
{
  ZgAssert( size == (int)vv.size() );

  int i = 0;
  for ( std::vector<Real>::const_iterator it = vv.begin(); it != vv.end(); ++it )
  {
    m[i++] = *it;
  }
}

Real Matrix::get( int j, int i ) const
{
  ZgAssert( j >= 0 && j < dimy );
  ZgAssert( i >= 0 && i < dimx );

  int idx = j * dimx + i;
  return m[idx];
}

void Matrix::setAt( int j, int i, Real value )
{
  ZgAssert( j >= 0 && j < dimy );
  ZgAssert( i >= 0 && i < dimx );

  m[j * dimx + i] = value;
}

void Matrix::addAt( int j, int i, Real delta )
{
  ZgAssert( j >= 0 && j < dimy );
  ZgAssert( i >= 0 && i < dimx );

  m[j * dimx + i] += delta;
}

Real Matrix::get( int idx ) const
{
  ZgAssert( idx >= 0 && idx < size );
  return m[idx];
}

void Matrix::setAt( int idx, Real value )
{
  ZgAssert( idx >= 0 && idx < size );
  m[idx] = value;
}

void Matrix::addAt( int idx, Real delta )
{
  ZgAssert( idx >= 0 && idx < size );
  m[idx] += delta;
}
#endif


} // namespace zygo
