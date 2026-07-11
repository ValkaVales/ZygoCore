#include "matrix.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>

#ifdef _OPENMP
#include <omp.h>
#endif


namespace zygo {

#ifdef _OPENMP

Matrix Matrix::multiply_parallel( Matrix const & other ) const
{
  ZgAssert( dimx == other.dimy );
  Matrix res( dimy, other.dimx );

  Real const* p1_0 = ptr();
  Real const* p2_0 = other.ptr();

  const int j_cnt = dimy;
#pragma omp parallel for
  for ( int j = 0; j < j_cnt; ++j )
  {
    Real* p0 = res.ptr() + j * other.dimx;

    Real const* p1_ = p1_0 + j * dimx;
    Real const* p2_ = p2_0;

    int i = other.dimx;
    while ( i-- )
    {
      Real const* p1 = p1_;
      Real const* p2 = p2_;

      Real s = REAL_ZERO;
      int k = dimx;
      while ( k-- )
      {
        //s += get( j, k ) * other.get( k, i );
        s += (*p1++) * (*p2);
        p2 += other.dimx;
      }

      //res[j][i] = s;
      *p0++ = s;
      ++p2_;
    }

    //p1_0 += dimx;
  }

  return res;
}

void Matrix::multiplyByColumnMatrix_parallel( Matrix const& other, Matrix& res ) const
{
  ZgAssert( &res != &other );
  ZgAssert( &res != this );

  ZgAssert( dimx == other.dimy );
  ZgAssert( other.dimx == 1 );
  ZgAssert( res.dimx == 1 );
  ZgAssert( res.dimy >= dimy );

  const int j_cnt = dimy;

#pragma omp parallel for
  for ( int j = 0; j < j_cnt; ++j )
  {
    Real const* p1 = ptr() + j * dimx;
    Real const* p2 = other.ptr();

    Real s = REAL_ZERO;
    int k = dimx;
    while ( k-- )
      s += *(p1++) * *(p2++);

    res.m[j] = s;
  }
}

void Matrix::multiplyByColumnMatrix_AddToRes_parallel( Matrix const& other, Matrix& res ) const
{
  ZgAssert( &res != &other );
  ZgAssert( &res != this );

  ZgAssert( dimx == other.dimy );
  ZgAssert( other.dimx == 1 );
  ZgAssert( res.dimx == 1 );
  ZgAssert( res.dimy >= dimy );

  const int j_cnt = dimy;

#pragma omp parallel for
  for ( int j = 0; j < j_cnt; ++j )
  {
    Real const* p1 = ptr() + j * dimx;
    Real const* p2 = other.ptr();

    Real s = REAL_ZERO;
    int k = dimx;
    while ( k-- )
      s += *(p1++) * *(p2++);

    res.m[j] += s; // Add to res
  }
}

#endif // _OPENMP

} // namespace zygo
