#include "matrix.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {

Matrix Matrix::operator*( Matrix const& other ) const
{
  ZgAssert( dimx == other.dimy );

#ifdef _OPENMP
  if ( shouldUseMultiThreading_inMultiplications( other.dimx ) )
    return multiply_parallel( other );
#endif // _OPENMP

  return multiply_singleThread( other );
}

Matrix Matrix::multiply_singleThread( Matrix const& other ) const
{
  ZgAssert( dimx == other.dimy );

  //if ( other.dimx == 1 )
  //  return multiplyByColumnMatrix( other );

  Matrix res( dimy, other.dimx );

  Real const* p1_0 = ptr();
  Real const* p2_0 = other.ptr();
  Real* p0 = res.ptr();

  int j = dimy;
  while ( j-- )
  {
    Real const* p2_ = p2_0;

    int i = other.dimx;
    while ( i-- )
    {
      Real const* p1 = p1_0;
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

    p1_0 += dimx;
  }

  return res;
}

Vector3 Matrix::multiplyByVector3( Vector3 const& v ) const
{
  ZgAssert( dimx == 3 );
  ZgAssert( dimy == 3 );

  Vector3 res;
  const int j_cnt = dimy;

  for ( int j = 0; j < j_cnt; ++j )
  {
    int shift = j * dimx;

    Real s = REAL_ZERO;
    int k = dimx;
    while ( k-- )
    {
      s += m[shift + k] * v.getElem( k );
    }

    res.setElem( j, s );
  }

  return res;
}


void Matrix::multiplyByColumnMatrix_singleThread( Matrix const& other, Matrix& res ) const
{
  ZgAssert( &res != &other );
  ZgAssert( &res != this );

  ZgAssert( dimx == other.dimy );
  ZgAssert( other.dimx == 1 );
  ZgAssert( res.dimx == 1 );
  ZgAssert( res.dimy >= dimy );

  Real const* p1_0 = ptr();
  Real const* p2_0 = other.ptr();
  Real* p0 = res.ptr();

  int j = dimy;
  while ( j-- )
  {
    Real const* p1 = p1_0;// +j * dimx;
    Real const* p2 = p2_0;

    Real s = REAL_ZERO;
    int k = dimx;
    while ( k-- )
    {
      Real a = (*p1++);
      Real b = (*p2++);
      Real ab = a * b;
      s += ab;
    }

    *p0++ = s;

    p1_0 += dimx;
  }
}

void Matrix::multiplyByColumnMatrix_AddToRes_singleThread( Matrix const& other, Matrix& res ) const
{
  ZgAssert( &res != &other );
  ZgAssert( &res != this );

  ZgAssert( dimx == other.dimy );
  ZgAssert( other.dimx == 1 );
  ZgAssert( res.dimx == 1 );
  ZgAssert( res.dimy >= dimy );

  Real const* p1_0 = ptr();
  Real const* p2_0 = other.ptr();
  Real* p0 = res.ptr();

  int j = dimy;
  while ( j-- )
  {
    Real const* p1 = p1_0;// +j * dimx;
    Real const* p2 = p2_0;

    Real s = REAL_ZERO;
    int k = dimx;
    while ( k-- )
    {
      Real a = (*p1++);
      Real b = (*p2++);
      Real ab = a * b;
      s += ab;
    }

    *p0++ += s; // Add to res

    p1_0 += dimx;
  }
}

} // namespace zygo
