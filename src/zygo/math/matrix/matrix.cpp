#include "matrix.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <utility> // std::move, swap


namespace zygo {

Matrix::Matrix( int dimy, int dimx, char const * name )
  : dimy  ( dimy )
  , dimx  ( dimx )
  , size  ( dimx * dimy )
#ifdef DEBUG_MATRIX
  , name  ( name )
#else
  , m     ( NULL )
#endif
{
  ZgAssert( dimx >= 1 && dimy >= 1 );

#ifdef DEBUG_MATRIX
  m.assign( size, REAL_ZERO );
#else
  m = new Real[size];
  (void)name;
#endif
}

Matrix::Matrix( Matrix const& other )
  : dimy  ( other.dimy )
  , dimx  ( other.dimx )
  , size  ( other.size )
#ifdef DEBUG_MATRIX
  , name  ( NULL )
#else
  , m     ( NULL )
#endif
{
#ifdef DEBUG_MATRIX
  m.reserve( size );
  for ( int i = 0; i < size; ++i )
    m.push_back( other.m[i] );
#else
  m = new Real[size];

  Real* p1 = m;
  Real const* p2 = other.m;

  //memcpy( p1, p2, size * sizeof(Real) );

  int k = size;
  while ( k-- )
    *p1++ = *p2++;
#endif
}

Matrix::~Matrix()
{
#ifndef DEBUG_MATRIX
  delete[] m;
#endif
}

Matrix::Matrix( Matrix&& other ) noexcept
  : dimy ( other.dimy )
  , dimx ( other.dimx )
  , size ( other.size )
  , m    ( std::move( other.m ) )
#ifdef DEBUG_MATRIX
  , name ( NULL )
#endif
{
  other.nullify();
}

Matrix& Matrix::operator=( Matrix&& other ) noexcept
{
  if ( this == &other )
    return *this;

#ifndef DEBUG_MATRIX
  delete[] m;
#endif

  m    = std::move( other.m );
  dimy = other.dimy;
  dimx = other.dimx;
  size = other.size;

  other.nullify();

  return *this;
}

void Matrix::nullify()
{
  dimx = 0;
  dimy = 0;
  size = 0;

#ifdef DEBUG_MATRIX
  m.clear();
#else
  m = nullptr;
#endif
}

void Matrix::swap( Matrix& other ) noexcept
{
  std::swap( dimx, other.dimx );
  std::swap( dimy, other.dimy );
  std::swap( size, other.size );
  std::swap( m, other.m );
}

void Matrix::makeAllZero()
{
  Real* p = ptr();

  int k = size;
  while ( k-- )
    *p++ = REAL_ZERO;
}

void Matrix::makeAllValue( Real v )
{
  Real* p = ptr();

  int k = size;
  while ( k-- )
    *p++ = v;
}

void Matrix::makeIdentity()
{
  ZgAssert( dimx == dimy );

  makeAllZero();

  for ( int i = 0; i < dimx; ++i )
    setAt( i, i, REAL_ONE );
}

bool Matrix::isEqual( Matrix const& other, Real eps ) const
{
  if ( dimx != other.dimx || dimy != other.dimy )
    return false;

  Real const* p1 = ptr();
  Real const* p2 = other.ptr();

  int k = size;
  while ( k-- )
  {
    if ( !eq( *p1++, *p2++, eps ) )
      return false;
  }

  return true;
}

void Matrix::copyFrom( Matrix const& mat )
{
  ZgAssert( dimx == mat.dimx );
  ZgAssert( dimy == mat.dimy );

  Real*       p  = ptr();
  Real const* p2 = mat.ptr();

  int k = size;
  while ( k-- )
    *p++ = *p2++;
}

void Matrix::normalizeFirst_byValue( Real value, uint first_cnt )
{
  if ( isZero( value ) )
    return; // no normalization is possible

  applyMax( first_cnt, (uint)size );

  Real coeff = REAL_ONE / value;
  Real* p = ptr();

  int k = first_cnt;
  while ( k-- )
    *p++ *= coeff;
}

Real Matrix::calcFirstSqrSum( uint first_cnt ) const
{
  Real sum = REAL_ZERO;
  applyMax( first_cnt, (uint)size );

  Real const * p = ptr();

  int k = first_cnt;
  while ( k-- )
    sum += sqr( *p++ );

  return sum;
}

Real Matrix::calcFirstAbsSum( uint first_cnt ) const
{
  Real sum = REAL_ZERO;
  applyMax( first_cnt, (uint)size );

  Real const * p = ptr();

  int k = first_cnt;
  while ( k-- )
    sum += std::fabs( *p++ );

  return sum;
}

void Matrix::setPrefixFromArray( Real const* arr, int len )
{
  ZgAssert( size >= len );

  Real* p = ptr();

  while ( len-- > 0 )
    *p++ = *arr++;
}

Matrix& Matrix::operator= ( Matrix const& other )
{
  ZgAssert( dimx == other.dimx && dimy == other.dimy );

  Real*       p1 = ptr();
  Real const* p2 = other.ptr();

  //memcpy( p1, p2, size * sizeof(Real) );
  
  int k = size;
  while ( k-- )
    *p1++ = *p2++;

  return *this;
}

bool Matrix::operator==( Matrix const& other ) const
{
  if ( dimx != other.dimx
    || dimy != other.dimy )
    return false;

  for ( int i = 0; i < size; ++i )
    if ( !eq( m[i], other.m[i], MATRIX_EPSILON ) )
      return false;

  return true;
}

bool Matrix::operator!=( Matrix const& other ) const
{
  return !(*this == other);
}

void Matrix::operator+=( Matrix const & other )
{
  ZgAssert( dimx == other.dimx && dimy == other.dimy );

  Real*       p1 = ptr();
  Real const* p2 = other.ptr();

  int k = size;
  while ( k-- )
    *p1++ += *p2++;
}

void Matrix::operator-=( Matrix const & other )
{
  ZgAssert( dimx == other.dimx && dimy == other.dimy );

  Real*       p1 = ptr();
  Real const* p2 = other.ptr();

  int k = size;
  while ( k-- )
    *p1++ -= *p2++;
}

void Matrix::multiplyElementwise( Matrix const& other )
{
  ZgAssert( dimx == other.dimx && dimy == other.dimy );

  Real*       p1 = ptr();
  Real const* p2 = other.ptr();

  int k = size;
  while ( k-- )
    *p1++ *= *p2++;
}

void Matrix::operator*=( Real d )
{
  Real* p = ptr();

  int k = size;
  while ( k-- )
    *p++ *= d;
}

void Matrix::operator/=( Real d )
{
  ZgAssert( !isZero( d, SMALL_EPSILON ) );

  (*this) *= (REAL_ONE / d);
}

Matrix Matrix::operator+( Matrix const& other ) const
{
  ZgAssert( dimx == other.dimx && dimy == other.dimy );

  Matrix res = *this;
  res += other;
  return res;
}

Matrix Matrix::operator-( Matrix const& other ) const
{
  ZgAssert( dimx == other.dimx && dimy == other.dimy );

  Matrix res = *this;
  res -= other;
  return res;
}

Matrix Matrix::operator*( Real v ) const
{
  Matrix res = *this;
  res *= v;
  return res;
}

Matrix Matrix::operator/( Real v ) const
{
  ZgAssert( !isZero( v, SMALL_EPSILON ) );

  Matrix res = *this;
  res /= v;
  return res;
}

Real* Matrix::operator[]( int j )
{
  ZgAssert( j >= 0 && j < dimy );
  return ptr() + j * dimx;
}

Real const* Matrix::operator[]( int j ) const
{
  ZgAssert( j >= 0 && j < dimy );
  return ptr() + j * dimx;
}


void Matrix::setCol( int col_idx, Vector3 const& v )
{
  ZgAssert( dimy == 3 );
  setAt( 0, col_idx, v.x );
  setAt( 1, col_idx, v.y );
  setAt( 2, col_idx, v.z );
}

void Matrix::setRow( int row_idx, Vector3 const& v )
{
  ZgAssert( dimx == 3 );
  setAt( row_idx, 0, v.x );
  setAt( row_idx, 1, v.y );
  setAt( row_idx, 2, v.z );
}


Matrix Matrix::transpose() const
{
  Matrix res( dimx, dimy );

  // TODO:
  //Real const* p1 = ptr();
  //Real*       p2 = res.ptr();

  for ( int i = 0; i < dimx; ++i )
    for ( int j = 0; j < dimy; ++j )
      res.setAt( i, j, get( j, i ) );

  return res;
}

Matrix Matrix::makeSkewMatrixFromVector( Vector3 const& v )
{
  // [v]_x * a = v x a
  Matrix mat( 3, 3 );

  mat.m[0] =  0.0;  mat.m[1] = -v.z;  mat.m[2] =  v.y;
  mat.m[3] =  v.z;  mat.m[4] =  0.0;  mat.m[5] = -v.x;
  mat.m[6] = -v.y;  mat.m[7] =  v.x;  mat.m[8] =  0.0;

  return mat;
}

Matrix Matrix::makeOuterProductMatrix( Vector3 const& a, Vector3 const& b )
{
  Matrix mat( 3, 3 );
  
  mat.m[0] = a.x * b.x;   mat.m[1] = a.x * b.y;   mat.m[2] = a.x * b.z;
  mat.m[3] = a.y * b.x;   mat.m[4] = a.y * b.y;   mat.m[5] = a.y * b.z;
  mat.m[6] = a.z * b.x;   mat.m[7] = a.z * b.y;   mat.m[8] = a.z * b.z;

  return mat;
}

// -------------------------------------------------------------------------------------------------
void Matrix::swapRows( int r1, int r2 )
{
  ZgAssert( r1 >= 0 && r1 < dimy && r2 >= 0 && r2 < dimy );
  if ( r1 == r2 )
    return;

  Real* p1 = (*this)[r1];
  Real* p2 = (*this)[r2];

  for ( int i = 0; i < dimx; ++i )
    std::swap( p1[i], p2[i] );
}

bool Matrix::isSymmetric( Real eps ) const
{
  if ( dimx != dimy )
    return false;

  for ( int j = 1; j < dimy; ++j )
    for ( int i = 0; i < j; ++i )
    {
      if ( !eq( get( j, i ), get( i, j ), eps ) )
        return false;
    }

  return true;
}

// matrix.h / matrix_spd_cholesky.cpp
// Практический тест на SPD: разложение Холецкого сходится <=> матрица SPD.
// Стоимость n^3/6 - дешевле не бывает для произвольного n.
bool Matrix::isSPD( Real eps ) const
{
  if ( !isSymmetric( (Real)1e-9 ) )
    return false;

  Matrix L( dimy, dimx );
  return tryCholesky( L, eps );
}

Real Matrix::maxAbsDiff( Matrix const& other ) const
{
  ZgAssert( dimx == other.dimx );
  ZgAssert( dimy == other.dimy );

  Real const* p1 = ptr();
  Real const* p2 = other.ptr();

  Real max_diff = REAL_ZERO;

  for ( int k = 0; k < size; ++k )
    updMax( max_diff, std::fabs( p1[k] - p2[k] ) );

  return max_diff;
}

} // namespace zygo
