#include "matrix.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>

// Only for SPD (Symmetric Positive Definite) matrix


namespace zygo {

#ifdef USE_CHOLESKY

// Cholesky decomposition: A = L * L^T, where L is lower triangular.
// Works ONLY for symmetric positive definite (SPD) matrices;
// Reads only the lower triangle of A (symmetry is assumed).
//
// Returns false if the matrix is ​​not SPD (the square root is <= eps).
// Incidentally, this is the cheapest practical test for positive definiteness.
//
// Pivot selection is not necessary: ​​for SPD matrices, the method is inherently stable.
//
// NOTE: Matrix symmetry is validated only in DEBUG builds.
// In release builds, symmetry is a caller precondition and the check is intentionally omitted to avoid its runtime cost.
// Positive-definiteness is still validated by the Cholesky decomposition itself.
bool Matrix::tryCholesky( Matrix& L, Real eps ) const
{
  ZgAssert( dimx == dimy );
  ZgAssert( L.dimx == dimx && L.dimy == dimy );
  ZgAssert( isSymmetric() );

  const int n = dimx;
  L.makeAllZero();

  for ( int j = 0; j < n; ++j )
  {
    Real const* lj = L[j];

    // Diagonal: L[j][j] = sqrt( A[j][j] - sum_{k<j} L[j][k]^2 )
    Real d = get( j, j );
    for ( int k = 0; k < j; ++k )
      d -= sqr( lj[k] );

    if ( d <= eps )
      return false; // not positive definite

    Real const ljj     = std::sqrt( d );
    Real const inv_ljj = REAL_ONE / ljj;
    L.setAt( j, j, ljj );

    // Column below the diagonal:
    // L[i][j] = ( A[i][j] - sum_{k<j} L[i][k]*L[j][k] ) / L[j][j]
    for ( int i = j + 1; i < n; ++i )
    {
      Real const* li = L[i];

      Real s = get( i, j );
      for ( int k = 0; k < j; ++k )
        s -= li[k] * lj[k];

      L.setAt( i, j, s * inv_ljj );
    }
  }

  return true;
}

// Inverting an SPD matrix using Cholesky. Approximately twice as fast as the universal tryInverse (benchmarked: 1.7-2.3x on n=25..400).
//
// Scheme: A = L*L^T; M = L^-1; A^-1 = M^T * M.
//
// IMPORTANT NOTE about the implementation: all inner loops run row-by-row sequentially.
// The brute-force version with column-by-column passes computes the same flops, but loses all the gains on cache misses and is no faster than the Gauss-Jordan algorithm.
bool Matrix::tryInverseSPD_Cholesky( Matrix& res, Real eps ) const
{
  ZgAssert( dimx == dimy );
  ZgAssert( res.dimx == dimx && res.dimy == dimy );

  const int n = dimx;

  Matrix L( n, n );
  if ( !tryCholesky( L, eps ) )
    return false;

  // 1. M = L^-1 row by row (M is also lower triangular).
  //    Row i: M[i][:] = ( e_i - sum_{k<i} L[i][k] * M[k][:] ) / L[i][i]
  //    Internal operation - appending a row to a row.
  Matrix M( n, n );
  M.makeAllZero();

  for ( int i = 0; i < n; ++i )
  {
    Real*       mi = M[i];
    Real const* li = L[i];

    for ( int k = 0; k < i; ++k )
    {
      Real const lik = li[k];
      if ( lik == REAL_ZERO )
        continue;

      Real const* mk = M[k];
      for ( int t = 0; t <= k; ++t )    // строка k нулевая правее k
        mi[t] -= lik * mk[t];
    }

    Real const inv_lii = REAL_ONE / li[i];
    for ( int t = 0; t < i; ++t )
      mi[t] *= inv_lii;
    mi[i] = inv_lii;
  }

  // 2. A^-1 = M^T * M as the sum of the outer products of the rows of M with themselves:
  //    row k (non-zero at 0..k) adds M[k][i]*M[k][j] to the upper triangle of the result.
  res.makeAllZero();

  for ( int k = 0; k < n; ++k )
  {
    Real const* mk = M[k];

    for ( int i = 0; i <= k; ++i )
    {
      Real const v = mk[i];
      if ( v == REAL_ZERO )
        continue;

      Real* ri = res[i];
      for ( int j = i; j <= k; ++j )
        ri[j] += v * mk[j];
    }
  }

  // 3. A^-1 is symmetrical: mirror the upper triangle into the lower one
  for ( int j = 1; j < n; ++j )
    for ( int i = 0; i < j; ++i )
      res.setAt( j, i, res.get( i, j ) );

  return true;
}


// Solution with an ALREADY READY factorization of L (A = L*L^T): O(n^2) on the right-hand side.
// If the same matrix A is solved with many b's arriving at different times, factor it once, and then use only this method.
void Matrix::solveWithCholesky( Matrix const& L, Matrix const& b, Matrix& x )
{
  const int n = L.dimy;
  const int k = b.dimx;

  ZgAssert( L.dimx == n );
  ZgAssert( b.dimy == n && x.dimy == n && x.dimx == k );

  x = b; // we work on place: b -> y -> x

  // --- Direct substitution L * Y = B, row by row:
  //     y_i = ( b_i - sum_{j<i} L[i][j] * y_j ) / L[i][i]
  for ( int i = 0; i < n; ++i )
  {
    Real*       xi = x[i];
    Real const* li = L[i];

    for ( int j = 0; j < i; ++j )
    {
      Real const lij = li[j];
      if ( lij == REAL_ZERO )
        continue;

      Real const* xj = x[j];
      for ( int t = 0; t < k; ++t )   // continuous AXPY on all right-hand sides
        xi[t] -= lij * xj[t];
    }

    Real const inv_lii = REAL_ONE / li[i];
    for ( int t = 0; t < k; ++t )
      xi[t] *= inv_lii;
  }

  // --- Reverse substitution L^T * X = Y, bottom up:
  //     x_i = ( y_i - sum_{j>i} L[j][i] * x_j ) / L[i][i]
  for ( int i = n - 1; i >= 0; --i )
  {
    Real* xi = x[i];

    for ( int j = i + 1; j < n; ++j )
    {
      Real const lji = L.get( j, i );
      if ( lji == REAL_ZERO )
        continue;

      Real const* xj = x[j];
      for ( int t = 0; t < k; ++t )
        xi[t] -= lji * xj[t];
    }

    Real const inv_lii = REAL_ONE / L.get( i, i );
    for ( int t = 0; t < k; ++t )
      xi[t] *= inv_lii;
  }
}


// Solve A*x = b for an SPD matrix A and k right-hand sides (columns of b).
// The right-hand sides are processed ALL AT ONCE: the internal operation is a row-wise AXPY over contiguous memory (row x contains all k right-hand sides in a row).
// The columnar version (one right-hand side at a time) loses out on large k due to strided access – see the benchmark in test_spd_cholesky.cpp.
bool Matrix::trySolveSPD_Cholesky( Matrix const& b, Matrix& x, Real eps ) const
{
  ZgAssert( dimx == dimy );
  ZgAssert( b.dimy == dimy );
  ZgAssert( x.dimy == dimx && x.dimx == b.dimx );

  Matrix L( dimy, dimx );

  if ( !tryCholesky( L, eps ) )
    return false;

  solveWithCholesky( L, b, x );
  return true;
}

Matrix Matrix::inverseSPD_Cholesky( Real eps ) const
{
  Matrix res( dimy, dimx );

  const bool ok = tryInverseSPD_Cholesky( res, eps );
  ZgAssertRelease( ok );

  return res;
}

#endif // USE_CHOLESKY

} // namespace zygo
