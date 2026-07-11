#pragma once

#include <zygo/math/vector/vec3.h>

#ifdef _DEBUG
#define DEBUG_MATRIX
#endif

#ifdef DEBUG_MATRIX
#include <vector>
#endif


namespace zygo {

#define MATRIX_EXTRA_RELEASE_ASSERTS
#define USE_CHOLESKY


class Matrix
{
private:
  int dimy;
  int dimx;
  int size;

#ifdef DEBUG_MATRIX
  std::vector<Real> m;
#else
  Real * m;
#endif

#ifdef DEBUG_MATRIX
  char const * name; // for DEBUG only
#endif

public:
  Matrix( int dimy, int dimx, char const * name = NULL );
  Matrix( Matrix const & other );
  ~Matrix();

  Matrix( Matrix&& other ) noexcept;
  Matrix& operator=( Matrix&& other ) noexcept;

  void swap( Matrix& other ) noexcept;
  void nullify();

  void copyFrom( Matrix const& mat );

  void makeAllZero();
  void makeIdentity();
  void makeAllValue( Real v );

  void makeAllIntegerRandom( int from, int to_inclusive );
  void makeAllLinearRandom( Real from, Real to );
  void makeAllGaussRandom ( Real mean, Real sigma );

  bool isEqual( Matrix const & other, Real eps = EPSILON ) const;

#ifdef DEBUG_MATRIX
  char const* getName() const { return name; }
#endif

  Real calcFirstSqrSum( uint first_cnt ) const;
  Real calcFirstAbsSum( uint first_cnt ) const;
  Real calcAllSqrSum() const { return calcFirstSqrSum( size ); }
  Real calcAllAbsSum() const { return calcFirstAbsSum( size ); }
  void normalizeFirst_byValue( Real value, uint first_cnt );
  void normalizeAll_byValue  ( Real value ) { normalizeFirst_byValue( value, size ); }

  inline int dimX() const { return dimx; }
  inline int dimY() const { return dimy; }
  inline int getSize() const { return size; }

  inline void set1( Real v0                                                                          ) { m[0] = v0; }
  inline void set2( Real v0, Real v1                                                                 ) { m[0] = v0; m[1] = v1; }
  inline void set3( Real v0, Real v1, Real v2                                                        ) { m[0] = v0; m[1] = v1; m[2] = v2; }
  inline void set4( Real v0, Real v1, Real v2, Real v3                                               ) { m[0] = v0; m[1] = v1; m[2] = v2; m[3] = v3; }
  inline void set5( Real v0, Real v1, Real v2, Real v3, Real v4                                      ) { m[0] = v0; m[1] = v1; m[2] = v2; m[3] = v3; m[4] = v4; }
  inline void set6( Real v0, Real v1, Real v2, Real v3, Real v4, Real v5                             ) { m[0] = v0; m[1] = v1; m[2] = v2; m[3] = v3; m[4] = v4; m[5] = v5; }
  //       no set7()
  inline void set8( Real v0, Real v1, Real v2, Real v3, Real v4, Real v5, Real v6, Real v7           ) { m[0] = v0; m[1] = v1; m[2] = v2; m[3] = v3; m[4] = v4; m[5] = v5; m[6] = v6; m[7] = v7; }
  inline void set9( Real v0, Real v1, Real v2, Real v3, Real v4, Real v5, Real v6, Real v7, Real v8  ) { m[0] = v0; m[1] = v1; m[2] = v2; m[3] = v3; m[4] = v4; m[5] = v5; m[6] = v6; m[7] = v7; m[8] = v8; }

#ifdef DEBUG_MATRIX
  void setFromVector( std::vector<Real> const& vv );
#endif
  void setPrefixFromArray( Real const * arr, int len );

  Real const* getArr() const  { return ptr(); }
  Real      * getArr()        { return ptr(); }

  //
  Real*       operator[]( int j );
  Real const* operator[]( int j ) const;

#ifdef DEBUG_MATRIX
  Real  get( int j, int i ) const;
  Real  get( int idx ) const;

  void  setAt( int j, int i, Real value );
  void  addAt( int j, int i, Real delta );
  void  setAt( int idx, Real value );
  void  addAt( int idx, Real delta );
#else
  inline Real get( int j, int i ) const { return m[j * dimx + i]; }
  inline Real get( int idx      ) const { return m[idx]; }

  inline void   setAt( int j, int i, Real value ) { m[j * dimx + i] = value; }
  inline void   addAt( int j, int i, Real delta ) { m[j * dimx + i] += delta; }
  inline void   setAt( int idx     , Real value ) { m[idx] = value; }
  inline void   addAt( int idx     , Real delta ) { m[idx] += delta; }
#endif

  void setCol( int col_idx, Vector3 const& v );
  void setRow( int row_idx, Vector3 const& v );

  Matrix& operator= ( Matrix const& other );

  bool operator==( Matrix const& other ) const;
  bool operator!=( Matrix const& other ) const;

  void operator+=( Matrix const & other );
  void operator-=( Matrix const & other );

  void multiplyElementwise( Matrix const & other );

  void operator*=( Real d );
  void operator/=( Real d );

  Matrix operator+( Matrix const & other ) const;
  Matrix operator-( Matrix const & other ) const;
  Matrix operator*( Matrix const & other ) const; // optimal: multi-threaded may be used
  Matrix operator*( Real v ) const;
  Matrix operator/( Real v ) const;

  Matrix multiply_parallel    ( Matrix const& other ) const;
  Matrix multiply_singleThread( Matrix const& other ) const;

  Vector3 multiplyByVector3( Vector3 const & v ) const;

  void multiplyByColumnMatrix_singleThread          ( Matrix const& other, Matrix& res ) const;
  void multiplyByColumnMatrix_parallel              ( Matrix const& other, Matrix& res ) const;

  void multiplyByColumnMatrix_AddToRes_singleThread ( Matrix const& other, Matrix& res ) const;
  void multiplyByColumnMatrix_AddToRes_parallel     ( Matrix const& other, Matrix& res ) const;

  //inline bool shouldUseMultiThreading_inMultiplications() const { return dimx * dimy > 5000; } // heuristic
  inline bool shouldUseMultiThreading_inMultiplications( int other_dimx ) const { return (long long)dimx * (long long)dimy * (long long)other_dimx > 300000LL; } // heuristic

  // unary operators
  inline Matrix operator~() const { return transpose(); }
  Matrix transpose() const;

  //
  void swapRows( int r1, int r2 );

  bool isSymmetric( Real eps = MATRIX_EPSILON ) const;
  bool isSPD      ( Real eps = MATRIX_EPSILON ) const;

  // Maximum element-wise discrepancy with another matrix of the same dimension.
  // Useful for testing: comparison with a reference matrix using a single number.
  Real maxAbsDiff( Matrix const& other ) const;

#ifdef USE_CHOLESKY
  bool tryCholesky            ( Matrix& L                 , Real eps = MATRIX_EPSILON ) const;
  bool trySolveSPD_Cholesky   ( Matrix const& b, Matrix& x, Real eps = MATRIX_EPSILON ) const;
  bool tryInverseSPD_Cholesky ( Matrix& res               , Real eps = MATRIX_EPSILON ) const;

  static void solveWithCholesky( Matrix const& L, Matrix const& b, Matrix& x );
  Matrix inverseSPD_Cholesky  ( Real eps = MATRIX_EPSILON ) const;
#endif

  // inverse
  Matrix inverse1x1() const;
  Matrix inverse2x2() const;
  Matrix inverse3x3() const;

  bool   tryInverse( Matrix& res, Real epsilon = MATRIX_EPSILON ) const;
  Matrix inverse   ( Real epsilon = MATRIX_EPSILON ) const;

  Vector3 solve3x3( Vector3 const& b ) const;

  //
  Matrix subMatrix   ( int col_indices[], int row_indices[], int columns_count, int rows_count );
  Matrix expandMatrix( int col_indices[], int row_indices[], int new_dimy, int new_dimx ); // rows_count should be equal to dimx, columns_count should be equal to dimy

  //
  static Matrix makeSkewMatrixFromVector( Vector3 const& v );
  static Matrix makeOuterProductMatrix( Vector3 const& a, Vector3 const& b );

private:
#ifdef DEBUG_MATRIX
  inline Real*       ptr()       { return m.data(); }
  inline Real const* ptr() const { return m.data(); }
#else
  inline Real*       ptr()       { return m; }
  inline Real const* ptr() const { return m; }
#endif
};

} // namespace zygo
