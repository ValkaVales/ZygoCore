#pragma once

#include <zygo/core/types.h>
#include <zygo/core/assert.h>
#include <zygo/math/vector/vec3.h>
#include <zygo/math/common/scalar.h>
#include <cmath> // std::isfinite(), fabs, sqrt


namespace zygo {

#define USE_RELATIVE_EPS_IN_MAT3_SOLVE_AND_INVERSE // comment this for max performance (but worse checks)


// class for the physics solver

struct Mat3 final
{
  // Row-major:
  //
  // [ m[0] m[1] m[2] ]
  // [ m[3] m[4] m[5] ]
  // [ m[6] m[7] m[8] ]
  //
  Real m[9];

  // IMPORTANT: does NOT initialize the matrix.
  // This is faster for temporary Mat3 in the physics solver.
  Mat3() noexcept = default;

  constexpr Mat3(
    Real m00, Real m01, Real m02,
    Real m10, Real m11, Real m12,
    Real m20, Real m21, Real m22
  ) noexcept
    : m { m00, m01, m02,
          m10, m11, m12,
          m20, m21, m22
        }
  {}

  // ------------------------------------------------------------
  // Factories
  // ------------------------------------------------------------

  static constexpr Mat3 zero() noexcept
  {
    return Mat3(
      REAL_ZERO, REAL_ZERO, REAL_ZERO,
      REAL_ZERO, REAL_ZERO, REAL_ZERO,
      REAL_ZERO, REAL_ZERO, REAL_ZERO
    );
  }

  static constexpr Mat3 identity() noexcept
  {
    return Mat3(
      REAL_ONE , REAL_ZERO, REAL_ZERO,
      REAL_ZERO, REAL_ONE , REAL_ZERO,
      REAL_ZERO, REAL_ZERO, REAL_ONE
    );
  }

  static constexpr Mat3 diagonal( Real x, Real y, Real z ) noexcept
  {
    return Mat3(
      x          , REAL_ZERO, REAL_ZERO,
      REAL_ZERO, y          , REAL_ZERO,
      REAL_ZERO, REAL_ZERO, z
    );
  }

  static Mat3 diagonal( Vector3 const & v ) noexcept
  {
    return diagonal( v.x, v.y, v.z );
  }

  static Mat3 fromRows(
    Vector3 const & r0,
    Vector3 const & r1,
    Vector3 const & r2
  ) noexcept
  {
    return Mat3(
      r0.x, r0.y, r0.z,
      r1.x, r1.y, r1.z,
      r2.x, r2.y, r2.z
    );
  }

  static Mat3 fromColumns(
    Vector3 const & c0,
    Vector3 const & c1,
    Vector3 const & c2
  ) noexcept
  {
    return Mat3(
      c0.x, c1.x, c2.x,
      c0.y, c1.y, c2.y,
      c0.z, c1.z, c2.z
    );
  }

  // Cross product matrix - from vector.
  //
  // skew(a) * b == cross(a, b)
  //
  // [   0  -az   ay ]
  // [  az    0  -ax ]
  // [ -ay   ax    0 ]
  //
  static Mat3 skew( Vector3 const & a ) noexcept
  {
    return Mat3(
       REAL_ZERO, -a.z        ,  a.y,
       a.z        ,  REAL_ZERO, -a.x,
      -a.y        ,  a.x        ,  REAL_ZERO
    );
  }

  // Outer product:
  //
  // result = a * b^T
  //
  static Mat3 outerProduct( Vector3 const & a, Vector3 const & b ) noexcept
  {
    return Mat3(
      a.x * b.x,  a.x * b.y,  a.x * b.z,
      a.y * b.x,  a.y * b.y,  a.y * b.z,
      a.z * b.x,  a.z * b.y,  a.z * b.z
    );
  }

  // Convenient for world inertia:
  //
  // Iworld = mat * diag(Ibody) * mat^T
  //
  static Mat3 rotatedDiagonal( Mat3 const & mat, Vector3 const & d ) noexcept
  {
    const Real dx = d.x;
    const Real dy = d.y;
    const Real dz = d.z;

    return Mat3(
      dx * mat.m[0] * mat.m[0]  +  dy * mat.m[1] * mat.m[1]  +  dz * mat.m[2] * mat.m[2],
      dx * mat.m[0] * mat.m[3]  +  dy * mat.m[1] * mat.m[4]  +  dz * mat.m[2] * mat.m[5],
      dx * mat.m[0] * mat.m[6]  +  dy * mat.m[1] * mat.m[7]  +  dz * mat.m[2] * mat.m[8],

      dx * mat.m[3] * mat.m[0]  +  dy * mat.m[4] * mat.m[1]  +  dz * mat.m[5] * mat.m[2],
      dx * mat.m[3] * mat.m[3]  +  dy * mat.m[4] * mat.m[4]  +  dz * mat.m[5] * mat.m[5],
      dx * mat.m[3] * mat.m[6]  +  dy * mat.m[4] * mat.m[7]  +  dz * mat.m[5] * mat.m[8],

      dx * mat.m[6] * mat.m[0]  +  dy * mat.m[7] * mat.m[1]  +  dz * mat.m[8] * mat.m[2],
      dx * mat.m[6] * mat.m[3]  +  dy * mat.m[7] * mat.m[4]  +  dz * mat.m[8] * mat.m[5],
      dx * mat.m[6] * mat.m[6]  +  dy * mat.m[7] * mat.m[7]  +  dz * mat.m[8] * mat.m[8]
    );
  }


  // Inertia tensor of a solid box around its center of mass.
  // sx, sy, sz are full side lengths, not half-extents.
  static Mat3 inertiaBox( Real mass, Real sx, Real sy, Real sz ) noexcept
  {
    ZgAssert( mass >= EPSILON );
    ZgAssert( sx >= EPSILON && sy >= EPSILON && sz >= EPSILON );

    const Real k = mass / (Real)12;

    return diagonal(
      k * (sy * sy + sz * sz),
      k * (sx * sx + sz * sz),
      k * (sx * sx + sy * sy)
    );
  }

  // Inertia tensor of a solid sphere around its center of mass.
  static Mat3 inertiaSphere( Real mass, Real radius ) noexcept
  {
    ZgAssert( mass >= EPSILON );
    ZgAssert( radius >= EPSILON );

    const Real i = (Real)0.4 * mass * radius * radius; // 2/5 * m * r^2
    return diagonal( i, i, i );
  }

  // Inertia tensor of a solid cylinder around its center of mass.
  // length is along the selected local axis.
  static Mat3 inertiaCylinderX( Real mass, Real radius, Real length ) noexcept
  {
    ZgAssert( mass >= EPSILON );
    ZgAssert( radius >= EPSILON );
    ZgAssert( length >= EPSILON );

    const Real r2 = radius * radius;
    const Real l2 = length * length;

    const Real i_axis = REAL_HALF * mass * r2;
    const Real i_side = mass * ((Real)3 * r2 + l2) / (Real)12;

    return diagonal( i_axis, i_side, i_side );
  }

  static Mat3 inertiaCylinderY( Real mass, Real radius, Real length ) noexcept
  {
    ZgAssert( mass >= EPSILON );
    ZgAssert( radius >= EPSILON );
    ZgAssert( length >= EPSILON );

    const Real r2 = radius * radius;
    const Real l2 = length * length;

    const Real i_axis = REAL_HALF * mass * r2;
    const Real i_side = mass * ((Real)3 * r2 + l2) / (Real)12;

    return diagonal( i_side, i_axis, i_side );
  }

  static Mat3 inertiaCylinderZ( Real mass, Real radius, Real length ) noexcept
  {
    ZgAssert( mass >= EPSILON );
    ZgAssert( radius >= EPSILON );
    ZgAssert( length >= EPSILON );

    const Real r2 = radius * radius;
    const Real l2 = length * length;

    const Real i_axis = REAL_HALF * mass * r2;
    const Real i_side = mass * ((Real)3 * r2 + l2) / (Real)12;

    return diagonal( i_side, i_side, i_axis );
  }

  // ------------------------------------------------------------
  // Access
  // ------------------------------------------------------------
/*
  Real& operator[] ( int idx ) noexcept
  {
    ZgAssert( idx >= 0 && idx < 9 );
    return m[idx];
  }
*/
  Real& operator() ( int row, int col ) noexcept
  {
    ZgAssert( row >= 0 && row < 3 );
    ZgAssert( col >= 0 && col < 3 );
    return m[row * 3 + col];
  }

  Real operator() ( int row, int col ) const noexcept
  {
    ZgAssert( row >= 0 && row < 3 );
    ZgAssert( col >= 0 && col < 3 );
    return m[row * 3 + col];
  }

  Real       * data()       noexcept { return m; }
  Real const * data() const noexcept { return m; }

  Vector3 row0() const noexcept { return Vector3( m[0], m[1], m[2] ); }
  Vector3 row1() const noexcept { return Vector3( m[3], m[4], m[5] ); }
  Vector3 row2() const noexcept { return Vector3( m[6], m[7], m[8] ); }

  Vector3 col0() const noexcept { return Vector3( m[0], m[3], m[6] ); }
  Vector3 col1() const noexcept { return Vector3( m[1], m[4], m[7] ); }
  Vector3 col2() const noexcept { return Vector3( m[2], m[5], m[8] ); }

  // ------------------------------------------------------------
  // Basic math
  // ------------------------------------------------------------

  Mat3 operator+ () const noexcept
  {
    return *this;
  }

  Mat3 operator- () const noexcept
  {
    return Mat3(
      -m[0], -m[1], -m[2],
      -m[3], -m[4], -m[5],
      -m[6], -m[7], -m[8]
    );
  }

  Mat3 operator+ ( Mat3 const & b ) const noexcept
  {
    return Mat3(
      m[0] + b.m[0], m[1] + b.m[1], m[2] + b.m[2],
      m[3] + b.m[3], m[4] + b.m[4], m[5] + b.m[5],
      m[6] + b.m[6], m[7] + b.m[7], m[8] + b.m[8]
    );
  }

  Mat3 operator- ( Mat3 const & b ) const noexcept
  {
    return Mat3(
      m[0] - b.m[0], m[1] - b.m[1], m[2] - b.m[2],
      m[3] - b.m[3], m[4] - b.m[4], m[5] - b.m[5],
      m[6] - b.m[6], m[7] - b.m[7], m[8] - b.m[8]
    );
  }

  Mat3 operator* ( Real s ) const noexcept
  {
    return Mat3(
      m[0] * s, m[1] * s, m[2] * s,
      m[3] * s, m[4] * s, m[5] * s,
      m[6] * s, m[7] * s, m[8] * s
    );
  }

  Mat3 operator/ ( Real s ) const noexcept
  {
    ZgAssert( !isZero( s, SMALL_EPSILON ) );
    const Real inv = REAL_ONE / s;
    return (*this) * inv;
  }

  Mat3& operator += ( Mat3 const & b ) noexcept
  {
    m[0] += b.m[0]; m[1] += b.m[1]; m[2] += b.m[2];
    m[3] += b.m[3]; m[4] += b.m[4]; m[5] += b.m[5];
    m[6] += b.m[6]; m[7] += b.m[7]; m[8] += b.m[8];
    return *this;
  }

  Mat3& operator -= ( Mat3 const & b ) noexcept
  {
    m[0] -= b.m[0]; m[1] -= b.m[1]; m[2] -= b.m[2];
    m[3] -= b.m[3]; m[4] -= b.m[4]; m[5] -= b.m[5];
    m[6] -= b.m[6]; m[7] -= b.m[7]; m[8] -= b.m[8];
    return *this;
  }

  Mat3& operator *= ( Real s ) noexcept
  {
    m[0] *= s; m[1] *= s; m[2] *= s;
    m[3] *= s; m[4] *= s; m[5] *= s;
    m[6] *= s; m[7] *= s; m[8] *= s;
    return *this;
  }

  Mat3& operator /= ( Real s ) noexcept
  {
    ZgAssert( !isZero( s, SMALL_EPSILON ) );
    const Real inv = REAL_ONE / s;
    return (*this) *= inv;
  }

  friend Mat3 operator* (Real s, const Mat3& a) noexcept
  {
    return a * s;
  }

  // ------------------------------------------------------------
  // Matrix * Vector
  // ------------------------------------------------------------

  Vector3 operator* ( const Vector3& v ) const noexcept
  {
    return Vector3(
      m[0] * v.x + m[1] * v.y + m[2] * v.z,
      m[3] * v.x + m[4] * v.y + m[5] * v.z,
      m[6] * v.x + m[7] * v.y + m[8] * v.z
    );
  }

  // this^T * v
  Vector3 transposedMul( const Vector3& v ) const noexcept
  {
    return Vector3(
      m[0] * v.x + m[3] * v.y + m[6] * v.z,
      m[1] * v.x + m[4] * v.y + m[7] * v.z,
      m[2] * v.x + m[5] * v.y + m[8] * v.z
    );
  }

  // ------------------------------------------------------------
  // Matrix * Matrix
  // ------------------------------------------------------------

  Mat3 operator* ( Mat3 const & b ) const noexcept
  {
    return Mat3(
      m[0] * b.m[0] + m[1] * b.m[3] + m[2] * b.m[6],
      m[0] * b.m[1] + m[1] * b.m[4] + m[2] * b.m[7],
      m[0] * b.m[2] + m[1] * b.m[5] + m[2] * b.m[8],

      m[3] * b.m[0] + m[4] * b.m[3] + m[5] * b.m[6],
      m[3] * b.m[1] + m[4] * b.m[4] + m[5] * b.m[7],
      m[3] * b.m[2] + m[4] * b.m[5] + m[5] * b.m[8],

      m[6] * b.m[0] + m[7] * b.m[3] + m[8] * b.m[6],
      m[6] * b.m[1] + m[7] * b.m[4] + m[8] * b.m[7],
      m[6] * b.m[2] + m[7] * b.m[5] + m[8] * b.m[8]
    );
  }

  // this * b^T
  Mat3 mulTransposedRight( Mat3 const & b ) const noexcept
  {
    return Mat3(
      m[0] * b.m[0] + m[1] * b.m[1] + m[2] * b.m[2],
      m[0] * b.m[3] + m[1] * b.m[4] + m[2] * b.m[5],
      m[0] * b.m[6] + m[1] * b.m[7] + m[2] * b.m[8],

      m[3] * b.m[0] + m[4] * b.m[1] + m[5] * b.m[2],
      m[3] * b.m[3] + m[4] * b.m[4] + m[5] * b.m[5],
      m[3] * b.m[6] + m[4] * b.m[7] + m[5] * b.m[8],

      m[6] * b.m[0] + m[7] * b.m[1] + m[8] * b.m[2],
      m[6] * b.m[3] + m[7] * b.m[4] + m[8] * b.m[5],
      m[6] * b.m[6] + m[7] * b.m[7] + m[8] * b.m[8]
    );
  }

  // this^T * b
  Mat3 transposedMul( Mat3 const & b ) const noexcept
  {
    return Mat3(
      m[0] * b.m[0] + m[3] * b.m[3] + m[6] * b.m[6],
      m[0] * b.m[1] + m[3] * b.m[4] + m[6] * b.m[7],
      m[0] * b.m[2] + m[3] * b.m[5] + m[6] * b.m[8],

      m[1] * b.m[0] + m[4] * b.m[3] + m[7] * b.m[6],
      m[1] * b.m[1] + m[4] * b.m[4] + m[7] * b.m[7],
      m[1] * b.m[2] + m[4] * b.m[5] + m[7] * b.m[8],

      m[2] * b.m[0] + m[5] * b.m[3] + m[8] * b.m[6],
      m[2] * b.m[1] + m[5] * b.m[4] + m[8] * b.m[7],
      m[2] * b.m[2] + m[5] * b.m[5] + m[8] * b.m[8]
    );
  }

  // ------------------------------------------------------------
  // Scale-relative tolerances
  // ------------------------------------------------------------
  //
  // Every "is this matrix still usable" test below compares against a threshold RELATIVE to the magnitude of the matrix itself.
  //
  // An absolute threshold cannot work here: the determinant of a 3x3 scales as (element magnitude)^3, so one fixed eps is simultaneously
  //   - far too strict for a healthy inertia tensor of a small part
  //     (40 g foot sphere, r = 14 mm: I = 3.1e-06, det = 3.1e-17), and
  //   - far too loose for a large one
  //     (diag(1e9, 1e9, 1e-9) is hopeless, yet its determinant is 1e9).
  //
  // So `relative_eps` means "how far above the double round-off we insist on being", not "how big the determinant must be".

  // The threshold a determinant of a matrix of THIS magnitude has to beat.
  Real determinantEpsilon( Real relative_eps = EPSILON ) const noexcept
  {
    const Real s = maxAbsElement();
    return relative_eps * s * s * s;
  }

  // The threshold a first-order quantity (a pivot, a diagonal element) has to beat.
  Real pivotEpsilon( Real relative_eps = EPSILON ) const noexcept
  {
    return relative_eps * maxAbsElement();
  }

  // ------------------------------------------------------------
  // Transpose / determinant / inverse
  // ------------------------------------------------------------

  Mat3 transposed() const noexcept
  {
    return Mat3(
      m[0], m[3], m[6],
      m[1], m[4], m[7],
      m[2], m[5], m[8]
    );
  }

  void transposeInPlace() noexcept
  {
    const Real t01 = m[1];
    const Real t02 = m[2];
    const Real t12 = m[5];

    m[1] = m[3];
    m[2] = m[6];
    m[5] = m[7];

    m[3] = t01;
    m[6] = t02;
    m[7] = t12;
  }

  Real trace() const noexcept
  {
    return m[0] + m[4] + m[8];
  }

  Real determinant() const noexcept
  {
    return
      m[0] * (m[4] * m[8] - m[5] * m[7]) -
      m[1] * (m[3] * m[8] - m[5] * m[6]) +
      m[2] * (m[3] * m[7] - m[4] * m[6]);
  }

  // relative_eps is RELATIVE - see "Scale-relative tolerances" above.
  bool tryInverse( Mat3& out, Real relative_eps = EPSILON ) const noexcept
  {
    const Real c00 = m[4] * m[8] - m[5] * m[7];
    const Real c01 = m[2] * m[7] - m[1] * m[8];
    const Real c02 = m[1] * m[5] - m[2] * m[4];

    const Real det = m[0] * c00 + m[3] * c01 + m[6] * c02;

    // Written as !( > ) so that a NaN determinant is rejected too.
#ifdef USE_RELATIVE_EPS_IN_MAT3_SOLVE_AND_INVERSE
    if ( !( std::fabs( det ) > determinantEpsilon( relative_eps ) ) )
#else
    if ( !( std::fabs( det ) > relative_eps ) )
#endif
      return false;

    const Real inv_det = REAL_ONE / det;

    out = Mat3(
      c00 * inv_det,
      c01 * inv_det,
      c02 * inv_det,

      (m[5] * m[6] - m[3] * m[8]) * inv_det,
      (m[0] * m[8] - m[2] * m[6]) * inv_det,
      (m[2] * m[3] - m[0] * m[5]) * inv_det,

      (m[3] * m[7] - m[4] * m[6]) * inv_det,
      (m[1] * m[6] - m[0] * m[7]) * inv_det,
      (m[0] * m[4] - m[1] * m[3]) * inv_det
    );

    return true;
  }

  Mat3 inversed() const noexcept
  {
    Mat3 out;
    const bool ok = tryInverse( out );
    ZgAssertRelease( ok );
    return out;
  }

  // ------------------------------------------------------------
  // Solvers
  // ------------------------------------------------------------

  // Solves: this * x = b
  // This is better than: x = this->inversed() * b
  //
  // relative_eps is RELATIVE - see "Scale-relative tolerances" above.
  bool trySolve( Vector3 const& b, Vector3& x, Real relative_eps = EPSILON ) const noexcept
  {
    const Real c00 = m[4] * m[8] - m[5] * m[7];
    const Real c01 = m[2] * m[7] - m[1] * m[8];
    const Real c02 = m[1] * m[5] - m[2] * m[4];

    const Real det = m[0] * c00 + m[3] * c01 + m[6] * c02;

    // Written as !( > ) so that a NaN determinant is rejected too.
#ifdef USE_RELATIVE_EPS_IN_MAT3_SOLVE_AND_INVERSE
    if ( !( std::fabs( det ) > determinantEpsilon( relative_eps ) ) )
#else
    if ( !( std::fabs( det ) > relative_eps ) )
#endif
      return false;

    const Real inv_det = REAL_ONE / det;

    const Real i00 = c00 * inv_det;
    const Real i01 = c01 * inv_det;
    const Real i02 = c02 * inv_det;

    const Real i10 = (m[5] * m[6] - m[3] * m[8]) * inv_det;
    const Real i11 = (m[0] * m[8] - m[2] * m[6]) * inv_det;
    const Real i12 = (m[2] * m[3] - m[0] * m[5]) * inv_det;

    const Real i20 = (m[3] * m[7] - m[4] * m[6]) * inv_det;
    const Real i21 = (m[1] * m[6] - m[0] * m[7]) * inv_det;
    const Real i22 = (m[0] * m[4] - m[1] * m[3]) * inv_det;

    x = Vector3(
      i00 * b.x + i01 * b.y + i02 * b.z,
      i10 * b.x + i11 * b.y + i12 * b.z,
      i20 * b.x + i21 * b.y + i22 * b.z
    );

    return true;
  }

  // A fast solver for symmetric positive definite matrices.
  // Very useful for effective mass matrix constraints: K * lambda = rhs
  //
  // Requirements:
  // - the matrix must be symmetric;
  // - the matrix must be positive definite.
  //
  // Uses the lower triangle:
  //
  // [ m00  *   *  ]
  // [ m10 m11  *  ]
  // [ m20 m21 m22 ]
  //
  bool trySolveSPD( Vector3 const& b, Vector3& x, Real relative_eps = EPSILON ) const noexcept
  {
    ZgAssert( isSymmetric( relative_eps * 10 ) );

    const Real a00 = m[0];
    const Real a10 = m[3];
    const Real a20 = m[6];
    const Real a11 = m[4];
    const Real a21 = m[7];
    const Real a22 = m[8];

    // The pivots are first-order in the matrix magnitude, so they are compared against pivotEpsilon(), not against a fixed number.
#ifdef USE_RELATIVE_EPS_IN_MAT3_SOLVE_AND_INVERSE
    const Real pivot_eps = pivotEpsilon( relative_eps );
#else
    const Real pivot_eps = relative_eps;
#endif

    if ( !( a00 > pivot_eps ) )
      return false;

    const Real l00 = std::sqrt( a00 );

    const Real l10 = a10 / l00;
    const Real l20 = a20 / l00;

    const Real d11 = a11 - l10 * l10;

    if ( !( d11 > pivot_eps ) )
      return false;

    const Real l11 = std::sqrt( d11 );
    const Real l21 = (a21 - l20 * l10) / l11;
    const Real d22 = a22 - l20 * l20 - l21 * l21;

    if ( !( d22 > pivot_eps ) )
      return false;

    const Real l22 = std::sqrt( d22 );

    // Solve L * y = b
    const Real y0 = b.x / l00;
    const Real y1 = (b.y - l10 * y0) / l11;
    const Real y2 = (b.z - l20 * y0 - l21 * y1) / l22;

    // Solve L^T * x = y
    const Real x2 = y2 / l22;
    const Real x1 = (y1 - l21 * x2) / l11;
    const Real x0 = (y0 - l10 * x1 - l20 * x2) / l00;

    x = Vector3( x0, x1, x2 );
    return true;
  }

  // Fast solver for symmetric positive definite matrices - LDL^T, WITHOUT square roots.
  //
  //   A = L * D * L^T,  L unit lower triangular,  D diagonal
  //
  // Same idea as trySolveSPD() (Cholesky), but A = L*D*L^T instead of L*L^T, so the three sqrt disappear,
  // and the three divisions are taken once as reciprocals instead of nine times inline.
  //
  // Measured on 3x3 constraint effective-mass matrices (x86-64, /O2), ns per solve:
  //
  //   trySolve    (Cramer,  1 division)          14.8   1.00x   residual 2.6e-16
  //   trySolveSPD (Cholesky, 3 sqrt + 9 div)     21.8   1.47x   residual 2.1e-16
  //   trySolveLDLT           (0 sqrt + 3 div)    13.7   0.93x   residual 1.8e-16
  //
  // The whole gap is the sqrt: folding the nine divisions of trySolveSPD into three reciprocals while KEEPING the sqrt only moved 1.47x to 1.45x.
  // On a 3x3 the sqrt are also the only reason Cholesky ever looked slower than Cramer here.
  //
  // Requirements: the matrix must be symmetric AND positive definite.
  // Unlike trySolve() this actually VERIFIES it - a non-SPD matrix returns false rather than a garbage solution,
  // which is what makes it the safer default for a constraint solver.
  //
  // Uses the lower triangle:
  //
  // [ m00  *   *  ]
  // [ m10 m11  *  ]
  // [ m20 m21 m22 ]
  //
  bool trySolveLDLT( Vector3 const& b, Vector3& x, Real relative_eps = EPSILON ) const noexcept
  {
    ZgAssert( isSymmetric( relative_eps * 10 ) );

    // The pivots are first-order in the matrix magnitude - see "Scale-relative tolerances".
#ifdef USE_RELATIVE_EPS_IN_MAT3_SOLVE_AND_INVERSE
    const Real pivot_eps = pivotEpsilon( relative_eps );
#else
    const Real pivot_eps = relative_eps;
#endif

    // ---- factorization ----
    const Real d0 = m[0];
    if ( !( d0 > pivot_eps ) )
      return false;

    const Real r0 = REAL_ONE / d0;

    const Real l10 = m[3] * r0;
    const Real l20 = m[6] * r0;

    const Real d1 = m[4] - l10 * l10 * d0;
    if ( !( d1 > pivot_eps ) )
      return false;

    const Real r1 = REAL_ONE / d1;

    const Real l21 = ( m[7] - l20 * l10 * d0 ) * r1;

    const Real d2 = m[8] - l20 * l20 * d0 - l21 * l21 * d1;
    if ( !( d2 > pivot_eps ) )
      return false;

    const Real r2 = REAL_ONE / d2;

    // ---- solve L * y = b ----
    const Real y0 = b.x;
    const Real y1 = b.y - l10 * y0;
    const Real y2 = b.z - l20 * y0 - l21 * y1;

    // ---- solve D * z = y, then L^T * x = z (fused) ----
    const Real x2 = y2 * r2;
    const Real x1 = y1 * r1 - l21 * x2;
    const Real x0 = y0 * r0 - l10 * x1 - l20 * x2;

    x = Vector3( x0, x1, x2 );
    return true;
  }

  // ------------------------------------------------------------
  // Physics helpers
  // ------------------------------------------------------------

  bool isEqual( Mat3 const& b, Real eps = EPSILON ) const noexcept
  {
    for ( int i = 0; i < 9; ++i )
      if ( !eq( m[i], b.m[i], eps ) )
        return false;

    return true;
  }

  bool isZeroMatrix( Real eps = EPSILON ) const noexcept
  {
    return maxAbsElement() <= eps;
  }

  // relative_eps is RELATIVE: a rotated inertia tensor is symmetric analytically but only to within a few ulps of ITS OWN elements,
  // so an absolute eps either rejects every large tensor or accepts every small non-symmetric one.
  bool isSymmetric( Real relative_eps = EPSILON ) const noexcept
  {
    const Real eps = pivotEpsilon( relative_eps );

    return
      eq( m[1], m[3], eps ) &&
      eq( m[2], m[6], eps ) &&
      eq( m[5], m[7], eps );
  }

  // SPD = Symmetric Positive Definite.
  // Sylvester's criterion: all corner minors > 0.
  // Works ONLY in conjunction with the symmetry check - for asymmetric matrices, the criterion is unacceptable.
  bool isSPD( Real relative_eps = EPSILON ) const noexcept
  {
    if ( !isSymmetric( relative_eps ) )
      return false;

    Real const s = maxAbsElement(); // maxAbsElement() is called twice in isSPD: once here, once - in isSymmetric(). maybeTODO: fix it.

    Real const minor1 = m[0];
    Real const minor2 = m[0] * m[4] - m[1] * m[3];
    Real const minor3 = determinant();

    // Each leading minor is compared against its OWN power of the matrix magnitude.
    return minor1 > relative_eps * s
        && minor2 > relative_eps * s * s
        && minor3 > relative_eps * s * s * s;
  }

  Real maxAbsElement() const noexcept
  {
    Real res = std::fabs( m[0] );

    for ( int i = 1; i < 9; ++i )
    {
      const Real a = std::fabs( m[i] );
      if ( res < a )
        res = a;
    }

    return res;
  }

  Real frobeniusNormSqr() const noexcept
  {
    return
      m[0] * m[0] + m[1] * m[1] + m[2] * m[2] +
      m[3] * m[3] + m[4] * m[4] + m[5] * m[5] +
      m[6] * m[6] + m[7] * m[7] + m[8] * m[8];
  }

  Mat3 withoutTinyValues( Real eps ) const noexcept
  {
    return Mat3(
      isZero( m[0], eps ) ? REAL_ZERO : m[0],
      isZero( m[1], eps ) ? REAL_ZERO : m[1],
      isZero( m[2], eps ) ? REAL_ZERO : m[2],
      isZero( m[3], eps ) ? REAL_ZERO : m[3],
      isZero( m[4], eps ) ? REAL_ZERO : m[4],
      isZero( m[5], eps ) ? REAL_ZERO : m[5],
      isZero( m[6], eps ) ? REAL_ZERO : m[6],
      isZero( m[7], eps ) ? REAL_ZERO : m[7],
      isZero( m[8], eps ) ? REAL_ZERO : m[8]
    );
  }

  Mat3 symmetrized() const noexcept
  {
    return Mat3(
      m[0],
      REAL_HALF * (m[1] + m[3]),
      REAL_HALF * (m[2] + m[6]),

      REAL_HALF * (m[3] + m[1]),
      m[4],
      REAL_HALF * (m[5] + m[7]),

      REAL_HALF * (m[6] + m[2]),
      REAL_HALF * (m[7] + m[5]),
      m[8]
    );
  }

  void makeSymmetric() noexcept
  {
    const Real a01 = REAL_HALF * (m[1] + m[3]);
    const Real a02 = REAL_HALF * (m[2] + m[6]);
    const Real a12 = REAL_HALF * (m[5] + m[7]);

    m[1] = a01;
    m[3] = a01;

    m[2] = a02;
    m[6] = a02;

    m[5] = a12;
    m[7] = a12;
  }

  Mat3 absElements() const noexcept
  {
    return Mat3(
      std::fabs(m[0]), std::fabs(m[1]), std::fabs(m[2]),
      std::fabs(m[3]), std::fabs(m[4]), std::fabs(m[5]),
      std::fabs(m[6]), std::fabs(m[7]), std::fabs(m[8])
    );
  }

  //
  bool isFinite() const noexcept
  {
    return
      std::isfinite(m[0]) && std::isfinite(m[1]) && std::isfinite(m[2]) &&
      std::isfinite(m[3]) && std::isfinite(m[4]) && std::isfinite(m[5]) &&
      std::isfinite(m[6]) && std::isfinite(m[7]) && std::isfinite(m[8]);
  }
};

} // namespace zygo
