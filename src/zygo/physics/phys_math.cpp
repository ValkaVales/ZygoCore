#include "phys_math.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <cmath>


namespace zygo {
namespace phys {

void buildSegmentBasis(
  Vector3 const & p1,
  Vector3 const & p2,
  Vector3 & x_axis,
  Vector3 & y_axis,
  Vector3 & z_axis
)
{
  x_axis = Vector3::safeNormalized( p2 - p1 );

  Vector3 world_up( 0.0, 0.0, 1.0 );
  Vector3 tmp = world_up.crossProduct( x_axis );

  if ( tmp.lengthSqr() < 1e-12 )
  {
    // The segment is (almost) vertical - fall back to another helper axis.
    Vector3 fallback( 1.0, 0.0, 0.0 );
    tmp = fallback.crossProduct( x_axis );
  }

  y_axis = Vector3::safeNormalized( tmp );
  z_axis = Vector3::safeNormalized( x_axis.crossProduct( y_axis ) );
}

Quaternion buildQuaternionFromAxes(
  Vector3 const & x_axis,
  Vector3 const & y_axis,
  Vector3 const & z_axis
)
{
  Matrix m( 3, 3 );

  // Columns = the local basis axes in object coordinates.
  m.set9(
    x_axis.x, y_axis.x, z_axis.x,
    x_axis.y, y_axis.y, z_axis.y,
    x_axis.z, y_axis.z, z_axis.z
  );

  return Quaternion::fromRotationMatrix( m );
}

Matrix calcParallelAxisTerm( double mass, Vector3 const & d )
{
  Matrix res( 3, 3 );
  res.makeAllZero();

  double x = d.x;
  double y = d.y;
  double z = d.z;
  double d2 = d.lengthSqr();

  res.setAt( 0, mass * ( d2 - x*x ) );
  res.setAt( 1, mass * (    - x*y ) );
  res.setAt( 2, mass * (    - x*z ) );

  res.setAt( 3, mass * (    - x*y ) );
  res.setAt( 4, mass * ( d2 - y*y ) );
  res.setAt( 5, mass * (    - y*z ) );

  res.setAt( 6, mass * (    - x*z ) );
  res.setAt( 7, mass * (    - y*z ) );
  res.setAt( 8, mass * ( d2 - z*z ) );

  return res;
}

void validateInertiaTensor( Matrix const & mat, double symmetry_eps, double det_eps, double det_eps_small )
{
  ZgAssert( mat.dimX() == 3 && mat.dimY() == 3 );

  double a00 = mat.get( 0, 0 );
  double a01 = mat.get( 0, 1 );
  double a02 = mat.get( 0, 2 );
  double a10 = mat.get( 1, 0 );
  double a11 = mat.get( 1, 1 );
  double a12 = mat.get( 1, 2 );
  double a20 = mat.get( 2, 0 );
  double a21 = mat.get( 2, 1 );
  double a22 = mat.get( 2, 2 );

  // 1) All elements are finite.
  ZgAssert( std::isfinite( a00 ) );
  ZgAssert( std::isfinite( a01 ) );
  ZgAssert( std::isfinite( a02 ) );
  ZgAssert( std::isfinite( a10 ) );
  ZgAssert( std::isfinite( a11 ) );
  ZgAssert( std::isfinite( a12 ) );
  ZgAssert( std::isfinite( a20 ) );
  ZgAssert( std::isfinite( a21 ) );
  ZgAssert( std::isfinite( a22 ) );

  // 2) Symmetry.
  ZgAssert( std::abs( a01 - a10 ) <= symmetry_eps );
  ZgAssert( std::abs( a02 - a20 ) <= symmetry_eps );
  ZgAssert( std::abs( a12 - a21 ) <= symmetry_eps );

  // 3) Positive diagonal.
  ZgAssert( a00 > 0.0 );
  ZgAssert( a11 > 0.0 );
  ZgAssert( a22 > 0.0 );

  // 4) Sylvester's criterion for an SPD matrix.
  ZgAssert( a00 > det_eps ); // first leading minor

  double det2x2 = a00 * a11 - a01 * a10; // second leading minor
  ZgAssert( det2x2 > det_eps );

  double det3x3 =
      a00 * ( a11 * a22 - a12 * a21 )
    - a01 * ( a10 * a22 - a12 * a20 )
    + a02 * ( a10 * a21 - a11 * a20 );

  ZgAssert( det3x3 > det_eps_small );

  // 5) A very rough check that the matrix is not almost degenerate
  //    (the determinant threshold is scaled by the magnitude of the elements).
  double max_abs = std::max(
    std::max( std::abs( a00 ), std::abs( a01 ) ),
    std::max(
      std::max( std::abs( a02 ), std::abs( a10 ) ),
      std::max(
        std::max( std::abs( a11 ), std::abs( a12 ) ),
        std::max( std::abs( a20 ),
          std::max( std::abs( a21 ), std::abs( a22 ) )
        )
      )
    )
  );

  double coeff = std::max( 1.0, max_abs * max_abs * max_abs );
  double scaled_det_eps = det_eps_small * coeff;
  ZgAssert( std::abs( det3x3 ) > scaled_det_eps );
}

} // namespace phys
} // namespace zygo
