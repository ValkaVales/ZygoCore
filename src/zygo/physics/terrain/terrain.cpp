#include "terrain.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <cmath>


namespace zygo {
namespace phys {

namespace
{
  // FlatGround debug drawing.
  const double FLAT_GROUND_DRAW_SIZE = 10.0; // meters in each direction
  const double FLAT_GROUND_GRID_STEP = 0.25; // meters

  const uint FLAT_GROUND_FILL_COLOR = 0x203020;
  const uint FLAT_GROUND_GRID_COLOR = 0x506050;

  // HeightField debug drawing.
  const uint HEIGHT_FIELD_FILL_COLOR = 0x304030;
  const uint HEIGHT_FIELD_WIRE_COLOR = 0x607060;
}


// --------------------------------------------------------------------------- FlatGround
FlatGround::FlatGround( double ground_z )
  : ground_z ( ground_z )
{
}

TerrainContact FlatGround::querySphere( Vector3 const & center, double radius, double margin ) const
{
  TerrainContact tc;

  double lowest       = center.z - radius; // the lowest point of the sphere
  double penetration  = ground_z - lowest; // > 0 => penetration

  if ( penetration < -margin ) // not touching the ground
    return tc;

  tc.hit          = true;
  tc.normal       = Vector3( 0.0, 0.0, 1.0 );
  tc.point        = Vector3( center.x, center.y, ground_z );
  tc.penetration  = std::max( 0.0, penetration );

  return tc;
}

void FlatGround::draw( IPhysicsDrawer const& drawer ) const
{
  drawer.planeXY(
    ground_z,
    -FLAT_GROUND_DRAW_SIZE,
    FLAT_GROUND_DRAW_SIZE,
    -FLAT_GROUND_DRAW_SIZE,
    FLAT_GROUND_DRAW_SIZE,
    FLAT_GROUND_FILL_COLOR
  );

  drawer.gridXY(
    ground_z,
    -FLAT_GROUND_DRAW_SIZE,
    FLAT_GROUND_DRAW_SIZE,
    -FLAT_GROUND_DRAW_SIZE,
    FLAT_GROUND_DRAW_SIZE,
    FLAT_GROUND_GRID_STEP,
    FLAT_GROUND_GRID_COLOR
  );
}


// --------------------------------------------------------------------------- HeightField
HeightField::HeightField( double origin_x, double origin_y, double cell, int nx, int ny, double init_h )
  : origin_x ( origin_x )
  , origin_y ( origin_y )
  , cell     ( cell )
  , nx       ( nx )
  , ny       ( ny )
{
  ZgAssert( cell > BIG_EPSILON );
  ZgAssert( nx >= 2 && ny >= 2 );

  heights.assign( (size_t)nx * (size_t)ny, init_h );
}

void HeightField::draw( IPhysicsDrawer const& drawer ) const
{
  drawer.heightFieldXY(
    origin_x,
    origin_y,
    cell,
    nx,
    ny,
    heights,
    HEIGHT_FIELD_FILL_COLOR,
    HEIGHT_FIELD_WIRE_COLOR
  );
}

void HeightField::setHeight( int ix, int iy, double h )
{
  ZgAssert( ix >= 0 && ix < nx );
  ZgAssert( iy >= 0 && iy < ny );
  heights[ (size_t)iy * nx + ix ] = h;
}

double HeightField::sampleClamped( int ix, int iy ) const
{
  // Outside the grid, extrapolate with the edge value (the ground continues flat).
  if ( ix < 0 )       ix = 0;
  if ( ix > nx - 1 )  ix = nx - 1;
  if ( iy < 0 )       iy = 0;
  if ( iy > ny - 1 )  iy = ny - 1;

  return heights[ (size_t)iy * nx + ix ];
}

double HeightField::heightAt( double x, double y ) const
{
  double fx = ( x - origin_x ) / cell;
  double fy = ( y - origin_y ) / cell;

  int ix = (int)floor( fx );
  int iy = (int)floor( fy );

  double tx = fx - ix;
  double ty = fy - iy;

  double h00 = sampleClamped( ix,     iy     );
  double h10 = sampleClamped( ix + 1, iy     );
  double h01 = sampleClamped( ix,     iy + 1 );
  double h11 = sampleClamped( ix + 1, iy + 1 );

  double h0 = h00 * ( 1.0 - tx ) + h10 * tx;
  double h1 = h01 * ( 1.0 - tx ) + h11 * tx;

  return h0 * ( 1.0 - ty ) + h1 * ty;
}

Vector3 HeightField::normalAt( double x, double y ) const
{
  // Central differences over the interpolated field.
  double hx1 = heightAt( x - cell, y );
  double hx2 = heightAt( x + cell, y );
  double hy1 = heightAt( x, y - cell );
  double hy2 = heightAt( x, y + cell );

  double dzdx = ( hx2 - hx1 ) / ( 2.0 * cell );
  double dzdy = ( hy2 - hy1 ) / ( 2.0 * cell );

  // The surface is z = h(x,y); its normal is (-dz/dx, -dz/dy, 1).
  Vector3 n( -dzdx, -dzdy, 1.0 );
  return Vector3::safeNormalized( n );
}

TerrainContact HeightField::querySphere( Vector3 const & center, double radius, double margin ) const
{
  TerrainContact tc;

  // Approximation: locally treat the ground as the tangent plane in the column
  // under the sphere center. Good enough for gentle hills and feet.
  double H = heightAt( center.x, center.y );
  Vector3 n = normalAt( center.x, center.y );

  // The surface point right under the center (same x,y).
  Vector3 surf( center.x, center.y, H );

  // The signed distance from the sphere center to the tangent plane along the normal.
  double dist = ( center - surf ) * n;        // dot
  double penetration = radius - dist;         // > 0 => penetration

  if ( penetration < -margin ) // not touching the ground
    return tc;

  tc.hit         = true;
  tc.normal      = n;
  tc.point       = center - n * dist; // the closest point on the surface
  tc.penetration = std::max( 0.0, penetration );

  return tc;
}

} // namespace phys
} // namespace zygo
