#pragma once

// Terrain interface and the standard terrains.
// Everything is SI: meters, +Z up (see physics.h).

#include <zygo/math/vector/vec3.h>
#include <zygo/physics/i_physics_drawer.h>
#include <vector>


namespace zygo {
namespace phys {

// The result of a "sphere against the ground" query.
struct TerrainContact
{
  bool    hit         = false;
  Vector3 normal;             // unit, out of the ground (up, toward the sphere center)
  Vector3 point;              // the contact point on the surface, in the world
  double  penetration = 0.0;  // > 0 => the sphere penetrates the ground; clamped at 0

  // The SIGNED gap: separation > 0 means the sphere has not touched yet and is only inside the query margin.
  // Speculative contacts need this, and `penetration` cannot carry it because it is clamped at zero (and every existing user relies on that).
  //   separation == -penetration whenever they overlap.
  double separation  = 0.0;
};


// The common ground interface.
class ITerrain
{
public:
  virtual ~ITerrain() = default;

  virtual void draw( IPhysicsDrawer const& drawer ) const = 0;

  // Intersection query of a sphere (center, radius) against the ground.
  // margin - the "already counts as a contact" distance (the world passes settings.contacts.margin here).
  virtual TerrainContact querySphere( Vector3 const & center, double radius, double margin ) const = 0;

  // The surface height at (x, y).
  virtual double heightAt( double x, double y ) const = 0;
};


// An infinite horizontal plane z = ground_z, normal +Z.
// The trivial "flat surface" case.
class FlatGround : public ITerrain
{
private:
  double ground_z;

public:
  explicit FlatGround( double ground_z = 0.0 ); // meters (the SI zone, unlike the mm-based construction API)

  void draw( IPhysicsDrawer const& drawer ) const override;

  double heightAt( double /*x*/, double /*y*/ ) const override { return ground_z; }

  TerrainContact querySphere( Vector3 const & center, double radius, double margin ) const override;
};


// A regular height grid over the XY plane (heights along +Z).
// heights[ iy * nx + ix ], the node (ix,iy) lies in the world at
//   ( origin_x + ix*cell, origin_y + iy*cell ).
class HeightField : public ITerrain
{
private:
  double origin_x;
  double origin_y;
  double cell;   // cell size, meters (square cells)
  int    nx;
  int    ny;

  std::vector<double> heights;

  double sampleClamped( int ix, int iy ) const;

public:
  HeightField( double origin_x, double origin_y, double cell, int nx, int ny, double init_h = 0.0 );

  int    sizeX()    const { return nx; }
  int    sizeY()    const { return ny; }
  double cellSize() const { return cell; }

  void setHeight( int ix, int iy, double h );

  void draw( IPhysicsDrawer const& drawer ) const override;

  // Fill from a function h = f( x, y ) (world coordinates, meters).
  template <typename F>
  void fillFromFunction( F && f )
  {
    for ( int iy = 0; iy < ny; ++iy )
      for ( int ix = 0; ix < nx; ++ix )
        heights[iy * nx + ix] = f( origin_x + ix * cell, origin_y + iy * cell );
  }

  double  heightAt( double x, double y ) const override; // bilinear interpolation
  Vector3 normalAt( double x, double y ) const;          // the normal from the height gradient

  TerrainContact querySphere( Vector3 const & center, double radius, double margin ) const override;
};

} // namespace phys
} // namespace zygo
