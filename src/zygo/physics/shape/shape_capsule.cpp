#include "shape_capsule.h"

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

ShapeCapsule::ShapeCapsule( double mass, Vector3 const & local_pos, double cylinder_len, double radius, Quaternion const & local_rot, uint color )
  : Shape( ShapeType::CAPSULE, mass, local_pos, Vector3( radius, radius, cylinder_len ), local_rot, color )
{
}

Matrix ShapeCapsule::calcLocalInertiaTensorForPart() const
{
  Matrix I( 3, 3 );
  I.makeAllZero();

  // The capsule axis is the local X axis (RigidBody::addCapsuleBySegment() builds local_rot that way).
  const double m = mass;
  const double r = size.x;   // radius
  const double L = size.z;   // cylinder length

  ZgAssert( m > BIG_EPSILON );
  ZgAssert( r > BIG_EPSILON );
  ZgAssert( L > BIG_EPSILON );

  // Part volumes.
  const double Vc = PI * r * r * L;                 // cylinder
  const double Vh = ( 2.0 / 3.0 ) * PI * r * r * r; // one hemisphere
  const double Vtotal = Vc + 2.0 * Vh;

  ZgAssert( Vtotal > BIG_EPSILON );

  // Part masses assuming uniform density.
  const double mc = m * ( Vc / Vtotal );   // cylinder mass
  const double mh = m * ( Vh / Vtotal );   // one hemisphere mass

  // ----- 1) Cylinder (axis along X) -----
  const double Ixx_cyl = 0.5 * mc * r * r;
  const double Iyy_cyl = mc * ( 3.0 * r * r + L * L ) / 12.0;
  const double Izz_cyl = Iyy_cyl;

  // ----- 2) Hemispheres -----
  // The center of mass of a hemisphere lies 3r/8 from the flat cut along the symmetry axis.
  // Distance from the capsule center to the center of mass of each hemisphere:
  const double d = L * 0.5 + 3.0 * r / 8.0;

  // Hemisphere about its symmetry axis (X), through its own center of mass:
  const double Ixx_hemi_centroid = ( 2.0 / 5.0 ) * mh * r * r;

  // Hemisphere about a transverse axis (Y or Z), through its own center of mass:
  const double Iyy_hemi_centroid = ( 83.0 / 320.0 ) * mh * r * r;
  const double Izz_hemi_centroid = Iyy_hemi_centroid;

  // Translate to the capsule center.
  // No addition for Ixx: the translation is along the same X axis.
  const double Ixx_hemis = 2.0 * Ixx_hemi_centroid;
  const double Iyy_hemis = 2.0 * ( Iyy_hemi_centroid + mh * d * d );
  const double Izz_hemis = 2.0 * ( Izz_hemi_centroid + mh * d * d );

  I.setAt( 0, Ixx_cyl + Ixx_hemis );
  I.setAt( 4, Iyy_cyl + Iyy_hemis );
  I.setAt( 8, Izz_cyl + Izz_hemis );

  return I;
}

void ShapeCapsule::draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const
{
  Vector3 world_center = calcWorldCenterOfMass( obj_world_pos, obj_world_rot );
  Quaternion world_rot = calcWorldRotation( obj_world_rot );

  drawer.capsule( world_center, world_rot, size.x, size.z, color );
}

} // namespace phys
} // namespace zygo
