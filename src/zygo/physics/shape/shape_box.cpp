#include "shape_box.h"
#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

ShapeBox::ShapeBox( double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color )
  : Shape( ShapeType::BOX, mass, local_pos, size, local_rot, color )
{
}

Matrix ShapeBox::calcLocalInertiaTensorForPart() const
{
  Matrix I( 3, 3 );
  I.makeAllZero();

  double m = mass;

  double a2 = sqr( size.x );
  double b2 = sqr( size.y );
  double c2 = sqr( size.z );

  // Uniform box.
  double Ixx = m * ( b2 + c2 ) / 12.0;
  double Iyy = m * ( a2 + c2 ) / 12.0;
  double Izz = m * ( a2 + b2 ) / 12.0;

  I.setAt( 0, Ixx );
  I.setAt( 4, Iyy );
  I.setAt( 8, Izz );

  return I;
}

void ShapeBox::draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const
{
  Vector3 world_center = calcWorldCenterOfMass( obj_world_pos, obj_world_rot );
  Quaternion world_rot = calcWorldRotation( obj_world_rot );

  drawer.box( world_center, world_rot, size.x, size.y, size.z, color );
}

} // namespace phys
} // namespace zygo
