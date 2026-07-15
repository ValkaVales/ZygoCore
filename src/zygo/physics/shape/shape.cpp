#include "shape.h"


namespace zygo {
namespace phys {

Shape::Shape( ShapeType type, double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color )
  : type      ( type )
  , mass      ( mass )
  , color     ( color )

  , local_pos ( local_pos )
  , size      ( size )
  , local_rot ( local_rot )
{
}

Vector3 Shape::calcLocalCenterOfMass() const
{
  // For uniform box / cylinder / capsule / sphere the center of mass is the geometric center.
  return local_pos;
}

Vector3 Shape::calcWorldCenterOfMass( Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const
{
  return obj_world_pos + obj_world_rot.rotateVector3( local_pos );
}

Quaternion Shape::calcWorldRotation( Quaternion const & obj_world_rot ) const
{
  // world_rot = obj_rot * local_rot
  Quaternion q = obj_world_rot.crossProduct( local_rot );
  q.normalize();
  return q;
}

} // namespace phys
} // namespace zygo
