#include "cross_vectors_funcs.h"


namespace zygo {

Vector2 vector3ToVector2_xz( Vector3 const & v )
{
  return Vector2( v.getX(), v.getZ() );
}

Vector3 vector2ToVector3_xz( Vector2 const & v )
{
  return Vector3( v.x, REAL_ZERO, v.y );
}


Vector3 vector4ToVector3( Vector4 const & v )
{
  return Vector3( v.getX(), v.getY(), v.getZ() );
}

Vector4 vector3ToVector4( Vector3 const & v )
{
  return Vector4( v.getX(), v.getY(), v.getZ(), REAL_ZERO );
}


Vector3Int vector2IntToVector3Int( Vector2Int const& v, int t )
{
  return Vector3Int( v.x, v.y, t );
}

Vector3Short vector2ShortToVector3Short( Vector2Short const& v, short t )
{
  return Vector3Short( v.x, v.y, t );
}


bool xyEquals_Vector3Short_Vector2Short( Vector3Short const& v3, Vector2Short const& v2 )
{
  return
    v3.x == v2.x &&
    v3.y == v2.y;
}

} // namespace zygo
