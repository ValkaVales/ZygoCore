#pragma once

#include "vec2.h"
#include "vec3.h"
#include "vec4.h"
#include "vec3int.h"
#include "vec2int.h"
#include "vec3short.h"
#include "vec2short.h"


namespace zygo {

Vector2 vector3ToVector2_xz ( Vector3 const & v );
Vector3 vector2ToVector3_xz ( Vector2 const & v );

Vector3 vector4ToVector3( Vector4 const & v );
Vector4 vector3ToVector4( Vector3 const & v );


Vector3Int   vector2IntToVector3Int    ( Vector2Int   const& v, int   t = 0 );
Vector3Short vector2ShortToVector3Short( Vector2Short const& v, short t = 0 );

bool xyEquals_Vector3Short_Vector2Short( Vector3Short const& v3, Vector2Short const& v2 );

} // namespace zygo
