#pragma once

#include <zygo/core/io/byte_reader.h>
#include <zygo/core/io/byte_writer.h>
#include <zygo/math/vector/vec2short.h>
#include <zygo/math/vector/vec3short.h>


namespace zygo {

inline void serializeVec2Short( ByteWriter& serializer, Vector2Short const& v )
{
  serializer.writeI16( v.x );
  serializer.writeI16( v.y );
}

inline void deserializeVec2Short( ByteReader& ds, Vector2Short& v )
{
  v.x = ds.readI16();
  v.y = ds.readI16();
}


inline void serializeVec3Short( ByteWriter& serializer, Vector3Short const& v )
{
  serializer.writeI16( v.x );
  serializer.writeI16( v.y );
  serializer.writeI16( v.z );
}

inline void deserializeVec3Short( ByteReader& ds, Vector3Short& v )
{
  v.x = ds.readI16();
  v.y = ds.readI16();
  v.z = ds.readI16();
}

} // namespace zygo
