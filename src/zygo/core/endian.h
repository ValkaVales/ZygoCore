#pragma once

#include "types.h"


namespace zygo {

enum class Endian
{
  Little,
  Big
};


// One implementation per width; signed versions forward to unsigned ones.

inline u16 swapBytesU16( u16 v )
{
  return (u16)( (v << 8) | (v >> 8) );
}

inline u32 swapBytesU32( u32 v )
{
  return ( v                << 24)
       | ((v & 0x0000ff00u) <<  8)
       | ((v & 0x00ff0000u) >>  8)
       | ( v                >> 24);
}

inline u64 swapBytesU64( u64 v )
{
  return ((u64)swapBytesU32( (u32)( v       ) ) << 32)
       |  (u64)swapBytesU32( (u32)( v >> 32 ) );
}

inline i16 swapBytesI16( i16 v ) { return (i16)swapBytesU16( (u16)v ); }
inline i32 swapBytesI32( i32 v ) { return (i32)swapBytesU32( (u32)v ); }
inline i64 swapBytesI64( i64 v ) { return (i64)swapBytesU64( (u64)v ); }


// ----------------------------------------------------------------------- Aliases
inline u16 swapBytes( u16 v ) { return swapBytesU16( v ); }
inline u32 swapBytes( u32 v ) { return swapBytesU32( v ); }
inline u64 swapBytes( u64 v ) { return swapBytesU64( v ); }

inline i16 swapBytes( i16 v ) { return swapBytesI16( v ); }
inline i32 swapBytes( i32 v ) { return swapBytesI32( v ); }
inline i64 swapBytes( i64 v ) { return swapBytesI64( v ); }

} // namespace zygo

