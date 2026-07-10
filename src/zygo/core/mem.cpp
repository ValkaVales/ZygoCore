#include "mem.h"


namespace zygo {

void my_memset( u8* dst, u8 value, u32 size )
{
  u8 * p1 = dst;

  while ( size-- )
    *p1++ = value;
}

void my_memset( int* dst, int value, u32 size )
{
  int * p1 = dst;

  while ( size-- )
    *p1++ = value;
}

void my_memcpy( u8* dst, u8 const* src, u32 size )
{
  u8 * p1 = dst;
  u8 const * p2 = src;

  while ( size-- )
    *p1++ = *p2++;
}

} // namespace zygo
