#pragma once

#include "types.h"


namespace zygo {

void my_memset( u8 * dst, u8  value, u32 size );
void my_memset( int* dst, int value, u32 size );

void my_memcpy( u8* dst, u8 const* src, u32 size );

} // namespace zygo
