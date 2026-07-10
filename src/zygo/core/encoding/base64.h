#pragma once

#include <zygo/core/types.h>


namespace zygo {

bool isCharBase64( u8 c );

u8 findInBase64Chars( char c0 );

void base64ToBytesArray( char const* text, u8* res, int text_len, int res_capacity );
void bytesArrayToBase64( u8 const* bytes, char* res, int bytes_count, int res_capacity );

} // namespace zygo
