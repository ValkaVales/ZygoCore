#pragma once

#include <zygo/core/types.h>


namespace zygo {

void hexToBytes( char const* text, u8* res, int text_len, int res_capacity ); // was hexStringToBytesArray
void bytesToHex( u8 const* bytes, char* res, int bytes_count, int res_capacity ); // was bytesArrayToHexString

} // namespace zygo
