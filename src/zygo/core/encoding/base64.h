#pragma once

#include <zygo/core/types.h>


namespace zygo {

bool isCharBase64( u8 c );

u8 findInBase64Chars( char c0 );


// The output buffer size is validated only in debug builds.
// Bounds checks are intentionally omitted from the release inner loop to avoid per-byte overhead.
// The caller must provide a buffer large enough for the complete encoded result, including the null terminator when one is written.
// Violating this precondition results in undefined behavior.
void base64ToBytesArray( char const* text, u8* res, int text_len, int res_capacity );
void bytesArrayToBase64( u8 const* bytes, char* res, int bytes_count, int res_capacity );

} // namespace zygo
