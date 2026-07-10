#include "hex.h"
#include "digit.h"


namespace zygo {

const int HEX_RADIX = 16;


void hexToBytes( char const* text, u8* res, int text_len, int res_capacity ) // was hexStringToBytesArray
{
  int n2 = res_capacity;
  char const* p = text;

  for ( int i = 0; i < n2; i++ )
  {
    int b1 = (text_len-- > 0) ? digitToValue( *p++, HEX_RADIX ) : 0;
    int b2 = (text_len-- > 0) ? digitToValue( *p++, HEX_RADIX ) : 0;

    res[i] = (b1 << 4) + b2;
  }
}

void bytesToHex( u8 const* bytes, char* res, int bytes_count, int res_capacity ) // was bytesArrayToHexString
{
  char* p = res;

  for ( int i = 0; i < bytes_count; i++ )
  {
    u8 x = bytes[i];
    u8 a1 = x >> 4;
    u8 a2 = x & 0xf;

    if ( res_capacity > 1 )
    {
      *p++ = valueToDigit_upperCase( a1, HEX_RADIX );
      *p++ = valueToDigit_upperCase( a2, HEX_RADIX );
    }
    res_capacity -= 2;
  }

  *p = 0;
}

} // namespace zygo
