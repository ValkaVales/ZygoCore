#include "base64.h"
#include <zygo/core/str.h>
#include <zygo/core/assert.h>


namespace zygo {

static const char* base64_chars =
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";


bool isCharBase64( u8 c )
{
  return isDigitOrLetter( c )
    || c == '+'
    || c == '/';
}

u8 findInBase64Chars( char c0 )
{
  const char* p = base64_chars;
  for ( u8 i = 0; i < 64; ++i )
  {
    if ( *p++ == c0 )
      return i;
  }
  return -1;
}

void base64ToBytesArray( char const* text, u8* res, int text_len, int res_capacity )
{
  int i = 0;
  int i0 = 0;
  int k = 0;

  u8 char_array_4[4];
  u8 char_array_3[3];

  while ( text_len-- && text[i0] != '=' && isCharBase64( text[i0] ) )
  {
    char_array_4[i++] = text[i0++];
    if ( i == 4 )
    {
      for ( i = 0; i < 4; i++ )
        char_array_4[i] = findInBase64Chars( char_array_4[i] );

      char_array_3[0] = ( char_array_4[0]        << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) +   char_array_4[3];

      for ( i = 0; (i < 3); i++ )
      {
        ZgAssert( k < res_capacity );
        res[k++] = char_array_3[i];
      }

      i = 0;
    }
  }

  if ( i )
  {
    for ( int j = i; j < 4; j++ )
      char_array_4[j] = 0;

    for ( int j = 0; j < 4; j++ )
      char_array_4[j] = findInBase64Chars( char_array_4[j] );

    char_array_3[0] = ( char_array_4[0]        << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) +   char_array_4[3];

    for ( int j = 0; j < i - 1; j++ )
    {
      ZgAssert( k < res_capacity );
      res[k++] = char_array_3[j];
    }
  }
}

void bytesArrayToBase64( u8 const* bytes, char* res, int bytes_count, int res_capacity )
{
  int i = 0;
  int k = 0;
  u8 char_array_3[3];
  u8 char_array_4[4];

  while ( bytes_count-- )
  {
    char_array_3[i++] = *(bytes++);
    if ( i == 3 )
    {
      char_array_4[0] = ( char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] =   char_array_3[2] & 0x3f;

      for ( i = 0; (i < 4); i++ )
      {
        ZgAssert( k < res_capacity );
        res[k++] = base64_chars[char_array_4[i]];
      }

      i = 0;
    }
  }

  if ( i )
  {
    for ( int j = i; j < 3; j++ )
      char_array_3[j] = '\0';

    char_array_4[0] = ( char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] =   char_array_3[2] & 0x3f;

    for ( int j = 0; j < i + 1; j++ )
    {
      ZgAssert( k < res_capacity );
      res[k++] = base64_chars[char_array_4[j]];
    }

    while ( (i++ < 3) )
    {
      ZgAssert( k < res_capacity );
      res[k++] = '=';
    }
  }

  ZgAssert( k < res_capacity );
  res[k++] = 0;
}

} // namespace zygo
