#include "str.h"


namespace zygo {

bool isCharLowCase( u8 c )
{
  return c >= 'a' && c <= 'z';
}

bool isCharUpperCase( u8 c )
{
  return c >= 'A' && c <= 'Z';
}

bool isChar( u8 c )
{
  return isCharLowCase(c) || isCharUpperCase(c);
}

bool isDigit( u8 c )
{
  return c >= '0' && c <= '9';
}

bool isDigitOrLetter( u8 c )
{
  return isDigit( c )
    || isChar( c );
}

bool isVisualSymbol( u8 c )
{
  static u8 symbols[] = " !@#$%^&*()-_=+`~[]{};:'\"/\\,<.>|?";

  u8 const * p = symbols;
  while ( true )
  {
    char c1 = *p++;
    if ( !c1 )
      return false;

    if ( c1 == c )
      return true;
  }
}

bool isAnyVisualSymbol( u8 c )
{
  return isChar         ( c )
      || isDigit        ( c )
      || isVisualSymbol ( c );
}

char toUpperCaseNoCheck( char c )
{
  return c - 'a' + 'A';
}

char toLowerCaseNoCheck( char c )
{
  return c - 'A' + 'a';
}

char toUpperCase( char c )
{
  return isCharLowCase( (u8)c ) ? toUpperCaseNoCheck(c) : c; // non-letters pass through unchanged
}

char toLowerCase( char c )
{
  return isCharUpperCase( (u8)c ) ? toLowerCaseNoCheck(c) : c;
}

void toUpperCase( char* s, u16 max_len )
{
  for ( u16 i = 0; i < max_len; ++i )
  {
    char c = *s;
    if ( !c )
      return;

    if ( isCharLowCase( c ) )
      *s++ = toUpperCaseNoCheck( c );
    else
      ++s;
  }
}

void toLowerCase( char* s, u16 max_len )
{
  for ( u16 i = 0; i < max_len; ++i )
  {
    char c = *s;
    if ( !c )
      return;

    if ( isCharUpperCase( c ) )
      *s++ = toLowerCaseNoCheck( c );
    else
      ++s;
  }
}


u16 my_strlen( char const * str, u16 max_len )
{
  if ( !str )
    return 0;

  char const * s = str;
  u16 i = 0;
  u16 k = max_len;

  while ( *(s++) )
  {
    if ( ++i >= k )
      break;
  }

  return i;
}

i8 my_strncmp( char const * s1, char const * s2, u16 len )
{
  while ( len-- )
  {
    char c1 = *s1++;
    char c2 = *s2++;

    if ( c1 > c2 )
      return 1;

    if ( c1 < c2 )
      return -1;
  }

  return 0;
}

void my_strncpy( char * s1, char const * s2, u16 len )
{
  while ( len-- )
  {
    if ( !(*s1++ = *s2++) )
      return;
  }
}

void my_strncat( char * s1, u16 len1, char const * s2, u16 len2 )
{
  u16 k1 = my_strlen( s1, len1 );
  if ( k1 >= len1 ) // no space left
    return;

  u16 k2 = my_strlen( s2, len2 );

  s1 += k1;
  len1 -= k1;

  while ( len1-- && k2-- )
  {
    *s1++ = *s2++;
  }

  if ( k2 == (u16)(-1) )
    *s1 = 0;
}

} // namespace zygo
