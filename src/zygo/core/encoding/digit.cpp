#include "digit.h"
#include <zygo/core/assert.h>
#include <zygo/core/str.h>


namespace zygo {

int digitToValue( char c, int radix ) // was charToInt
{
  if ( radix <= 10 )
  {
    ZgAssert( isDigit( c ) );
    return c - '0';
  }

  if ( isDigit( c ) )
    return c - '0';

  if ( isCharLowCase( c ) )
    return c - 'a' + 10;

  ZgAssert( isCharUpperCase( c ) );
  return c - 'A' + 10;
}

char valueToDigit_upperCase( u8 k, int radix ) // was intToChar
{
  ZgAssert( k < radix );

  if ( k < 10 )
    return k + '0';

  return k - 10 + 'A';
}

char valueToDigit_lowerCase( u8 k, int radix ) // was intToChar
{
  ZgAssert( k < radix );

  if ( k < 10 )
    return k + '0';

  return k - 10 + 'a';
}

} // namespace zygo
