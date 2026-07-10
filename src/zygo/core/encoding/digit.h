#pragma once

#include <zygo/core/types.h>


namespace zygo {

int digitToValue( char c, int radix ); // was charToInt

char valueToDigit_upperCase( u8 k, int radix ); // was intToChar
char valueToDigit_lowerCase( u8 k, int radix ); // was intToChar

} // namespace zygo
