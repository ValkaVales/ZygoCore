#pragma once

#include "types.h"


namespace zygo {

const u16 MAX_STR_LEN = 16 * 1024; // it can be 32 * 1024 or even 64 * 1024


bool isCharLowCase    ( u8 c );
bool isCharUpperCase  ( u8 c );
bool isChar           ( u8 c );
bool isDigit          ( u8 c );
bool isDigitOrLetter  ( u8 c );
bool isVisualSymbol   ( u8 c );
bool isAnyVisualSymbol( u8 c );

char toUpperCase( char c );
char toLowerCase( char c );

void toUpperCase( char* s, u16 max_len = MAX_STR_LEN );
void toLowerCase( char* s, u16 max_len = MAX_STR_LEN );


u16 my_strlen( char const * s, u16 max_len = MAX_STR_LEN );
i8 my_strncmp( char const * s1, char const * s2, u16 len ); // returns 0, if match
void my_strncpy( char * s1, char const * s2, u16 len );
void my_strncat( char * s1, u16 len, char const * s2, u16 len2 );

} // namespace zygo
