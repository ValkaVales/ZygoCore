#pragma once

#include <cstdint>


namespace zygo {

using u64 = std::uint64_t;
using u32 = std::uint32_t;
using u16 = std::uint16_t;
using u8  = std::uint8_t;

using i64 = std::int64_t;
using i32 = std::int32_t;
using i16 = std::int16_t;
using i8  = std::int8_t;

using ulong  = unsigned long;
using uint   = unsigned int;
using ushort = unsigned short;
using uchar  = unsigned char;


const int U16_SZ      = 2;
const int U32_SZ      = 4;
const int U64_SZ      = 8;
const int DOUBLE_SZ   = sizeof( double );


#define FLOAT_MNUMBER // Comment this to use double, uncomment - to use float


#ifdef FLOAT_MNUMBER
typedef float Real;

const Real REAL_ZERO      = 0.0f;
const Real REAL_ONE       = 1.0f;
const Real REAL_TWO       = 2.0f;
const Real REAL_HALF      = 0.5f;
const Real REAL_BIG_VALUE = 10e15f;
#else
typedef double Real;

const Real REAL_ZERO      = 0.0;
const Real REAL_ONE       = 1.0;
const Real REAL_TWO       = 2.0;
const Real REAL_HALF      = 0.5;
const Real REAL_BIG_VALUE = 10e20f;
#endif


/*
#ifdef FLOAT_MNUMBER
#define sqrtn   sqrtf
#define expn    expf
#define logn    logf
#define cosn    cosf
#define sinn    sinf
#define tann    tanf
#define atann   atanf
#define asinn   asinf
#define acosn   acosf
#define atan2n  atan2f
#define roundn  roundf
#define floorn  floorf
#define ceiln   ceilf
#define fmodn   fmodf
#define fabsn   fabsf
#else
#define sqrtn   sqrt
#define expn    exp
#define logn    log
#define cosn    cos
#define sinn    sin
#define tann    tan
#define atann   atan
#define asinn   asin
#define acosn   acos
#define atan2n  atan2
#define roundn  round
#define floorn  floor
#define ceiln   ceil
#define fmodn   fmod
#define fabsn   fabs
#endif
*/

} // namespace zygo
