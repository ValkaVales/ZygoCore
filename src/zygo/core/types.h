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


constexpr int U16_SZ      = 2;
constexpr int U32_SZ      = 4;
constexpr int U64_SZ      = 8;
constexpr int DOUBLE_SZ   = sizeof( double );


//#define FLOAT_NUMBER // Comment this to use double, uncomment - to use float


#ifdef FLOAT_NUMBER
typedef float Real;

constexpr Real REAL_ZERO      = 0.0f;
constexpr Real REAL_ONE       = 1.0f;
constexpr Real REAL_TWO       = 2.0f;
constexpr Real REAL_HALF      = 0.5f;
constexpr Real REAL_BIG_VALUE = 1e15f;
#else
typedef double Real;

constexpr Real REAL_ZERO      = 0.0;
constexpr Real REAL_ONE       = 1.0;
constexpr Real REAL_TWO       = 2.0;
constexpr Real REAL_HALF      = 0.5;
constexpr Real REAL_BIG_VALUE = 1e20;
#endif

} // namespace zygo
