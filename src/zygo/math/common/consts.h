#pragma once

#include <zygo/core/types.h>


namespace zygo {

#ifdef FLOAT_MNUMBER
constexpr Real MATRIX_EPSILON   = 1e-7f;
constexpr Real EPSILON          = 1e-7f;
constexpr Real SMALL_EPSILON    = 1e-20f;
constexpr Real BIG_EPSILON      = 1e-9f;
constexpr Real ASSERT_EPSILON   = 1e-5f;
constexpr Real PID_EPSILON      = 1e-4f;
constexpr Real QUATERNION_SPHERICAL_INTERPOLATION_THRESHOLD_EPSILON = 2e-5f;

constexpr Real PI           = 3.1415926535897932384626433832795028841971693993751f;
constexpr Real PI_DIV_2     = PI * 0.5f;
constexpr Real PI_MUL_2     = PI * 2.0f;
constexpr Real COS_OF_1DIV2 = 0.877582561890372716130286068203503191f;

constexpr Real DEG_PI       = 180.0f;
constexpr Real DEG_PI_MUL_2 = 360.0f;

constexpr Real SQRT2        = 1.4142135623730950488016887242096980785696718753769f;
constexpr Real SQRT3        = 1.7320508075688772935274463415058723669428052538103f;
#else

constexpr Real MATRIX_EPSILON   = 1e-15;
constexpr Real EPSILON          = 1e-15;
constexpr Real SMALL_EPSILON    = 1e-50;
constexpr Real BIG_EPSILON      = 1e-12;
constexpr Real ASSERT_EPSILON   = 1e-10;
constexpr Real PID_EPSILON      = 1e-7;
constexpr Real QUATERNION_SPHERICAL_INTERPOLATION_THRESHOLD_EPSILON = 1e-8;

constexpr Real PI           = 3.1415926535897932384626433832795028841971693993751;
constexpr Real PI_DIV_2     = PI * 0.5;
constexpr Real PI_MUL_2     = PI * 2.0;
constexpr Real COS_OF_1DIV2 = 0.877582561890372716130286068203503191;

constexpr Real DEG_PI       = 180.0;
constexpr Real DEG_PI_MUL_2 = 360.0;

constexpr Real SQRT2        = 1.4142135623730950488016887242096980785696718753769;
constexpr Real SQRT3        = 1.7320508075688772935274463415058723669428052538103;
#endif


constexpr Real CONST_RAD2DEG = DEG_PI / PI;
constexpr Real CONST_DEG2RAD = PI / DEG_PI;


#define DEGTORAD(x) ((x) * CONST_DEG2RAD)
#define RADTODEG(x) ((x) * CONST_RAD2DEG)

#define DEG2RAD(x) ((x) * CONST_DEG2RAD)
#define RAD2DEG(x) ((x) * CONST_RAD2DEG)

} // namespace zygo
