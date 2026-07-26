#pragma once

// True constants of the physics engine: unit conversions and float-precision guards.
// Everything tunable at runtime lives in SolverSettings (solver_settings.h).

#include <zygo/math/common/scalar.h>


namespace zygo {

// Unit conversion for the construction API (grams/millimeters -> SI).
const Real GRAMS_IN_KG          = Real(1000.0);
const Real MILLIMETERS_IN_METER = Real(1000.0);

namespace phys {

// Float-precision guards (NOT tuning - do not change these).
#ifdef FLOAT_MNUMBER
const float PHYS_EPSILON     = 1e-10f;
const float PHYS_EPSILON_SQR = sqr( PHYS_EPSILON );
#else
const double PHYS_EPSILON     = 1e-15;
const double PHYS_EPSILON_SQR = sqr( PHYS_EPSILON );
#endif

} // namespace phys
} // namespace zygo
