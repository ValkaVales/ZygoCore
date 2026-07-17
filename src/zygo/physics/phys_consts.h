#pragma once

// True constants of the physics engine: unit conversions and float-precision guards.
// Everything tunable at runtime lives in SolverSettings (solver_settings.h).

#include <zygo/math/common/scalar.h>


namespace zygo {
namespace phys {

// Unit conversion for the construction API (grams/millimeters -> SI).
const double GRAMS_IN_KG          = 1000.0;
const double MILLIMETERS_IN_METER = 1000.0;

// Float-precision guards (NOT tuning - do not change these).
const double PHYS_EPSILON     = 1e-15;
const double PHYS_EPSILON_SQR = sqr( PHYS_EPSILON );

} // namespace phys
} // namespace zygo
