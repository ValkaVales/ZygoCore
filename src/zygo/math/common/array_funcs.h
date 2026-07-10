#pragma once

#include <zygo/core/types.h>
#include <vector>


namespace zygo {

Real  calcSquaresSum( Real const* arr, int size ) noexcept; // was calcSqrsSum
int   argMax        ( Real const* arr, int size ) noexcept; // was maxValueIndex

std::vector<Real> calcSoftMax( Real const* arr, int size );

} // namespace zygo
