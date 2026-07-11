#pragma once

#include <zygo/math/vector/vec3.h>


namespace zygo {

struct TriangleWithNormal
{
  Vector3 normal;
  Vector3 points[3];
};

} // namespace zygo
