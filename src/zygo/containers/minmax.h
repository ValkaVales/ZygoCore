#pragma once

#include <zygo/core/types.h>


namespace zygo {

struct MinMax
{
  Real min;
  Real max;

  MinMax();
  MinMax( Real min, Real max = -REAL_BIG_VALUE );

  void upd( Real val );
  void upd( MinMax val );

  inline Real diff() const { return max - min; }
};

} // namespace zygo
