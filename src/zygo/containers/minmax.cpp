#include "minmax.h"
#include <zygo/math/common/scalar.h>


namespace zygo {

MinMax::MinMax()
  : min (  REAL_BIG_VALUE )
  , max ( -REAL_BIG_VALUE )
{
}

MinMax::MinMax( Real min, Real max )
  : min ( min )
  , max ( max )
{
}

void MinMax::upd( Real val )
{
  updMax( max, val );
  updMin( min, val );
}

void MinMax::upd( MinMax val )
{
  updMax( max, val.max );
  updMin( min, val.min );
}

} // namespace zygo
