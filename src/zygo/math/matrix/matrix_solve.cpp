#include "matrix.h"
#include <zygo/core/assert.h>


namespace zygo {

Vector3 Matrix::solve3x3( Vector3 const& b ) const
{
  ZgAssert( dimx == 3 );
  ZgAssert( dimy == 3 );

  // Для простоты через inverse().
  Matrix inv = inverse3x3();
  Vector3 res = inv.multiplyByVector3( b );

  return res;
}

} // namespace zygo
