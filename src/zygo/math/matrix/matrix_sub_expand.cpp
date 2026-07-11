#include "matrix.h"
#include <zygo/core/assert.h>


namespace zygo {

Matrix Matrix::subMatrix( int col_indices[], int row_indices[], int columns_count, int rows_count )
{
  ZgAssert( columns_count > 0 && rows_count > 0 && columns_count <= dimx && rows_count <= dimy );

  Matrix res( rows_count, columns_count );

  for ( int i = 0; i < columns_count; ++i )
  {
    int i1 = col_indices[i];
    ZgAssert( i1 >= 0 && i1 < dimx );

    for ( int j = 0; j < rows_count; ++j )
    {
      int j1 = row_indices[j];
      ZgAssert( j1 >= 0 && j1 < dimy );

      res.setAt( j, i,  get( j1, i1 ) );
    }
  }

  return res;
}

Matrix Matrix::expandMatrix( int col_indices[], int row_indices[], int new_dimy, int new_dimx ) // columns_count should be equal to dimy, rows_count should be equal to dimx
{
  ZgAssert( new_dimx > dimx && new_dimy > dimy );

  Matrix res( new_dimy, new_dimx );

#ifndef DEBUG_MATRIX
  res.makeAllZero(); // call this only in Release, because in case of DEBUG_MATRIX, all its values already are zero
#endif

  for ( int i = 0; i < dimx; ++i )
  {
    int i1 = col_indices[i];
    ZgAssert( i1 >= 0 && i1 < new_dimx );

    for ( int j = 0; j < dimy; ++j )
    {
      int j1 = row_indices[j];
      ZgAssert( j1 >= 0 && j1 < new_dimy );

      res.setAt( j1, i1,  get( j, i ) );
    }
  }

  return res;
}

} // namespace zygo
