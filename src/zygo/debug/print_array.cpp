#include "print_array.h"
#include <zygo/math/common/scalar.h>
#include <stdio.h>


namespace zygo {

void printArr( char const* name, Real const* arr, int sz )
{
  printf( "%s:   ", name );

  for ( int i = 0; i < sz; ++i )
    printf( "%0.3f  ", *arr++ );

  printf( "\n" );
}

void printArr( char const* name, Real const* arr, int dimy, int dimx )
{
  printf( "%s:\n", name );

  int stride = dimx;

  applyMax( dimy, 10 );
  applyMax( dimx, 15 );

  for ( int j = 0; j < dimy; ++j )
  {
    Real const* tmp = arr;
    for ( int i = 0; i < dimx; ++i )
      printf( "%0.3f  ", *arr++ );

    arr = tmp + stride;
    printf( "\n" );
  }

  printf( "\n" );
}

// TODO:
//void printMatrix( const char* name, const Number* values, int rows, int cols );

} // namespace zygo
