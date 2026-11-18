#include "print_matrix.h"
#include <zygo/core/str.h>
#include <zygo/math/common/scalar.h>
#include <stdio.h>


namespace zygo {

void printMatrix( Matrix const& m, char const * name, int max_dim )
{
#ifdef DEBUG_MATRIX
  if ( !name )
    name = m.getName();
#endif

  if ( name )
    printf( "-------------------- matrix %s   [%d, %d]\n", name, m.dimY(), m.dimX() );
  else
    printf( "-------------------- matrix  [%d, %d]\n", m.dimY(), m.dimX() );

  const int LEN1 = 200;
  const int LEN2 = 20;

  int dy = max_dim ? min2( m.dimY(), max_dim ) : m.dimY();
  int dx = max_dim ? min2( m.dimX(), max_dim ) : m.dimX();

  for ( int j = 0; j < dy; ++j )
  {
    char buf1[LEN1];
    char buf2[LEN2];
    buf1[0] = 0;

    for ( int i = 0; i < dx; ++i )
    {
      Real v = m.get( j, i );
      snprintf( buf2, LEN2, "%0.4f  ", v );
      my_strncat( buf1, LEN1, buf2, LEN2 );
    }

    printf( "%s\n", buf1 );
  }

  printf( "\n" );
}

} // namespace zygo
