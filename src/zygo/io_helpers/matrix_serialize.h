#pragma once

#include <zygo/core/io/byte_reader.h>
#include <zygo/core/io/byte_writer.h>
#include <zygo/math/matrix/matrix.h>
#include <zygo/core/assert.h>


namespace zygo {

void serializeMatrix( ByteWriter& sr, Matrix const& m )
{
  sr.writeU32( m.getSize() );
  sr.writeU32( m.dimX() );
  sr.writeU32( m.dimY() );

  Real const* p = m.getArr();

  for ( int i = 0; i < m.getSize(); ++i )
    sr.writeDouble( *p++ );
}

void deserializeMatrix( ByteReader& ds, Matrix& m )
{
  int read_size = ds.readU32();
  int read_dimx = ds.readU32();
  int read_dimy = ds.readU32();

  ZgAssertRelease( read_size == m.getSize() );
  ZgAssertRelease( read_dimx == m.dimX() );
  ZgAssertRelease( read_dimy == m.dimY() );

  Real* p = m.getArr();

  for ( int i = 0; i < m.getSize(); ++i )
    *p++ = (Real)ds.readDouble();
}

} // namespace zygo
