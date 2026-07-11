#pragma once

#include <zygo/core/io/byte_reader.h>
#include <zygo/core/io/byte_writer.h>
#include <zygo/math/matrix/matrix.h>
#include <zygo/core/assert.h>


namespace zygo {

void serializeMatrix( ByteWriter& sr, Matrix const& m );
void deserializeMatrix( ByteReader& ds, Matrix& m );

} // namespace zygo
