#pragma once

#include <zygo/core/types.h>
#include <zygo/core/endian.h>
#include <vector>


namespace zygo {

// Sequential writer of binary data.
// Symmetric to ByteReader: same endian handling, same wire format, same "poisoned stream" error model.
//
// Two modes:
//   ByteWriter w;                  — grow mode: internal buffer, no limit;
//   ByteWriter w( buf, capacity ); — fixed mode: external buffer, overflow poisons the stream.
class ByteWriter
{
private:
  std::vector<u8> owned; // grow mode storage

  u8 * external_buf; // fixed mode buffer (nullptr in grow mode)
  u32  ext_capacity;
  u32  ext_pos;

  Endian endian;
  bool   is_ok;

public:
  explicit ByteWriter( Endian endian = Endian::Little );
  ByteWriter( u8 * buf, u32 capacity, Endian endian = Endian::Little );

  bool isOk() const { return is_ok; }

  u8 const * data() const { return external_buf ? external_buf     : owned.data();      }
  u32        size() const { return external_buf ? ext_pos : (u32)owned.size(); }

  void reserve( u32 bytes ); // grow mode only: pre-allocate

  bool saveToFile( char const * filename ) const;

  void writeI8  ( i8   v );
  void writeU8  ( u8   v );
  void writeBool( bool v ); // 0xaa / 0x00, as MySerializer did

  void writeI16( i16 v );
  void writeU16( u16 v );

  void writeI32( i32 v );
  void writeU32( u32 v );

  void writeI64( i64 v );
  void writeU64( u64 v );

  void writeFloat ( float  v );
  void writeDouble( double v );

  void writeString( char const * str ); // bytes + terminating zero

  // u8 count + payload / u16 count + payload;
  // too long a vector poisons the stream
  void writeVectorU8 ( std::vector<u8 > const & v );
  void writeVectorU16( std::vector<u16> const & v );
  void writeVectorU32( std::vector<u32> const & v );

private:
  // The ONLY place that touches the buffers: bounds check / grow + copy.
  bool put( void const * src, u32 count );

  template<typename T>
  void writeScalar( T v ); // swapBytes() if big-endian + put()
};

} // namespace zygo
