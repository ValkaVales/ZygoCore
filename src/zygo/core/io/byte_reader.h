#pragma once

#include <zygo/core/types.h>
#include <zygo/core/endian.h>
#include <vector>
#include <string>


namespace zygo {

// Sequential reader of binary data.
//
// Error model — "poisoned stream": any failure (file not found, read past the end, corrupted length field) sets isOk() to false;
// the failed read and ALL further reads return 0.
// Parse first, check isOk() once at the end instead of checking every single call.
class ByteReader
{
private:
  std::vector<u8> owned; // filled only by the file constructor

  u8 const * data;
  u32 buf_sz;
  u32 cur_pos;

  Endian endian;
  bool   is_ok;

public:
  ByteReader( u8 const * buf, u32 buf_sz, Endian endian = Endian::Little );
  explicit ByteReader( char const * filename, Endian endian = Endian::Little );

  bool isOk     () const { return is_ok           ; }
  u32  size     () const { return buf_sz          ; }
  u32  pos      () const { return cur_pos         ; }
  u32  available() const { return buf_sz - cur_pos; }

  bool skip( u32 count );

  i8   readI8  ();
  u8   readU8  ();
  bool readBool();

  i16  readI16();
  u16  readU16();

  i32  readI32();
  u32  readU32();

  i64  readI64();
  u64  readU64();

  float  readFloat ();
  double readDouble();

  std::string readString(); // zero-terminated in the stream
  //bool checkStrCommand( char const * command );

  void readVectorU8 ( std::vector<u8 > & out ); // u8 count + payload
  void readVectorU16( std::vector<u16> & out ); // u16 count + payload
  void readVectorU32( std::vector<u32> & out ); // u16 count + payload

private:
  // The ONLY place that touches the buffer: bounds check + copy + advance.
  bool grab( void * dst, u32 count );

  template<typename T>
  T readScalar(); // grab() + swapBytes() if big-endian
};

} // namespace zygo
