#include "byte_reader.h"
#include <fstream>
#include <cstring> // memcpy


namespace zygo {

ByteReader::ByteReader( u8 const * buf, u32 buf_sz_, Endian endian_ )
  : data    ( buf )
  , buf_sz  ( buf_sz_ )
  , cur_pos ( 0u )
  , endian  ( endian_ )
  , is_ok   ( buf != nullptr )
{
}

ByteReader::ByteReader( char const * filename, Endian endian_ )
  : data    ( nullptr )
  , buf_sz  ( 0u )
  , cur_pos ( 0u )
  , endian  ( endian_ )
  , is_ok   ( false )
{
  std::ifstream f( filename, std::ifstream::binary );
  if ( !f.is_open() )
    return;

  f.seekg( 0, f.end );
  std::streamoff sz = f.tellg();
  f.seekg( 0, f.beg );

  if ( sz <= 0 )
    return;

  owned.resize( (size_t)sz );
  f.read( (char*)owned.data(), sz );

  if ( !f )
    return; // read failed halfway

  data   = owned.data();
  buf_sz = (u32)sz;
  is_ok  = true;
}

// ------------------------------------------------------------------- core
bool ByteReader::grab( void * dst, u32 count )
{
  if ( !is_ok )
    return false;

  // overflow-safe form of: cur_pos + count > buf_sz
  if ( count > buf_sz - cur_pos )
  {
    is_ok = false;
    return false;
  }

  // memcpy instead of *(u32*)(data + cur_pos): unaligned pointer casts are UB
  // and crash on ARM; on x86 the compiler emits the exact same mov anyway
  std::memcpy( dst, data + cur_pos, count );
  cur_pos += count;
  return true;
}

template<typename T>
T ByteReader::readScalar()
{
  T v = T();

  if ( !grab( &v, (u32)sizeof(T) ) )
    return v;

  if ( endian == Endian::Big && sizeof(T) > 1 )
    v = swapBytes( v );

  return v;
}

bool ByteReader::skip( u32 count )
{
  if ( !is_ok || count > buf_sz - cur_pos )
  {
    is_ok = false;
    return false;
  }

  cur_pos += count;
  return true;
}

// ------------------------------------------------------------------- scalars
i8  ByteReader::readI8()
{
  i8 v = 0;
  grab( &v, 1 );
  return v;
}

u8  ByteReader::readU8()
{
  u8 v = 0;
  grab( &v, 1 );
  return v;
}

bool ByteReader::readBool()
{
  return readU8() != 0; // MySerializer wrote 0xaa / 0x00
}

i16 ByteReader::readI16() { return readScalar<i16>(); }
u16 ByteReader::readU16() { return readScalar<u16>(); }

i32 ByteReader::readI32() { return readScalar<i32>(); }
u32 ByteReader::readU32() { return readScalar<u32>(); }

i64 ByteReader::readI64() { return readScalar<i64>(); }
u64 ByteReader::readU64() { return readScalar<u64>(); }

float ByteReader::readFloat()
{
  u32 raw = readU32(); // endianness handled there

  float v;
  std::memcpy( &v, &raw, sizeof(v) );
  return v;
}

double ByteReader::readDouble()
{
  u64 raw = readU64(); // endianness handled there

  double v;
  std::memcpy( &v, &raw, sizeof(v) );
  return v;
}

// ------------------------------------------------------------------- strings
std::string ByteReader::readString()
{
  std::string res;

  while ( is_ok )
  {
    u8 c = readU8();
    if ( c == 0 )
      break; // terminator reached, or the stream got poisoned

    res += (char)c;
  }

  return res;
}

/*
bool ByteReader::checkStrCommand( char const * command )
{
  std::string s = readString();
  return is_ok && s == command;
}
*/

// ------------------------------------------------------------------- vectors
void ByteReader::readVectorU8( std::vector<u8> & out )
{
  out.clear();

  u32 count = readU8();
  if ( !is_ok || count > available() )
  {
    is_ok = false;
    return;
  }

  out.resize( count );
  grab( out.data(), count );
}

void ByteReader::readVectorU16( std::vector<u16> & out )
{
  out.clear();

  u32 count = readU16();
  if ( !is_ok || count * 2 > available() ) // reject corrupted length early
  {
    is_ok = false;
    return;
  }

  out.reserve( count );
  for ( u32 i = 0; i < count; ++i )
    out.push_back( readU16() );
}

void ByteReader::readVectorU32( std::vector<u32> & out )
{
  out.clear();

  u32 count = readU16(); // yes, u16: that is ByteReader's wire format
  if ( !is_ok || count * 4 > available() )
  {
    is_ok = false;
    return;
  }

  out.reserve( count );
  for ( u32 i = 0; i < count; ++i )
    out.push_back( readU32() );
}

} // namespace zygo
