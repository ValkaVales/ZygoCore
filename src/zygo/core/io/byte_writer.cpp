#include "byte_writer.h"
#include <fstream>
#include <cstring> // memcpy, strlen


namespace zygo {

ByteWriter::ByteWriter( Endian endian_ )
  : external_buf ( nullptr )
  , ext_capacity ( 0u )
  , ext_pos      ( 0u )
  , endian       ( endian_ )
  , is_ok        ( true )
{
}

ByteWriter::ByteWriter( u8 * buf, u32 capacity, Endian endian_ )
  : external_buf ( buf )
  , ext_capacity ( capacity )
  , ext_pos      ( 0u )
  , endian       ( endian_ )
  , is_ok        ( buf != nullptr )
{
}

void ByteWriter::reserve( u32 bytes )
{
  if ( !external_buf )
    owned.reserve( bytes );
}

bool ByteWriter::saveToFile( char const * filename ) const
{
  std::ofstream f( filename, std::ofstream::binary );
  if ( !f.is_open() )
    return false;

  f.write( (char const *)data(), (std::streamsize)size() );
  return (bool)f;
}

// ------------------------------------------------------------------- core
bool ByteWriter::put( void const * src, u32 count )
{
  if ( !is_ok )
    return false;

  if ( count == 0 )
    return true;

  if ( external_buf )
  {
    // overflow-safe form of: ext_pos + count > ext_capacity
    if ( ext_pos + count > ext_capacity )
    {
      is_ok = false;
      return false;
    }

    std::memcpy( external_buf + ext_pos, src, count );
    ext_pos += count;
    return true;
  }

  u8 const* bytes = (u8 const*)src;
  owned.insert( owned.end(), bytes, bytes + count );
  return true;
}

template<typename T>
void ByteWriter::writeScalar( T v )
{
  if ( endian == Endian::Big )
    v = swapBytes( v );

  put( &v, (u32)sizeof( T ) );
}

// ------------------------------------------------------------------- scalars
void ByteWriter::writeI8( i8 v )
{
  put( &v, 1 );
}

void ByteWriter::writeU8( u8 v )
{
  put( &v, 1 );
}

void ByteWriter::writeBool( bool v )
{
  writeU8( v ? (u8)0xaa : (u8)0x00 );
}


void ByteWriter::writeI16( i16 v ) { writeScalar( v ); }
void ByteWriter::writeU16( u16 v ) { writeScalar( v ); }

void ByteWriter::writeI32( i32 v ) { writeScalar( v ); }
void ByteWriter::writeU32( u32 v ) { writeScalar( v ); }

void ByteWriter::writeI64( i64 v ) { writeScalar( v ); }
void ByteWriter::writeU64( u64 v ) { writeScalar( v ); }


void ByteWriter::writeFloat( float v )
{
  u32 raw;
  std::memcpy( &raw, &v, sizeof( v ) );
  writeU32( raw ); // endianness handled there
}

void ByteWriter::writeDouble( double v )
{
  u64 raw;
  std::memcpy( &raw, &v, sizeof( v ) );
  writeU64( raw ); // endianness handled there
}

// ------------------------------------------------------------------- strings
void ByteWriter::writeString( char const * str )
{
  if ( str == nullptr )
  {
    is_ok = false;
    return;
  }

  put( str, (u32)std::strlen( str ) + 1 ); // including the terminating zero
}

// ------------------------------------------------------------------- vectors
void ByteWriter::writeVectorU8( std::vector<u8> const & v )
{
  if ( v.size() > 0xff )
  {
    is_ok = false;
    return;
  }

  writeU8( (u8)v.size() );
  put( v.data(), (u32)v.size() );
}

void ByteWriter::writeVectorU16( std::vector<u16> const & v )
{
  if ( v.size() > 0xffff )
  {
    is_ok = false;
    return;
  }

  writeU16( (u16)v.size() );

  for ( u16 x : v )
    writeU16( x );
}

void ByteWriter::writeVectorU32( std::vector<u32> const & v )
{
  if ( v.size() > 0xffff ) // u16 length field
  {
    is_ok = false;
    return;
  }

  writeU16( (u16)v.size() );

  for ( u32 x : v )
    writeU32( x );
}

} // namespace zygo
