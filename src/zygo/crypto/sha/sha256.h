#pragma once

#include <zygo/core/types.h>


namespace zygo {

// SHA-256, implemented from the specification (FIPS 180-4):
// https://nvlpubs.nist.gov/nistpubs/FIPS/NIST.FIPS.180-4.pdf
//
// Usage:
//   Sha256 h;
//   h.update( data, size ); // any number of times, any chunk sizes
//   u8 digest[Sha256::DIGEST_SIZE];
//   h.calcFinal( digest );  // also resets, object is ready for a new message
//
// or in one call:
//   Sha256::hash( data, size, digest );
class Sha256
{
public:
  static const int DIGEST_SIZE = 32; // bytes in the resulting hash
  static const int BLOCK_SIZE  = 64; // bytes per processed block

private:
  u32 state[8];
  u8  buffer[BLOCK_SIZE]; // incomplete block, waiting for more data
  u32 buffer_len;         // bytes accumulated in buffer, always < BLOCK_SIZE
  u64 total_len;          // total bytes hashed so far

public:
  Sha256();

  void reset();

  void update   ( u8 const * data, u32 len );
  void calcFinal( u8 * digest );

  static void hash( u8 const * data, u32 len, u8 * digest );

private:
  void processBlock( u8 const * block );
};

} // namespace zygo
