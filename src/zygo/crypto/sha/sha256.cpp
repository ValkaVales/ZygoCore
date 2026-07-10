#include "sha256.h"
#include <cstring> // memcpy, memset


namespace zygo {

// Round constants (FIPS 180-4, section 4.2.2):
// fractional parts of the cube roots of the first 64 primes.
static const u32 K[64] =
{
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};


// ------------------------- FIPS 180-4, section 4.1.2: logical functions
static inline u32 rotr( u32 x, int n ) { return (x >> n) | (x << (32 - n)); }

static inline u32 ch ( u32 x, u32 y, u32 z ) { return (x & y) ^ (~x & z); }
static inline u32 maj( u32 x, u32 y, u32 z ) { return (x & y) ^ (x & z) ^ (y & z); }

static inline u32 bigSigma0  ( u32 x ) { return rotr( x,  2 ) ^ rotr( x, 13 ) ^ rotr( x, 22 ); }
static inline u32 bigSigma1  ( u32 x ) { return rotr( x,  6 ) ^ rotr( x, 11 ) ^ rotr( x, 25 ); }
static inline u32 smallSigma0( u32 x ) { return rotr( x,  7 ) ^ rotr( x, 18 ) ^ (x >>  3); }
static inline u32 smallSigma1( u32 x ) { return rotr( x, 17 ) ^ rotr( x, 19 ) ^ (x >> 10); }

// SHA works in big-endian byte order
static inline u32 loadBE32( u8 const * p )
{
  return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | (u32)p[3];
}

static inline void storeBE32( u8 * p, u32 v )
{
  p[0] = (u8)(v >> 24);
  p[1] = (u8)(v >> 16);
  p[2] = (u8)(v >>  8);
  p[3] = (u8)(v      );
}


// ------------------------------------------------------------------- Sha256
Sha256::Sha256()
{
  reset();
}

void Sha256::reset()
{
  // initial hash values (section 5.3.3):
  // fractional parts of the square roots of the first 8 primes
  state[0] = 0x6a09e667;
  state[1] = 0xbb67ae85;
  state[2] = 0x3c6ef372;
  state[3] = 0xa54ff53a;
  state[4] = 0x510e527f;
  state[5] = 0x9b05688c;
  state[6] = 0x1f83d9ab;
  state[7] = 0x5be0cd19;

  buffer_len = 0;
  total_len  = 0;
}

// one 64-byte block through the compression function (section 6.2.2)
void Sha256::processBlock( u8 const * block )
{
  u32 w[64]; // message schedule

  for ( int t = 0; t < 16; t++ )
    w[t] = loadBE32( block + t * 4 );

  for ( int t = 16; t < 64; t++ )
    w[t] = smallSigma1( w[t - 2] ) + w[t - 7] + smallSigma0( w[t - 15] ) + w[t - 16];

  u32 a = state[0];
  u32 b = state[1];
  u32 c = state[2];
  u32 d = state[3];
  u32 e = state[4];
  u32 f = state[5];
  u32 g = state[6];
  u32 h = state[7];

  for ( int t = 0; t < 64; t++ )
  {
    u32 t1 = h + bigSigma1( e ) + ch( e, f, g ) + K[t] + w[t];
    u32 t2 = bigSigma0( a ) + maj( a, b, c );

    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;
}

void Sha256::update( u8 const * data, u32 len )
{
  total_len += len;

  // top up a partially filled buffer first
  if ( buffer_len > 0 )
  {
    u32 room = (u32)BLOCK_SIZE - buffer_len;
    u32 take = len < room ? len : room;

    std::memcpy( buffer + buffer_len, data, take );
    buffer_len += take;
    data       += take;
    len        -= take;

    if ( buffer_len == (u32)BLOCK_SIZE )
    {
      processBlock( buffer );
      buffer_len = 0;
    }
  }

  // whole blocks go straight from the input, without copying
  while ( len >= (u32)BLOCK_SIZE )
  {
    processBlock( data );
    data += BLOCK_SIZE;
    len  -= BLOCK_SIZE;
  }

  // stash the tail (buffer is empty here: see the first branch)
  if ( len > 0 )
  {
    std::memcpy( buffer, data, len );
    buffer_len = len;
  }
}

void Sha256::calcFinal( u8 * digest )
{
  u64 total_bits = total_len * 8;

  // padding (section 5.1.1): single 0x80 byte, zeros,
  // then the message length in bits as a 64-bit big-endian number
  buffer[buffer_len++] = 0x80; // there is always room: buffer_len < BLOCK_SIZE

  if ( buffer_len > (u32)BLOCK_SIZE - 8 ) // no room for the length field
  {
    std::memset( buffer + buffer_len, 0, (u32)BLOCK_SIZE - buffer_len );
    processBlock( buffer );
    buffer_len = 0;
  }

  std::memset( buffer + buffer_len, 0, (u32)BLOCK_SIZE - 8 - buffer_len );
  storeBE32( buffer + BLOCK_SIZE - 8, (u32)(total_bits >> 32) );
  storeBE32( buffer + BLOCK_SIZE - 4, (u32)(total_bits      ) );
  processBlock( buffer );

  for ( int i = 0; i < 8; i++ )
    storeBE32( digest + i * 4, state[i] );

  reset(); // ready for the next message
}

void Sha256::hash( u8 const * data, u32 len, u8 * digest )
{
  Sha256 h;
  h.update( data, len );
  h.calcFinal( digest );
}

} // namespace zygo