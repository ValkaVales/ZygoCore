#include "sha160.h"
#include <zygo/core/mem.h>


namespace zygo {

#define ROTLEFT(a,b) (((a) << (b)) | ((a) >> (32-(b))))

#define F1(b, c, d) (((b) & (c)) | ((~b) & (d)))
#define F2(b, c, d) ((b) ^ (c) ^ (d))
#define F3(b, c, d) (((b) & (c)) | ((b) & (d)) | ((c) & (d)))
#define F4(b, c, d) ((b) ^ (c) ^ (d))

static const u32 K[4] = { 0x5A827999, 0x6ED9EBA1, 0x8F1BBCDC, 0xCA62C1D6 };


Sha160::Sha160()
  : datalen ( 0 )
  , bitlen  ( 0 )
{
  my_memset( data, 0, MY_SHA160_DATA_SIZE );

  state[0] = 0x67452301;
  state[1] = 0xEFCDAB89;
  state[2] = 0x98BADCFE;
  state[3] = 0x10325476;
  state[4] = 0xC3D2E1F0;
}

void Sha160::transform()
{
  u32 a, b, c, d, e, f, k, temp;
  u32 m[80];

  for ( u32 i = 0, j = 0; i < 16; ++i, j += 4 )
  {
    m[i] = (data[j] << 24) | (data[j + 1] << 16) | (data[j + 2] << 8) | (data[j + 3]);
  }

  for ( u32 i = 16; i < 80; ++i )
  {
    m[i] = ROTLEFT( m[i - 3] ^ m[i - 8] ^ m[i - 14] ^ m[i - 16], 1 );
  }

  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];

  for ( u32 i = 0; i < 80; ++i )
  {
    if ( i < 20 )
    {
      f = F1( b, c, d );
      k = K[0];
    } else
    if ( i < 40 )
    {
      f = F2( b, c, d );
      k = K[1];
    } else
    if ( i < 60 )
    {
      f = F3( b, c, d );
      k = K[2];
    } else
    {
      f = F4( b, c, d );
      k = K[3];
    }

    temp = ROTLEFT( a, 5 ) + f + e + k + m[i];
    e = d;
    d = c;
    c = ROTLEFT( b, 30 );
    b = a;
    a = temp;
  }

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
}

void Sha160::update( u8 const input_data[], u32 len )
{
  for ( u32 i = 0; i < len; ++i )
  {
    data[datalen++] = input_data[i];

    if ( datalen == MY_SHA160_DATA_SIZE )
    {
      transform();
      bitlen += 512;
      datalen = 0;
    }
  }
}

void Sha160::calcFinal( u8 hash[] )
{
  u32 i = datalen;

  data[i++] = 0x80;
  while ( i < MY_SHA160_DATA_SIZE )
  {
    data[i++] = 0x00;
  }

  if ( datalen + 1 > MY_SHA160_DATA_SIZE - 8 )
  {
    transform();
    my_memset( data, 0, 56 );
  }

  bitlen += datalen * 8;

  data[63] = (u8)(bitlen			);
  data[62] = (u8)(bitlen >>  8);
  data[61] = (u8)(bitlen >> 16);
  data[60] = (u8)(bitlen >> 24);
  data[59] = (u8)(bitlen >> 32);
  data[58] = (u8)(bitlen >> 40);
  data[57] = (u8)(bitlen >> 48);
  data[56] = (u8)(bitlen >> 56);

  transform();

  for ( i = 0; i < 4; ++i )
  {
    u32 k = 24 - i * 8;
    hash[i     ] = (state[0] >> k) & 0xff;
    hash[i +  4] = (state[1] >> k) & 0xff;
    hash[i +  8] = (state[2] >> k) & 0xff;
    hash[i + 12] = (state[3] >> k) & 0xff;
    hash[i + 16] = (state[4] >> k) & 0xff;
  }
}

} // namespace zygo
