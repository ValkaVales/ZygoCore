#include "aes128.h"


namespace zygo {

const u16 AES_MOD_CONST = 0x011b; // x^8 + x^4 + x^3 + x + 1


u8 AES_ARRAY_S[256] =
{
  0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
  0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0, 0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0,
  0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
  0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
  0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0, 0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84,
  0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
  0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
  0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5, 0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
  0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
  0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB,
  0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C, 0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
  0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
  0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A,
  0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E, 0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E,
  0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
  0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16
};

u8 AES_INVERTED_S[256] =
{
  0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38, 0xBF, 0x40, 0xA3, 0x9E, 0x81, 0xF3, 0xD7, 0xFB,
  0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87, 0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB,
  0x54, 0x7B, 0x94, 0x32, 0xA6, 0xC2, 0x23, 0x3D, 0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
  0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2, 0x76, 0x5B, 0xA2, 0x49, 0x6D, 0x8B, 0xD1, 0x25,
  0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92,
  0x6C, 0x70, 0x48, 0x50, 0xFD, 0xED, 0xB9, 0xDA, 0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
  0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A, 0xF7, 0xE4, 0x58, 0x05, 0xB8, 0xB3, 0x45, 0x06,
  0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02, 0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B,
  0x3A, 0x91, 0x11, 0x41, 0x4F, 0x67, 0xDC, 0xEA, 0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
  0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85, 0xE2, 0xF9, 0x37, 0xE8, 0x1C, 0x75, 0xDF, 0x6E,
  0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89, 0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B,
  0xFC, 0x56, 0x3E, 0x4B, 0xC6, 0xD2, 0x79, 0x20, 0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
  0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31, 0xB1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xEC, 0x5F,
  0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D, 0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF,
  0xA0, 0xE0, 0x3B, 0x4D, 0xAE, 0x2A, 0xF5, 0xB0, 0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
  0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26, 0xE1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0C, 0x7D
};


Aes128::Aes128( u8 const* key )
  : key ( key )
{
  expandKey();
}

u8 Aes128::gmul( u8 a, u8 b )
{
  u8 res = 0;
  while ( a && b )
  {
    if ( b & 1 )
      res ^= a;

    if ( a & 0x80 )
      a = (a << 1) ^ AES_MOD_CONST;
    else
      a <<= 1;

    b >>= 1;
  }
  return res;
}

u8 Aes128::myXor( u8 i )
{
  if ( i == 0 )
    return 0x8d;

  u8 res = 1;
  for ( u8 x = 1; x < i; x++ )
    res = gmul( res, 2 );

  return res;
}

void Aes128::rotateArrayWithMyXor( u8* arr, u8 i )
{
  u8 tmp = arr[0];
  arr[0] = arr[1];
  arr[1] = arr[2];
  arr[2] = arr[3];
  arr[3] = tmp;

  arr[0] = AES_ARRAY_S[arr[0]];
  arr[1] = AES_ARRAY_S[arr[1]];
  arr[2] = AES_ARRAY_S[arr[2]];
  arr[3] = AES_ARRAY_S[arr[3]];

  arr[0] ^= myXor( i );
  // other 3 bytes are XORed with 0
}

void Aes128::expandKey()
{
  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    expanded_key[i] = key[i];

  //
  int bytes_count = AES_128_BLOCK_SIZE;
  int rcon_i = 1;

  while ( bytes_count < AES_128_EXPANDED_KEY_SIZE )
  {
    u8 tmp[4];
    for ( int i = 0; i < 4; i++ )
      tmp[i] = expanded_key[i + bytes_count - 4];

    if ( bytes_count % AES_128_BLOCK_SIZE == 0 )
      rotateArrayWithMyXor( tmp, rcon_i++ );

    for ( u8 i = 0; i < 4; i++ )
    {
      expanded_key[bytes_count] = expanded_key[bytes_count - AES_128_BLOCK_SIZE] ^ tmp[i];
      bytes_count++;
    }
  }
}

void Aes128::addSubRoundKey( u8 const* round_key )
{
  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] ^= round_key[i];
}

void Aes128::encryptSubBytes()
{
  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = AES_ARRAY_S[state[i]];
}

void Aes128::decryptSubBytes()
{
  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = AES_INVERTED_S[state[i]];
}

void Aes128::shiftRowsLeft()
{
  u8 tmp[AES_128_BLOCK_SIZE];

  // 0  4  8 12 ->  0  4  8 12
  // 1  5  9 13 ->  5  9 13  1
  // 2  6 10 14 -> 10 14  2  6
  // 3  7 11 15 -> 15  3  7 11

  tmp[ 0] = state[ 0];
  tmp[ 1] = state[ 5];
  tmp[ 2] = state[10];
  tmp[ 3] = state[15];

  tmp[ 4] = state[ 4];
  tmp[ 5] = state[ 9];
  tmp[ 6] = state[14];
  tmp[ 7] = state[ 3];

  tmp[ 8] = state[ 8];
  tmp[ 9] = state[13];
  tmp[10] = state[ 2];
  tmp[11] = state[ 7];

  tmp[12] = state[12];
  tmp[13] = state[ 1];
  tmp[14] = state[ 6];
  tmp[15] = state[11];

  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = tmp[i];
}

void Aes128::shiftRowsRight()
{
  u8 tmp[AES_128_BLOCK_SIZE];

  // 0  4  8 12 ->  0  4  8 12
  // 1  5  9 13 -> 13  1  5  9
  // 2  6 10 14 -> 10 14  2  6
  // 3  7 11 15 ->  7 11 15  3

  tmp[ 0] = state[ 0];
  tmp[ 1] = state[13];
  tmp[ 2] = state[10];
  tmp[ 3] = state[ 7];

  tmp[ 4] = state[ 4];
  tmp[ 5] = state[ 1];
  tmp[ 6] = state[14];
  tmp[ 7] = state[11];

  tmp[ 8] = state[ 8];
  tmp[ 9] = state[ 5];
  tmp[10] = state[ 2];
  tmp[11] = state[15];

  tmp[12] = state[12];
  tmp[13] = state[ 9];
  tmp[14] = state[ 6];
  tmp[15] = state[ 3];

  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = tmp[i];
}

void Aes128::mixColumns()
{
  u8 temp_state[AES_128_BLOCK_SIZE];

  temp_state[ 0] = (u8)(gmul( state[ 0], 2 )  ^ gmul( state[ 1], 3 )  ^       state[ 2]       ^       state[3]);
  temp_state[ 1] = (u8)(      state[ 0]       ^ gmul( state[ 1], 2 )  ^ gmul( state[ 2], 3 )  ^       state[3]);
  temp_state[ 2] = (u8)(      state[ 0]       ^       state[ 1]       ^ gmul( state[ 2], 2 )  ^ gmul( state[3], 3 ));
  temp_state[ 3] = (u8)(gmul( state[ 0], 3 )  ^       state[ 1]       ^       state[ 2]       ^ gmul( state[3], 2 ));

  temp_state[ 4] = (u8)(gmul( state[ 4], 2 )  ^ gmul( state[ 5], 3 )  ^       state[ 6]       ^       state[7]);
  temp_state[ 5] = (u8)(      state[ 4]       ^ gmul( state[ 5], 2 )  ^ gmul( state[ 6], 3 )  ^       state[7]);
  temp_state[ 6] = (u8)(      state[ 4]       ^       state[ 5]       ^ gmul( state[ 6], 2 )  ^ gmul( state[7], 3 ));
  temp_state[ 7] = (u8)(gmul( state[ 4], 3 )  ^       state[ 5]       ^       state[ 6]       ^ gmul( state[7], 2 ));

  temp_state[ 8] = (u8)(gmul( state[ 8], 2 )  ^ gmul( state[ 9], 3 )  ^       state[10]       ^       state[11]);
  temp_state[ 9] = (u8)(      state[ 8]       ^ gmul( state[ 9], 2 )  ^ gmul( state[10], 3 )  ^       state[11]);
  temp_state[10] = (u8)(      state[ 8]       ^       state[ 9]       ^ gmul( state[10], 2 )  ^ gmul( state[11], 3 ));
  temp_state[11] = (u8)(gmul( state[ 8], 3 )  ^       state[ 9]       ^       state[10]       ^ gmul( state[11], 2 ));

  temp_state[12] = (u8)(gmul( state[12], 2 )  ^ gmul( state[13], 3 )  ^       state[14]       ^       state[15]);
  temp_state[13] = (u8)(      state[12]       ^ gmul( state[13], 2 )  ^ gmul( state[14], 3 )  ^       state[15]);
  temp_state[14] = (u8)(      state[12]       ^       state[13]       ^ gmul( state[14], 2 )  ^ gmul( state[15], 3 ));
  temp_state[15] = (u8)(gmul( state[12], 3 )  ^       state[13]       ^       state[14]       ^ gmul( state[15], 2 ));

  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = temp_state[i];
}

void Aes128::inverseMixColumns()
{
  u8 temp_state[AES_128_BLOCK_SIZE];

  temp_state[ 0] = (u8)(gmul( state[ 0], 14 ) ^ gmul( state[ 1], 11 ) ^ gmul( state[ 2], 13 ) ^ gmul( state[ 3],  9 ));
  temp_state[ 1] = (u8)(gmul( state[ 0],  9 ) ^ gmul( state[ 1], 14 ) ^ gmul( state[ 2], 11 ) ^ gmul( state[ 3], 13 ));
  temp_state[ 2] = (u8)(gmul( state[ 0], 13 ) ^ gmul( state[ 1],  9 ) ^ gmul( state[ 2], 14 ) ^ gmul( state[ 3], 11 ));
  temp_state[ 3] = (u8)(gmul( state[ 0], 11 ) ^ gmul( state[ 1], 13 ) ^ gmul( state[ 2],  9 ) ^ gmul( state[ 3], 14 ));

  temp_state[ 4] = (u8)(gmul( state[ 4], 14 ) ^ gmul( state[ 5], 11 ) ^ gmul( state[ 6], 13 ) ^ gmul( state[ 7],  9 ));
  temp_state[ 5] = (u8)(gmul( state[ 4],  9 ) ^ gmul( state[ 5], 14 ) ^ gmul( state[ 6], 11 ) ^ gmul( state[ 7], 13 ));
  temp_state[ 6] = (u8)(gmul( state[ 4], 13 ) ^ gmul( state[ 5],  9 ) ^ gmul( state[ 6], 14 ) ^ gmul( state[ 7], 11 ));
  temp_state[ 7] = (u8)(gmul( state[ 4], 11 ) ^ gmul( state[ 5], 13 ) ^ gmul( state[ 6],  9 ) ^ gmul( state[ 7], 14 ));

  temp_state[ 8] = (u8)(gmul( state[ 8], 14 ) ^ gmul( state[ 9], 11 ) ^ gmul( state[10], 13 ) ^ gmul( state[11],  9 ));
  temp_state[ 9] = (u8)(gmul( state[ 8],  9 ) ^ gmul( state[ 9], 14 ) ^ gmul( state[10], 11 ) ^ gmul( state[11], 13 ));
  temp_state[10] = (u8)(gmul( state[ 8], 13 ) ^ gmul( state[ 9],  9 ) ^ gmul( state[10], 14 ) ^ gmul( state[11], 11 ));
  temp_state[11] = (u8)(gmul( state[ 8], 11 ) ^ gmul( state[ 9], 13 ) ^ gmul( state[10],  9 ) ^ gmul( state[11], 14 ));

  temp_state[12] = (u8)(gmul( state[12], 14 ) ^ gmul( state[13], 11 ) ^ gmul( state[14], 13 ) ^ gmul( state[15],  9 ));
  temp_state[13] = (u8)(gmul( state[12],  9 ) ^ gmul( state[13], 14 ) ^ gmul( state[14], 11 ) ^ gmul( state[15], 13 ));
  temp_state[14] = (u8)(gmul( state[12], 13 ) ^ gmul( state[13],  9 ) ^ gmul( state[14], 14 ) ^ gmul( state[15], 11 ));
  temp_state[15] = (u8)(gmul( state[12], 11 ) ^ gmul( state[13], 13 ) ^ gmul( state[14],  9 ) ^ gmul( state[15], 14 ));

  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = temp_state[i];
}

void Aes128::encryptBlock( u8 const* bytes, u8* encrypted_res, int bytes_left )
{
  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = i < bytes_left ? bytes[i] : 0;

  addSubRoundKey( expanded_key );

  for ( int i = 1; i < AES_128_ROUNDS_COUNT; i++ )
  {
    encryptSubBytes();
    shiftRowsLeft();
    mixColumns();
    addSubRoundKey( expanded_key + (AES_128_BLOCK_SIZE * i) );
  }

  encryptSubBytes();
  shiftRowsLeft();
  addSubRoundKey( expanded_key + AES_128_BLOCK_SIZE * AES_128_ROUNDS_COUNT );

  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    if ( i < bytes_left )
      encrypted_res[i] = state[i];
}

void Aes128::decryptBlock( u8 const* bytes, u8* decrypted_res, int bytes_left )
{
  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    state[i] = i < bytes_left ? bytes[i] : 0;

  addSubRoundKey( expanded_key + AES_128_BLOCK_SIZE * AES_128_ROUNDS_COUNT );
  shiftRowsRight();
  decryptSubBytes();

  for ( int i = AES_128_ROUNDS_COUNT - 1; i >= 1; i-- )
  {
    addSubRoundKey( expanded_key + AES_128_BLOCK_SIZE * i );
    inverseMixColumns();
    shiftRowsRight();
    decryptSubBytes();
  }

  addSubRoundKey( expanded_key );

  for ( int i = 0; i < AES_128_BLOCK_SIZE; i++ )
    if ( i < bytes_left )
      decrypted_res[i] = state[i];
}

void Aes128::encrypt( u8 const* bytes, u8* encrypted_res, int bytes_count )
{
  int blocks_count = (bytes_count + AES_128_BLOCK_SIZE - 1) / AES_128_BLOCK_SIZE;

  u8 const* p1 = bytes;
  u8* p2 = encrypted_res;
  int bytes_left = bytes_count;

  for ( int block_idx = 0; block_idx < blocks_count; ++block_idx )
  {
    encryptBlock( p1, p2, bytes_left );

    p1 += AES_128_BLOCK_SIZE;
    p2 += AES_128_BLOCK_SIZE;
    bytes_left -= AES_128_BLOCK_SIZE;
  }
}

void Aes128::decrypt( u8 const* bytes, u8* decrypted_res, int bytes_count )
{
  int blocks_count = (bytes_count + AES_128_BLOCK_SIZE - 1) / AES_128_BLOCK_SIZE;

  u8 const* p1 = bytes;
  u8* p2 = decrypted_res;
  int bytes_left = bytes_count;

  for ( int block_idx = 0; block_idx < blocks_count; ++block_idx )
  {
    decryptBlock( p1, p2, bytes_left );

    p1 += AES_128_BLOCK_SIZE;
    p2 += AES_128_BLOCK_SIZE;
    bytes_left -= AES_128_BLOCK_SIZE;
  }
}

} // namespace zygo
