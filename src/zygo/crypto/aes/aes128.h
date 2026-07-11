#pragma once

#include <zygo/core/types.h>


namespace zygo {

const int AES_128_BLOCK_SIZE        = 16; // 16 bytes == 128 bits
//const int AES_128_BLOCK_SIZE2       = AES_128_BLOCK_SIZE * 2;
const int AES_128_KEY_SIZE          = AES_128_BLOCK_SIZE;
const int AES_128_EXPANDED_KEY_SIZE = 176;
const int AES_128_ROUNDS_COUNT      = 10;

class Aes128
{
private:
  u8 const* key;
  u8 expanded_key [AES_128_EXPANDED_KEY_SIZE];
  u8 state        [AES_128_BLOCK_SIZE];

public:
  Aes128( u8 const* key );

  // Ciphertext is always a whole number of blocks: for bytes_count == 20
  // it occupies 32 bytes. Use cipherSize() to allocate encrypted_res.
  static int cipherSize( int bytes_count )
  {
    return (bytes_count + AES_128_BLOCK_SIZE - 1) / AES_128_BLOCK_SIZE * AES_128_BLOCK_SIZE;
  }

  // encrypted_res must have room for cipherSize( bytes_count ) bytes
  void encrypt( u8 const* bytes, u8* encrypted_res, int bytes_count );

  // bytes holds cipherSize( bytes_count ) bytes of ciphertext;
  // exactly bytes_count bytes of plaintext are written to decrypted_res
  void decrypt( u8 const* bytes, u8* decrypted_res, int bytes_count );

private:
  void encryptBlock( u8 const* bytes, u8* encrypted_res, int bytes_left );
  void decryptBlock( u8 const* bytes, u8* decrypted_res, int bytes_left );

  static u8 gmul( u8 a, u8 b );
  static u8 myXor( u8 i );
  static void rotateArrayWithMyXor( u8* arr, u8 i );

  void expandKey();

  void addSubRoundKey( u8 const* round_key );
  void encryptSubBytes();
  void decryptSubBytes();
  void shiftRowsLeft();
  void shiftRowsRight();

  void mixColumns();
  void inverseMixColumns();

};

} // namespace zygo
