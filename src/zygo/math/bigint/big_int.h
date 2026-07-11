#pragma once

#include <zygo/core/types.h>


namespace zygo {

typedef i64 BIType;

const int BIGINT_LEN_MAX  = 150;// 128;
const int BIGINT_BPU      = 8 * sizeof(BIType) - 1;
const int BIGINT_BPU2     = 4 * sizeof(BIType);
const BIType BIGINT_LO_MASK  = 0x00000000ffffffff;
const BIType BIGINT_MASK     = 0x7fffffffffffffff;


class BigInt
{
private:
  enum class Sign
  {
    PLUS = 0,
    MINUS
  };

private:
  int len;
  Sign sign;
  BIType val[BIGINT_LEN_MAX];

public:
  BigInt();
  BigInt( BIType a );
  BigInt( BigInt const& a );
  BigInt( char const * s, int radix );

  inline bool isZero() const { return len == 0 || (len == 1 && val[0] == 0); }
  inline bool isEven() const { return len == 0 || (val[0] & 1) == 0; }
  inline bool isOdd() const { return !isEven(); }
  inline bool isNegative() const { return sign == Sign::MINUS; }
  
  inline int getLen() const { return len; }
  inline BIType getVal( int i ) const { return val[i]; }

  void set( BIType a );
  void set( char const * s, int radix );
  void copyFrom( BigInt const& a );

  void print( int radix, char const * name = nullptr );
  void printRaw( char const * name = nullptr );

  inline void inc() { add( 1 ); }
  inline void dec() { sub( 1 ); }

  int bitsNum() const;
  inline bool checkBit( int i ) const { return (val[i / BIGINT_BPU] & ((BIType)1 << (i % BIGINT_BPU))) != 0; }

  void add( BigInt const& a );
  void sub( BigInt const& a );
  void add( BIType a );
  void sub( BIType a );

  void shiftLeft();
  //void shiftLeft( int x );
  void bigShiftLeft();
  void shiftRight();
  void shiftRight( int x );

  void pow( int power );
  void powModP( BigInt power, BigInt const& p );

  int compare( BigInt const& a ) const;
  inline bool eq( BigInt const& a ) const { return compare( a ) == 0; }

  void mul( BigInt const& b );
  void mul( BigInt const& b, int keep );
  void div( BigInt const& b, BigInt& rem );
  void div2( BigInt const& b, BigInt& rem );
  void calcRemainder( BigInt const& b );
  void divInt( BIType b, BIType& rem );

  void modExpSimple ( BigInt const& e, BigInt const& M );
  void modInverse   ( BigInt const& m );

  static BIType greatestCommonDivisor( BigInt const& a, BigInt const& b );

  void signMul( BigInt const& b );
  void signDiv( BigInt const& b, BigInt& rem );
  void signSub( BigInt const& a );
};

} // namespace zygo
