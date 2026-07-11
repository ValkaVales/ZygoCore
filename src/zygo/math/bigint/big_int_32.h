#pragma once

#include <zygo/core/types.h>

#include <vector>
#include <string>


namespace zygo {

// Arbitrary-precision signed integer with 32-BIT limbs (base 2^32).
//
// This is the twin of BigInt, retargeted for 32-bit machines (STM32 / ARM Cortex-M). 
// The whole design rests on one fact: the product of two 32-bit limbs fits in a 64-bit accumulator, 
// so every carry and every partial product is plain portable C++ -- no 64-bit intrinsics, no __int128, nothing platform-specific.
// On the desktop prefer BigInt (64-bit limbs do half as many iterations); on a 32-bit core BigInt32 wins because its limb matches the native word.
//
// The API mirrors BigInt exactly, so tests and call sites transfer as-is.
class BigInt32
{
private:
  std::vector<u32> limbs; // little-endian; empty vector means zero; no leading zero limbs
  bool neg;               // zero is never negative

public:
  BigInt32();
  BigInt32( int v );
  BigInt32( i64 v );
  BigInt32( u64 v );
  explicit BigInt32( char const * decimal_str ); // optional leading '-'

  bool isZero    () const { return limbs.empty(); }
  bool isNegative() const { return neg; }
  bool isEven    () const { return limbs.empty() || (limbs[0] & 1) == 0; }
  bool isOdd     () const { return !isEven(); }

  int  bitsNum () const;
  bool checkBit( int i ) const;

  std::string toString() const;
  std::string toHex   () const;

  int compare( BigInt32 const& other ) const;

  void reserveLimbs( int count ) { limbs.reserve( (size_t)count ); }

  // --- arithmetic ---
  BigInt32 operator-() const;

  BigInt32& operator+=( BigInt32 const& b );
  BigInt32& operator-=( BigInt32 const& b );
  BigInt32& operator*=( BigInt32 const& b );
  BigInt32& operator/=( BigInt32 const& b );
  BigInt32& operator%=( BigInt32 const& b );

  friend BigInt32 operator+( BigInt32 a, BigInt32 const& b ) { a += b; return a; }
  friend BigInt32 operator-( BigInt32 a, BigInt32 const& b ) { a -= b; return a; }
  friend BigInt32 operator*( BigInt32 a, BigInt32 const& b ) { a *= b; return a; }
  friend BigInt32 operator/( BigInt32 a, BigInt32 const& b ) { a /= b; return a; }
  friend BigInt32 operator%( BigInt32 a, BigInt32 const& b ) { a %= b; return a; }

  // in-place mutating twins (no temporaries), matching BigInt
  void add( BigInt32 const& b );
  void sub( BigInt32 const& b );

  static void divMod( BigInt32 const& a, BigInt32 const& b, BigInt32& q, BigInt32& r );

  BigInt32& operator<<=( int bits );
  BigInt32& operator>>=( int bits );

  friend BigInt32 operator<<( BigInt32 a, int bits ) { a <<= bits; return a; }
  friend BigInt32 operator>>( BigInt32 a, int bits ) { a >>= bits; return a; }

  // --- comparisons ---
  friend bool operator==( BigInt32 const& a, BigInt32 const& b ) { return a.compare( b ) == 0; }
  friend bool operator!=( BigInt32 const& a, BigInt32 const& b ) { return a.compare( b ) != 0; }
  friend bool operator< ( BigInt32 const& a, BigInt32 const& b ) { return a.compare( b ) <  0; }
  friend bool operator<=( BigInt32 const& a, BigInt32 const& b ) { return a.compare( b ) <= 0; }
  friend bool operator> ( BigInt32 const& a, BigInt32 const& b ) { return a.compare( b ) >  0; }
  friend bool operator>=( BigInt32 const& a, BigInt32 const& b ) { return a.compare( b ) >= 0; }

  // --- number theory ---
  BigInt32 modPow( BigInt32 const& e, BigInt32 const& m ) const;
  static BigInt32 gcd( BigInt32 a, BigInt32 b );

  void pow          ( int power );
  void powModP      ( BigInt32 const& power, BigInt32 const& p );
  void calcRemainder( BigInt32 const& b );

private:
  void trim();
  void setU64( u64 magnitude, bool negative );

  static int  cmpAbs( std::vector<u32> const& a, std::vector<u32> const& b );
  static void addAbs( std::vector<u32> & a, std::vector<u32> const& b );
  static void subAbs( std::vector<u32> & a, std::vector<u32> const& b ); // requires a >= b
  static void mulAbs( std::vector<u32> const& a, std::vector<u32> const& b,
    std::vector<u32> & res );
  static void divModAbs( std::vector<u32> const& u, std::vector<u32> const& d,
    std::vector<u32> & q, std::vector<u32> & r );

  static void mulAddSmall( std::vector<u32> & a, u32 mul, u32 add );
  static u32  divSmall   ( std::vector<u32> & a, u32 d );

  void addSigned( std::vector<u32> const& b_limbs, bool b_neg );
};

} // namespace zygo
