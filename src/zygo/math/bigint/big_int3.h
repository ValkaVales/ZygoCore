#pragma once

#include <zygo/core/types.h>

#include <vector>
#include <string>


namespace zygo {

//#define ZYGO_FORCE_PORTABLE_128

// Arbitrary-precision signed integer (sign-magnitude).
//
// - limbs are FULL 64-bit words (base 2^64), little-endian order;
//   carries/products use hardware intrinsics (_addcarry_u64 / _umul128
//   on MSVC, unsigned __int128 on GCC and Clang)
// - division is Knuth's Algorithm D with 64-bit trial digits,
//   not bit-by-bit shift-and-subtract
// - size is dynamic, no fixed limb count
//
// Division semantics match built-in C++ integers (truncation toward zero,
// remainder carries the sign of the dividend).
class BigInt3
{
private:
  std::vector<u64> limbs; // little-endian; empty vector means zero; no leading zero limbs
  bool neg;               // zero is never negative

public:
  BigInt3();
  BigInt3( int v );
  BigInt3( i64 v );
  BigInt3( u64 v );
  explicit BigInt3( char const * decimal_str ); // optional leading '-'

  bool isZero    () const { return limbs.empty(); }
  bool isNegative() const { return neg; }
  bool isEven    () const { return limbs.empty() || (limbs[0] & 1) == 0; }
  bool isOdd     () const { return !isEven(); }

  int  bitsNum() const;          // bits in the magnitude; 0 for zero
  bool checkBit( int i ) const;  // i-th bit of the magnitude

  void reserveLimbs( int size );

  std::string toString() const;  // decimal, with leading '-' if negative
  std::string toHex   () const;  // magnitude in hex, with leading '-' if negative

  int compare( BigInt3 const& other ) const; // -1 / 0 / +1

  // --- arithmetic ---
  BigInt3 operator-() const;

  BigInt3& operator+=( BigInt3 const& b );
  BigInt3& operator-=( BigInt3 const& b );
  BigInt3& operator*=( BigInt3 const& b );
  BigInt3& operator/=( BigInt3 const& b );
  BigInt3& operator%=( BigInt3 const& b );

  friend BigInt3 operator+( BigInt3 a, BigInt3 const& b ) { a += b; return a; }
  friend BigInt3 operator-( BigInt3 a, BigInt3 const& b ) { a -= b; return a; }
  friend BigInt3 operator*( BigInt3 a, BigInt3 const& b ) { a *= b; return a; }
  friend BigInt3 operator/( BigInt3 a, BigInt3 const& b ) { a /= b; return a; }
  friend BigInt3 operator%( BigInt3 a, BigInt3 const& b ) { a %= b; return a; }

  void add( BigInt3 const& a );
  void sub( BigInt3 const& a );

  //
  void pow          ( int power );                             // this = this ^ power; 0^0 == 1
  void powModP      ( BigInt3 const& power, BigInt3 const& p );  // this = this ^ power mod p
  void calcRemainder( BigInt3 const& b );                       // this %= b

  // quotient and remainder in one division
  static void divMod( BigInt3 const& a, BigInt3 const& b, BigInt3& q, BigInt3& r );

  // shifts operate on the magnitude, the sign is preserved
  BigInt3& operator<<=( int bits );
  BigInt3& operator>>=( int bits );

  friend BigInt3 operator<<( BigInt3 a, int bits ) { a <<= bits; return a; }
  friend BigInt3 operator>>( BigInt3 a, int bits ) { a >>= bits; return a; }

  // --- comparisons ---
  friend bool operator==( BigInt3 const& a, BigInt3 const& b ) { return a.compare( b ) == 0; }
  friend bool operator!=( BigInt3 const& a, BigInt3 const& b ) { return a.compare( b ) != 0; }
  friend bool operator< ( BigInt3 const& a, BigInt3 const& b ) { return a.compare( b ) <  0; }
  friend bool operator<=( BigInt3 const& a, BigInt3 const& b ) { return a.compare( b ) <= 0; }
  friend bool operator> ( BigInt3 const& a, BigInt3 const& b ) { return a.compare( b ) >  0; }
  friend bool operator>=( BigInt3 const& a, BigInt3 const& b ) { return a.compare( b ) >= 0; }

  // --- number theory ---
  BigInt3 modPow( BigInt3 const& e, BigInt3 const& m ) const; // was: modExpSimple; e >= 0, m > 0
  static BigInt3 gcd( BigInt3 a, BigInt3 b );                 // was: greatestCommonDivisor

private:
  void trim();
  void setU64( u64 magnitude, bool negative );

  static int  cmpAbs( std::vector<u64> const& a, std::vector<u64> const& b );
  static void addAbs( std::vector<u64> & a, std::vector<u64> const& b );        // a += b
  static void subAbs( std::vector<u64> & a, std::vector<u64> const& b );        // a -= b, requires a >= b
  static void mulAbs( std::vector<u64> const& a, std::vector<u64> const& b, std::vector<u64> & res );
  static void divModAbs( std::vector<u64> const& u, std::vector<u64> const& d, std::vector<u64> & q, std::vector<u64> & r );          // Knuth's Algorithm D

  static void mulAddSmall( std::vector<u64> & a, u64 mul, u64 add );            // a = a * mul + add
  static u64  divSmall   ( std::vector<u64> & a, u64 d );                       // a /= d, returns remainder

  void addSigned( std::vector<u64> const& b_limbs, bool b_neg );
};

} // namespace zygo
