#include "big_int3.h"
#include <zygo/core/assert.h>

#if defined(_MSC_VER) && !defined(ZYGO_FORCE_PORTABLE_128)
#include <intrin.h> // must be included OUTSIDE any namespace
#endif


namespace zygo {

// ==================================================================
// 64-bit hardware primitives.
// MSVC x64 exposes them as intrinsics; GCC and Clang go through
// unsigned __int128 and compile to the same single instructions.
// ==================================================================
#if defined(_MSC_VER) && !defined(ZYGO_FORCE_PORTABLE_128)

static inline u8 addCarry64( u8 carry, u64 a, u64 b, u64 * res )
{
  return _addcarry_u64( carry, a, b, (unsigned long long *)res );
}

static inline u8 subBorrow64( u8 borrow, u64 a, u64 b, u64 * res )
{
  return _subborrow_u64( borrow, a, b, (unsigned long long *)res );
}

static inline u64 mul64x64( u64 a, u64 b, u64 * hi )
{
  return _umul128( a, b, hi );
}

static inline int clz64( u64 v ) // v != 0
{
  unsigned long idx;
  _BitScanReverse64( &idx, v );
  return 63 - (int)idx;
}

#if _MSC_VER < 1922
#define ZYGO_PORTABLE_DIV128 1 // _udiv128 appeared in VS2019 16.2;
// older toolsets get the portable fallback
#endif
#else

using u128 = unsigned __int128;

static inline u8 addCarry64( u8 carry, u64 a, u64 b, u64 * res )
{
  u128 s = (u128)a + b + carry;
  *res = (u64)s;
  return (u8)(s >> 64);
}

static inline u8 subBorrow64( u8 borrow, u64 a, u64 b, u64 * res )
{
  u128 d = (u128)a - b - borrow;
  *res = (u64)d;
  return (u8)((d >> 64) != 0);
}

static inline u64 mul64x64( u64 a, u64 b, u64 * hi )
{
  u128 p = (u128)a * b;
  *hi = (u64)(p >> 64);
  return (u64)p;
}

static inline int clz64( u64 v ) // v != 0
{
  return __builtin_clzll( v );
}

#if defined(ZYGO_FORCE_PORTABLE_128)
#define ZYGO_PORTABLE_DIV128 1
#endif

#endif


// (hi:lo) / d -> quotient, remainder. Requires hi < d (quotient fits in 64 bits).
#if defined(ZYGO_PORTABLE_DIV128)

// Classic two-halves long division ("Hacker's Delight", divlu2).
static u64 div128by64( u64 hi, u64 lo, u64 d, u64 * rem )
{
  const u64 B = (u64)1 << 32;

  int s = clz64( d ); // d != 0 is guaranteed by the caller
  d <<= s;

  u64 dh = d >> 32;
  u64 dl = d & 0xffffffffu;

  u64 un64 = ( s == 0 ) ? hi : (hi << s) | (lo >> (64 - s));
  u64 un10 = lo << s;

  u64 un1 = un10 >> 32;
  u64 un0 = un10 & 0xffffffffu;

  u64 q1 = un64 / dh;
  u64 r1 = un64 % dh;

  while ( q1 >= B || q1 * dl > B * r1 + un1 )
  {
    q1--;
    r1 += dh;
    if ( r1 >= B )
      break;
  }

  u64 un21 = un64 * B + un1 - q1 * d;

  u64 q0 = un21 / dh;
  u64 r0 = un21 % dh;

  while ( q0 >= B || q0 * dl > B * r0 + un0 )
  {
    q0--;
    r0 += dh;
    if ( r0 >= B )
      break;
  }

  *rem = ( un21 * B + un0 - q0 * d ) >> s;
  return q1 * B + q0;
}

#elif defined(_MSC_VER)

static inline u64 div128by64( u64 hi, u64 lo, u64 d, u64 * rem )
{
  return _udiv128( hi, lo, d, rem );
}

#else

static inline u64 div128by64( u64 hi, u64 lo, u64 d, u64 * rem )
{
  u128 n = ((u128)hi << 64) | lo;
  *rem = (u64)( n % d );
  return (u64)( n / d );
}

#endif


// one inner step of schoolbook multiplication: r += a * b + carry
#if defined(_MSC_VER) && !defined(ZYGO_FORCE_PORTABLE_128)

static inline void mulStep( u64 a, u64 b, u64 & r, u64 & carry )
{
  u64 hi;
  u64 lo = mul64x64( a, b, &hi );

  hi += addCarry64( 0, lo, carry, &lo ); // cannot overflow: hi <= 2^64 - 2
  hi += addCarry64( 0, r, lo, &r );
  carry = hi;
}

#else

static inline void mulStep( u64 a, u64 b, u64 & r, u64 & carry )
{
  u128 t = (u128)a * b + r + carry; // fits: (2^64-1)^2 + 2*(2^64-1) = 2^128 - 1
  r     = (u64)t;
  carry = (u64)( t >> 64 );
}

#endif


void BigInt3::reserveLimbs( int size )
{
  limbs.reserve( size );
}


// ==================================================================
// magnitude helpers (unsigned vectors of limbs)
// ==================================================================
int BigInt3::cmpAbs( std::vector<u64> const& a, std::vector<u64> const& b )
{
  if ( a.size() != b.size() )
    return a.size() < b.size() ? -1 : 1;

  for ( size_t i = a.size(); i-- > 0; )
    if ( a[i] != b[i] )
      return a[i] < b[i] ? -1 : 1;

  return 0;
}

void BigInt3::addAbs( std::vector<u64> & a, std::vector<u64> const& b )
{
  if ( a.size() < b.size() )
  {
    //a.reserve( a.size() * 10 );
    a.resize( b.size(), 0 );
  }

  u8 carry = 0;
  size_t i = 0;

  for ( ; i < b.size(); i++ )
    carry = addCarry64( carry, a[i], b[i], &a[i] );

  for ( ; carry && i < a.size(); i++ )
    carry = addCarry64( carry, a[i], 0, &a[i] );

  if ( carry )
    a.push_back( 1 );
}

// requires a >= b
void BigInt3::subAbs( std::vector<u64> & a, std::vector<u64> const& b )
{
  u8 borrow = 0;
  size_t i = 0;

  for ( ; i < b.size(); i++ )
    borrow = subBorrow64( borrow, a[i], b[i], &a[i] );

  for ( ; borrow && i < a.size(); i++ )
    borrow = subBorrow64( borrow, a[i], 0, &a[i] );

  while ( !a.empty() && a.back() == 0 )
    a.pop_back();
}

// schoolbook multiplication, one 64x64->128 hardware multiply per limb pair
void BigInt3::mulAbs( std::vector<u64> const& a, std::vector<u64> const& b,
  std::vector<u64> & res )
{
  res.assign( a.size() + b.size(), 0 );

  u64 *       r  = res.data();
  u64 const * bp = b.data();
  size_t      bn = b.size();

  for ( size_t i = 0; i < a.size(); i++, r++ )
  {
    u64 ai = a[i];
    if ( ai == 0 )
      continue;

    u64 carry = 0;

    for ( size_t j = 0; j < bn; j++ )
      mulStep( ai, bp[j], r[j], carry );

    r[bn] = carry; // this limb is untouched so far: plain store
  }

  while ( !res.empty() && res.back() == 0 )
    res.pop_back();
}

// a = a * mul + add (for decimal parsing)
void BigInt3::mulAddSmall( std::vector<u64> & a, u64 mul, u64 add )
{
  u64 carry = add;

  for ( size_t i = 0; i < a.size(); i++ )
  {
    u64 hi;
    u64 lo = mul64x64( a[i], mul, &hi );

    hi += addCarry64( 0, lo, carry, &a[i] );
    carry = hi;
  }

  if ( carry )
    a.push_back( carry );
}

// a /= d, returns the remainder (for decimal printing)
u64 BigInt3::divSmall( std::vector<u64> & a, u64 d )
{
  u64 rem = 0;

  for ( size_t i = a.size(); i-- > 0; )
    a[i] = div128by64( rem, a[i], d, &rem );

  while ( !a.empty() && a.back() == 0 )
    a.pop_back();

  return rem;
}

// Knuth, TAOCP vol. 2, Algorithm D. 64-bit trial digits.
void BigInt3::divModAbs( std::vector<u64> const& u, std::vector<u64> const& d,
  std::vector<u64> & q, std::vector<u64> & r )
{
  ZgAssertRelease( !d.empty() ); // division by zero

  if ( cmpAbs( u, d ) < 0 )
  {
    q.clear();
    r = u;
    return;
  }

  if ( d.size() == 1 ) // fast path: single-limb divisor
  {
    q = u;
    u64 rem = divSmall( q, d[0] );

    r.clear();
    if ( rem )
      r.push_back( rem );
    return;
  }

  size_t n = d.size();
  size_t m = u.size() - n;

  // D1: normalize so that the top bit of the divisor is set
  int s = clz64( d.back() );

  std::vector<u64> dn( n );
  for ( size_t i = n; i-- > 1; )
    dn[i] = ( s == 0 ) ? d[i] : (d[i] << s) | (d[i - 1] >> (64 - s));
  dn[0] = d[0] << s;

  std::vector<u64> un( u.size() + 1 );
  un[u.size()] = ( s == 0 ) ? 0 : u.back() >> (64 - s);
  for ( size_t i = u.size(); i-- > 1; )
    un[i] = ( s == 0 ) ? u[i] : (u[i] << s) | (u[i - 1] >> (64 - s));
  un[0] = u[0] << s;

  q.assign( m + 1, 0 );

  for ( size_t j = m + 1; j-- > 0; )
  {
    // D3: estimate the trial digit from the top two limbs
    u64 qhat, rhat;

    if ( un[j + n] == dn[n - 1] )
    {
      qhat = ~(u64)0; // B - 1
      rhat = un[j + n - 1] + dn[n - 1];
      if ( rhat < dn[n - 1] ) // rhat overflowed past B: the test below always passes
        goto mulsub;
    }
    else
      qhat = div128by64( un[j + n], un[j + n - 1], dn[n - 1], &rhat );

    // refine qhat using the third limb (at most two corrections)
    while ( true )
    {
      u64 ph;
      u64 pl = mul64x64( qhat, dn[n - 2], &ph );

      if ( ph > rhat || ( ph == rhat && pl > un[j + n - 2] ) )
      {
        qhat--;
        u64 old = rhat;
        rhat += dn[n - 1];
        if ( rhat < old ) // overflowed past B: done refining
          break;
      }
      else
        break;
    }

mulsub:
    // D4: un[j .. j+n] -= qhat * dn
    {
      u64 mul_carry = 0;
      u8  borrow    = 0;

      for ( size_t i = 0; i < n; i++ )
      {
        u64 hi;
        u64 lo = mul64x64( qhat, dn[i], &hi );

        u64 t = lo + mul_carry;
        hi += ( t < lo );

        borrow    = subBorrow64( borrow, un[j + i], t, &un[j + i] );
        mul_carry = hi;
      }

      borrow = subBorrow64( borrow, un[j + n], mul_carry, &un[j + n] );

      // D6: qhat was one too large (rare) - add the divisor back
      if ( borrow )
      {
        qhat--;

        u8 carry = 0;
        for ( size_t i = 0; i < n; i++ )
          carry = addCarry64( carry, un[j + i], dn[i], &un[j + i] );

        un[j + n] += carry; // wraps around, cancelling the borrow
      }
    }

    q[j] = qhat;
  }

  while ( !q.empty() && q.back() == 0 )
    q.pop_back();

  // D8: denormalize the remainder
  r.assign( n, 0 );
  for ( size_t i = 0; i + 1 < n; i++ )
    r[i] = ( s == 0 ) ? un[i] : (un[i] >> s) | (un[i + 1] << (64 - s));
  r[n - 1] = un[n - 1] >> s;

  while ( !r.empty() && r.back() == 0 )
    r.pop_back();
}


// ==================================================================
// BigInt3 itself
// ==================================================================
BigInt3::BigInt3()
  : neg( false )
{
}

void BigInt3::setU64( u64 magnitude, bool negative )
{
  limbs.clear();
  if ( magnitude )
    limbs.push_back( magnitude );

  neg = negative && magnitude != 0;
}

BigInt3::BigInt3( int v ) : BigInt3( (i64)v )
{
}

BigInt3::BigInt3( i64 v )
{
  u64 mag = v < 0 ? (u64)( -(v + 1) ) + 1 : (u64)v; // careful with INT64_MIN
  setU64( mag, v < 0 );
}

BigInt3::BigInt3( u64 v )
{
  setU64( v, false );
}

BigInt3::BigInt3( char const * decimal_str )
  : neg( false )
{
  ZgAssertRelease( decimal_str != nullptr && *decimal_str != 0 );

  bool minus = ( *decimal_str == '-' );
  if ( minus )
    decimal_str++;

  for ( char const * p = decimal_str; *p; )
  {
    // consume up to 19 digits at once: 10^19 still fits in u64
    u64 chunk = 0;
    u64 base  = 1;

    for ( int k = 0; k < 19 && *p; k++, p++ )
    {
      ZgAssertRelease( *p >= '0' && *p <= '9' );
      chunk = chunk * 10 + (u64)( *p - '0' );
      base *= 10;
    }

    mulAddSmall( limbs, base, chunk );
  }

  neg = minus && !limbs.empty();
}

void BigInt3::trim()
{
  while ( !limbs.empty() && limbs.back() == 0 )
    limbs.pop_back();

  if ( limbs.empty() )
    neg = false;
}

int BigInt3::bitsNum() const
{
  if ( limbs.empty() )
    return 0;

  return (int)limbs.size() * 64 - clz64( limbs.back() );
}

bool BigInt3::checkBit( int i ) const
{
  size_t limb = (size_t)i / 64;

  if ( limb >= limbs.size() )
    return false;

  return ( limbs[limb] >> (i % 64) ) & 1;
}

int BigInt3::compare( BigInt3 const& other ) const
{
  if ( neg != other.neg )
    return neg ? -1 : 1;

  int c = cmpAbs( limbs, other.limbs );
  return neg ? -c : c;
}

BigInt3 BigInt3::operator-() const
{
  BigInt3 res = *this;
  if ( !res.limbs.empty() )
    res.neg = !res.neg;
  return res;
}

// *this += (b_limbs with sign b_neg): shared by += and -=
void BigInt3::addSigned( std::vector<u64> const& b_limbs, bool b_neg )
{
  if ( neg == b_neg )
  {
    addAbs( limbs, b_limbs ); // same sign: magnitudes add up
    return;
  }

  int c = cmpAbs( limbs, b_limbs );

  if ( c == 0 )
  {
    limbs.clear();
    neg = false;
  } else
  if ( c > 0 )
  {
    subAbs( limbs, b_limbs ); // |a| > |b|: sign of a wins
  } else
  {
    std::vector<u64> tmp = b_limbs; // |b| > |a|: sign of b wins
    subAbs( tmp, limbs );
    limbs.swap( tmp );
    neg = b_neg;
  }
}

void BigInt3::add( BigInt3 const& a )
{
  addSigned( a.limbs, a.neg );
}

void BigInt3::sub( BigInt3 const& a )
{
  addSigned( a.limbs, !a.neg );
}

BigInt3& BigInt3::operator+=( BigInt3 const& b )
{
  addSigned( b.limbs, b.neg );
  return *this;
}

BigInt3& BigInt3::operator-=( BigInt3 const& b )
{
  addSigned( b.limbs, !b.neg );
  return *this;
}

BigInt3& BigInt3::operator*=( BigInt3 const& b )
{
  if ( limbs.empty() || b.limbs.empty() )
  {
    limbs.clear();
    neg = false;
    return *this;
  }

  std::vector<u64> res;
  mulAbs( limbs, b.limbs, res );
  limbs.swap( res );

  neg = ( neg != b.neg );
  return *this;
}

void BigInt3::divMod( BigInt3 const& a, BigInt3 const& b, BigInt3& q, BigInt3& r )
{
  bool a_neg = a.neg; // capture before writing: q or r may alias a or b
  bool b_neg = b.neg;

  std::vector<u64> ql, rl;
  divModAbs( a.limbs, b.limbs, ql, rl );

  q.limbs.swap( ql );
  q.neg = ( a_neg != b_neg ) && !q.limbs.empty(); // truncation toward zero

  r.limbs.swap( rl );
  r.neg = a_neg && !r.limbs.empty(); // remainder keeps the dividend's sign
}

BigInt3& BigInt3::operator/=( BigInt3 const& b )
{
  BigInt3 q, r;
  divMod( *this, b, q, r );
  limbs.swap( q.limbs );
  neg = q.neg;
  return *this;
}

BigInt3& BigInt3::operator%=( BigInt3 const& b )
{
  BigInt3 q, r;
  divMod( *this, b, q, r );
  limbs.swap( r.limbs );
  neg = r.neg;
  return *this;
}

BigInt3& BigInt3::operator<<=( int bits )
{
  ZgAssert( bits >= 0 );

  if ( limbs.empty() || bits == 0 )
    return *this;

  size_t limb_shift = (size_t)bits / 64;
  int    bit_shift  = bits % 64;

  size_t old_size = limbs.size();
  limbs.resize( old_size + limb_shift + ( bit_shift != 0 ), 0 );

  for ( size_t i = old_size; i-- > 0; )
  {
    u64 v = limbs[i];
    limbs[i] = 0; // will be overwritten unless it stays in the shifted-out gap

    if ( bit_shift == 0 )
    {
      limbs[i + limb_shift] = v;
    }
    else
    {
      limbs[i + limb_shift + 1] |= v >> (64 - bit_shift);
      limbs[i + limb_shift]      = v << bit_shift;
    }
  }

  trim();
  return *this;
}

BigInt3& BigInt3::operator>>=( int bits )
{
  ZgAssert( bits >= 0 );

  size_t limb_shift = (size_t)bits / 64;
  int    bit_shift  = bits % 64;

  if ( limb_shift >= limbs.size() )
  {
    limbs.clear();
    neg = false;
    return *this;
  }

  size_t new_size = limbs.size() - limb_shift;

  for ( size_t i = 0; i < new_size; i++ )
  {
    u64 v = limbs[i + limb_shift] >> bit_shift;

    if ( bit_shift != 0 && i + limb_shift + 1 < limbs.size() )
      v |= limbs[i + limb_shift + 1] << (64 - bit_shift);

    limbs[i] = v;
  }

  limbs.resize( new_size );
  trim();
  return *this;
}

std::string BigInt3::toString() const
{
  if ( limbs.empty() )
    return "0";

  const u64 CHUNK = 10000000000000000000ull; // 10^19

  std::vector<u64> tmp = limbs;
  std::string rev; // digits collected in reverse order

  while ( !tmp.empty() )
  {
    u64 part = divSmall( tmp, CHUNK );

    if ( tmp.empty() ) // most significant chunk: no zero padding
    {
      do
      {
        rev += (char)( '0' + part % 10 );
        part /= 10;
      }
      while ( part );
    }
    else // inner chunk: exactly 19 digits, zero padded
    {
      for ( int k = 0; k < 19; k++ )
      {
        rev += (char)( '0' + part % 10 );
        part /= 10;
      }
    }
  }

  if ( neg )
    rev += '-';

  return std::string( rev.rbegin(), rev.rend() );
}

std::string BigInt3::toHex() const
{
  if ( limbs.empty() )
    return "0";

  static char const digits[] = "0123456789abcdef";

  std::string res;
  res.reserve( limbs.size() * 16 + 1 );

  if ( neg )
    res += '-';

  bool leading = true;

  for ( size_t i = limbs.size(); i-- > 0; )
  {
    for ( int k = 60; k >= 0; k -= 4 )
    {
      int d = (int)( ( limbs[i] >> k ) & 0xf );

      if ( leading && d == 0 )
        continue;

      leading = false;
      res += digits[d];
    }
  }

  return res;
}

BigInt3 BigInt3::modPow( BigInt3 const& e, BigInt3 const& m ) const
{
  ZgAssertRelease( !e.neg && !m.limbs.empty() );

  BigInt3 base = *this % m;
  if ( base.neg )
    base += m;

  BigInt3 res( 1 );

  for ( int i = e.bitsNum(); i-- > 0; )
  {
    res *= res;
    res %= m;

    if ( e.checkBit( i ) )
    {
      res *= base;
      res %= m;
    }
  }

  return res;
}

// this = this ^ power, binary exponentiation (the old version multiplied
// in a loop: O(power) full multiplications, this one does O(log power))
void BigInt3::pow( int power )
{
  ZgAssertRelease( power >= 0 );

  BigInt3 base = *this;
  *this = BigInt3( 1 ); // 0^0 == 1

  while ( power > 0 )
  {
    if ( power & 1 )
      *this *= base;

    power >>= 1;

    if ( power )
      base *= base;
  }
}

// this = this ^ power mod p (mutating twin of modPow, kept under the old name)
void BigInt3::powModP( BigInt3 const& power, BigInt3 const& p )
{
  *this = modPow( power, p );
}

// this %= b; sign semantics match operator%= (remainder keeps dividend's sign)
void BigInt3::calcRemainder( BigInt3 const& b )
{
  *this %= b;
}

BigInt3 BigInt3::gcd( BigInt3 a, BigInt3 b )
{
  a.neg = false;
  b.neg = false;

  while ( !b.isZero() )
  {
    BigInt3 r = a % b;
    a = b;
    b = r;
  }

  return a;
}

} // namespace zygo
