#include "big_int_32.h"
#include <zygo/core/assert.h>

#if defined(_MSC_VER)
#include <intrin.h> // _BitScanReverse, outside any namespace
#endif


namespace zygo {

// count leading zeros of a 32-bit word (v != 0).
// arm-gcc emits the single CLZ instruction; MSVC uses _BitScanReverse.
static inline int clz32( u32 v )
{
#if defined(_MSC_VER)
  unsigned long idx;
  _BitScanReverse( &idx, v );
  return 31 - (int)idx;
#else
  return __builtin_clz( v );
#endif
}

// ==================================================================
// The whole point of 32-bit limbs: a 64-bit accumulator holds any
// limb product plus carries with no overflow and no special hardware.
// (2^32-1)^2 + 2*(2^32-1) = 2^64 - 1 fits exactly in u64.
// ==================================================================

// one inner step of schoolbook multiplication: r += a * b + carry
static inline void mulStep( u32 a, u32 b, u32 & r, u32 & carry )
{
  u64 t = (u64)a * b + r + carry;
  r     = (u32)t;
  carry = (u32)( t >> 32 );
}


// ==================================================================
// magnitude helpers (unsigned vectors of 32-bit limbs)
// ==================================================================
int BigInt32::cmpAbs( std::vector<u32> const& a, std::vector<u32> const& b )
{
  if ( a.size() != b.size() )
    return a.size() < b.size() ? -1 : 1;

  for ( size_t i = a.size(); i-- > 0; )
    if ( a[i] != b[i] )
      return a[i] < b[i] ? -1 : 1;

  return 0;
}

void BigInt32::addAbs( std::vector<u32> & a, std::vector<u32> const& b )
{
  if ( a.size() < b.size() )
    a.resize( b.size(), 0 );

  u64 carry = 0;
  size_t i = 0;

  for ( ; i < b.size(); i++ )
  {
    u64 s = (u64)a[i] + b[i] + carry;
    a[i]  = (u32)s;
    carry = s >> 32;
  }

  for ( ; carry && i < a.size(); i++ )
  {
    u64 s = (u64)a[i] + carry;
    a[i]  = (u32)s;
    carry = s >> 32;
  }

  if ( carry )
    a.push_back( (u32)carry );
}

// requires a >= b
void BigInt32::subAbs( std::vector<u32> & a, std::vector<u32> const& b )
{
  u64 borrow = 0;
  size_t i = 0;

  for ( ; i < b.size(); i++ )
  {
    u64 d  = (u64)a[i] - b[i] - borrow;
    a[i]   = (u32)d;
    borrow = (d >> 32) & 1; // high word is 0xFFFFFFFF on underflow
  }

  for ( ; borrow && i < a.size(); i++ )
  {
    u64 d  = (u64)a[i] - borrow;
    a[i]   = (u32)d;
    borrow = (d >> 32) & 1;
  }

  while ( !a.empty() && a.back() == 0 )
    a.pop_back();
}

void BigInt32::mulAbs( std::vector<u32> const& a, std::vector<u32> const& b,
  std::vector<u32> & res )
{
  res.assign( a.size() + b.size(), 0 );

  u32 *       r  = res.data();
  u32 const * bp = b.data();
  size_t      bn = b.size();

  for ( size_t i = 0; i < a.size(); i++, r++ )
  {
    u32 ai = a[i];
    if ( ai == 0 )
      continue;

    u32 carry = 0;

    for ( size_t j = 0; j < bn; j++ )
      mulStep( ai, bp[j], r[j], carry );

    r[bn] = carry;
  }

  while ( !res.empty() && res.back() == 0 )
    res.pop_back();
}

// a = a * mul + add (for decimal parsing)
void BigInt32::mulAddSmall( std::vector<u32> & a, u32 mul, u32 add )
{
  u64 carry = add;

  for ( size_t i = 0; i < a.size(); i++ )
  {
    u64 cur = (u64)a[i] * mul + carry;
    a[i]    = (u32)cur;
    carry   = cur >> 32;
  }

  while ( carry )
  {
    a.push_back( (u32)carry );
    carry >>= 32;
  }
}

// a /= d, returns the remainder (for decimal printing).
// Note the elegance of 32-bit limbs: "two limbs / one limb" is a
// native u64 / u32 divide -- no 128-bit division routine needed.
u32 BigInt32::divSmall( std::vector<u32> & a, u32 d )
{
  u64 rem = 0;

  for ( size_t i = a.size(); i-- > 0; )
  {
    u64 cur = (rem << 32) | a[i];
    a[i]    = (u32)( cur / d );
    rem     = cur % d;
  }

  while ( !a.empty() && a.back() == 0 )
    a.pop_back();

  return (u32)rem;
}

// Knuth, TAOCP vol. 2, Algorithm D, with 32-bit trial digits.
void BigInt32::divModAbs( std::vector<u32> const& u, std::vector<u32> const& d,
  std::vector<u32> & q, std::vector<u32> & r )
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
    u32 rem = divSmall( q, d[0] );

    r.clear();
    if ( rem )
      r.push_back( rem );
    return;
  }

  size_t n = d.size();
  size_t m = u.size() - n;

  // D1: normalize so the divisor's top limb has its high bit set
  int s = clz32( d.back() );

  std::vector<u32> dn( n );
  for ( size_t i = n; i-- > 1; )
    dn[i] = ( s == 0 ) ? d[i] : (d[i] << s) | (d[i - 1] >> (32 - s));
  dn[0] = d[0] << s;

  std::vector<u32> un( u.size() + 1 );
  un[u.size()] = ( s == 0 ) ? 0 : u.back() >> (32 - s);
  for ( size_t i = u.size(); i-- > 1; )
    un[i] = ( s == 0 ) ? u[i] : (u[i] << s) | (u[i - 1] >> (32 - s));
  un[0] = u[0] << s;

  q.assign( m + 1, 0 );

  for ( size_t j = m + 1; j-- > 0; )
  {
    // D3: estimate the trial digit from the top two limbs
    u32 qhat;
    u64 rhat;

    if ( un[j + n] == dn[n - 1] )
    {
      qhat = 0xffffffffu; // base - 1
      rhat = (u64)un[j + n - 1] + dn[n - 1];
    }
    else
    {
      u64 num = ( (u64)un[j + n] << 32 ) | un[j + n - 1];
      qhat = (u32)( num / dn[n - 1] );
      rhat = num % dn[n - 1];
    }

    // refine using the third limb; qhat*dn[n-2] < 2^64, no overflow
    while ( rhat <= 0xffffffffull &&
      (u64)qhat * dn[n - 2] > ( (rhat << 32) | un[j + n - 2] ) )
    {
      qhat--;
      rhat += dn[n - 1];
    }

    // D4: un[j .. j+n] -= qhat * dn
    u64 carry  = 0;
    u64 borrow = 0;

    for ( size_t i = 0; i < n; i++ )
    {
      u64 p   = (u64)qhat * dn[i] + carry;
      carry   = p >> 32;

      u64 sub = (u64)un[j + i] - (u32)p - borrow;
      un[j + i] = (u32)sub;
      borrow    = (sub >> 32) & 1;
    }

    u64 sub = (u64)un[j + n] - carry - borrow;
    un[j + n] = (u32)sub;
    borrow    = (sub >> 32) & 1;

    // D6: qhat was one too large (rare) - add the divisor back
    if ( borrow )
    {
      qhat--;

      u64 c = 0;
      for ( size_t i = 0; i < n; i++ )
      {
        u64 t = (u64)un[j + i] + dn[i] + c;
        un[j + i] = (u32)t;
        c = t >> 32;
      }
      un[j + n] += (u32)c; // wraps, cancelling the borrow
    }

    q[j] = qhat;
  }

  while ( !q.empty() && q.back() == 0 )
    q.pop_back();

  // D8: denormalize the remainder
  r.assign( n, 0 );
  for ( size_t i = 0; i + 1 < n; i++ )
    r[i] = ( s == 0 ) ? un[i] : (un[i] >> s) | (un[i + 1] << (32 - s));
  r[n - 1] = un[n - 1] >> s;

  while ( !r.empty() && r.back() == 0 )
    r.pop_back();
}


// ==================================================================
// BigInt32 itself (identical logic to BigInt, 32-bit limbs)
// ==================================================================
BigInt32::BigInt32()
  : neg( false )
{
}

void BigInt32::setU64( u64 magnitude, bool negative )
{
  limbs.clear();

  if ( magnitude )
  {
    limbs.push_back( (u32)magnitude );
    if ( magnitude >> 32 )
      limbs.push_back( (u32)( magnitude >> 32 ) );
  }

  neg = negative && magnitude != 0;
}

BigInt32::BigInt32( int v ) : BigInt32( (i64)v )
{
}

BigInt32::BigInt32( i64 v )
{
  u64 mag = v < 0 ? (u64)( -(v + 1) ) + 1 : (u64)v; // careful with INT64_MIN
  setU64( mag, v < 0 );
}

BigInt32::BigInt32( u64 v )
{
  setU64( v, false );
}

BigInt32::BigInt32( char const * decimal_str )
  : neg( false )
{
  ZgAssertRelease( decimal_str != nullptr && *decimal_str != 0 );

  bool minus = ( *decimal_str == '-' );
  if ( minus )
    decimal_str++;

  for ( char const * p = decimal_str; *p; )
  {
    // consume up to 9 digits at once: 10^9 still fits in u32
    u32 chunk = 0;
    u32 base  = 1;

    for ( int k = 0; k < 9 && *p; k++, p++ )
    {
      ZgAssertRelease( *p >= '0' && *p <= '9' );
      chunk = chunk * 10 + (u32)( *p - '0' );
      base *= 10;
    }

    mulAddSmall( limbs, base, chunk );
  }

  neg = minus && !limbs.empty();
}

void BigInt32::trim()
{
  while ( !limbs.empty() && limbs.back() == 0 )
    limbs.pop_back();

  if ( limbs.empty() )
    neg = false;
}

int BigInt32::bitsNum() const
{
  if ( limbs.empty() )
    return 0;

  return (int)limbs.size() * 32 - clz32( limbs.back() );
}

bool BigInt32::checkBit( int i ) const
{
  size_t limb = (size_t)i / 32;

  if ( limb >= limbs.size() )
    return false;

  return ( limbs[limb] >> (i % 32) ) & 1;
}

int BigInt32::compare( BigInt32 const& other ) const
{
  if ( neg != other.neg )
    return neg ? -1 : 1;

  int c = cmpAbs( limbs, other.limbs );
  return neg ? -c : c;
}

BigInt32 BigInt32::operator-() const
{
  BigInt32 res = *this;
  if ( !res.limbs.empty() )
    res.neg = !res.neg;
  return res;
}

void BigInt32::addSigned( std::vector<u32> const& b_limbs, bool b_neg )
{
  if ( neg == b_neg )
  {
    addAbs( limbs, b_limbs );
    return;
  }

  int c = cmpAbs( limbs, b_limbs );

  if ( c == 0 )
  {
    limbs.clear();
    neg = false;
  }
  else if ( c > 0 )
  {
    subAbs( limbs, b_limbs );
  }
  else
  {
    std::vector<u32> tmp = b_limbs;
    subAbs( tmp, limbs );
    limbs.swap( tmp );
    neg = b_neg;
  }
}

void BigInt32::add( BigInt32 const& b ) { addSigned( b.limbs,  b.neg ); }
void BigInt32::sub( BigInt32 const& b ) { addSigned( b.limbs, !b.neg ); }

BigInt32& BigInt32::operator+=( BigInt32 const& b )
{
  addSigned( b.limbs, b.neg );
  return *this;
}

BigInt32& BigInt32::operator-=( BigInt32 const& b )
{
  addSigned( b.limbs, !b.neg );
  return *this;
}

BigInt32& BigInt32::operator*=( BigInt32 const& b )
{
  if ( limbs.empty() || b.limbs.empty() )
  {
    limbs.clear();
    neg = false;
    return *this;
  }

  std::vector<u32> res;
  mulAbs( limbs, b.limbs, res );
  limbs.swap( res );

  neg = ( neg != b.neg );
  return *this;
}

void BigInt32::divMod( BigInt32 const& a, BigInt32 const& b, BigInt32& q, BigInt32& r )
{
  bool a_neg = a.neg; // capture before writing: q or r may alias a or b
  bool b_neg = b.neg;

  std::vector<u32> ql, rl;
  divModAbs( a.limbs, b.limbs, ql, rl );

  q.limbs.swap( ql );
  q.neg = ( a_neg != b_neg ) && !q.limbs.empty();

  r.limbs.swap( rl );
  r.neg = a_neg && !r.limbs.empty();
}

BigInt32& BigInt32::operator/=( BigInt32 const& b )
{
  BigInt32 q, r;
  divMod( *this, b, q, r );
  limbs.swap( q.limbs );
  neg = q.neg;
  return *this;
}

BigInt32& BigInt32::operator%=( BigInt32 const& b )
{
  BigInt32 q, r;
  divMod( *this, b, q, r );
  limbs.swap( r.limbs );
  neg = r.neg;
  return *this;
}

BigInt32& BigInt32::operator<<=( int bits )
{
  ZgAssert( bits >= 0 );

  if ( limbs.empty() || bits == 0 )
    return *this;

  size_t limb_shift = (size_t)bits / 32;
  int    bit_shift  = bits % 32;

  size_t old_size = limbs.size();
  limbs.resize( old_size + limb_shift + ( bit_shift != 0 ), 0 );

  for ( size_t i = old_size; i-- > 0; )
  {
    u32 v = limbs[i];
    limbs[i] = 0;

    if ( bit_shift == 0 )
    {
      limbs[i + limb_shift] = v;
    }
    else
    {
      limbs[i + limb_shift + 1] |= v >> (32 - bit_shift);
      limbs[i + limb_shift]      = v << bit_shift;
    }
  }

  trim();
  return *this;
}

BigInt32& BigInt32::operator>>=( int bits )
{
  ZgAssert( bits >= 0 );

  size_t limb_shift = (size_t)bits / 32;
  int    bit_shift  = bits % 32;

  if ( limb_shift >= limbs.size() )
  {
    limbs.clear();
    neg = false;
    return *this;
  }

  size_t new_size = limbs.size() - limb_shift;

  for ( size_t i = 0; i < new_size; i++ )
  {
    u32 v = limbs[i + limb_shift] >> bit_shift;

    if ( bit_shift != 0 && i + limb_shift + 1 < limbs.size() )
      v |= limbs[i + limb_shift + 1] << (32 - bit_shift);

    limbs[i] = v;
  }

  limbs.resize( new_size );
  trim();
  return *this;
}

std::string BigInt32::toString() const
{
  if ( limbs.empty() )
    return "0";

  const u32 CHUNK = 1000000000u; // 10^9

  std::vector<u32> tmp = limbs;
  std::string rev;

  while ( !tmp.empty() )
  {
    u32 part = divSmall( tmp, CHUNK );

    if ( tmp.empty() ) // most significant chunk: no zero padding
    {
      do
      {
        rev += (char)( '0' + part % 10 );
        part /= 10;
      }
      while ( part );
    }
    else // inner chunk: exactly 9 digits, zero padded
    {
      for ( int k = 0; k < 9; k++ )
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

std::string BigInt32::toHex() const
{
  if ( limbs.empty() )
    return "0";

  static char const digits[] = "0123456789abcdef";

  std::string res;
  res.reserve( limbs.size() * 8 + 1 );

  if ( neg )
    res += '-';

  bool leading = true;

  for ( size_t i = limbs.size(); i-- > 0; )
  {
    for ( int k = 28; k >= 0; k -= 4 )
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

BigInt32 BigInt32::modPow( BigInt32 const& e, BigInt32 const& m ) const
{
  ZgAssertRelease( !e.neg && !m.limbs.empty() );

  BigInt32 base = *this % m;
  if ( base.neg )
    base += m;

  BigInt32 res( 1 );

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

BigInt32 BigInt32::gcd( BigInt32 a, BigInt32 b )
{
  a.neg = false;
  b.neg = false;

  while ( !b.isZero() )
  {
    BigInt32 r = a % b;
    a = b;
    b = r;
  }

  return a;
}

void BigInt32::pow( int power )
{
  ZgAssertRelease( power >= 0 );

  BigInt32 base = *this;
  *this = BigInt32( 1 ); // 0^0 == 1

  while ( power > 0 )
  {
    if ( power & 1 )
      *this *= base;

    power >>= 1;

    if ( power )
      base *= base;
  }
}

void BigInt32::powModP( BigInt32 const& power, BigInt32 const& p )
{
  *this = modPow( power, p );
}

void BigInt32::calcRemainder( BigInt32 const& b )
{
  *this %= b;
}

} // namespace zygo
