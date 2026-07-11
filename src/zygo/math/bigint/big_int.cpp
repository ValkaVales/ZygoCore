#include "big_int.h"
#include <zygo/core/encoding/digit.h>
#include <zygo/core/assert.h>
#include <zygo/core/str.h>
#include <zygo/math/common/scalar.h>
#include <iostream>


namespace zygo {

BigInt::BigInt()
  : len   ( 0 )
  , sign  ( Sign::PLUS )
{
  val[0] = 0;
}

BigInt::BigInt( BIType a )
  : len   ( 1 )
  , sign  ( a < 0 ? Sign::MINUS : Sign::PLUS )
{
  val[0] = a < 0 ? -a : a;
  val[1] = 0;
}

BigInt::BigInt( BigInt const& a )
  : len   ( a.len )
  , sign  ( a.sign )
{
  for ( int i = 0; i <= len; i++ )
    val[i] = a.val[i];
}

BigInt::BigInt( char const* s, int radix )
  : len   ( 0 )
  , sign  ( Sign::PLUS )
{
  set( s, radix );
}

void BigInt::set( BIType a )
{
  if ( a < 0 )
  {
    a = -a;
    sign = Sign::MINUS;
  } else
    sign = Sign::PLUS;
  
  val[0] = a;
  val[1] = 0;
  len = 1;
}

void BigInt::set( char const * s, int radix )
{
  len = 0;
  val[0] = 0;
  val[1] = 0;

  BigInt bradix( radix );

  int slen = my_strlen( s );
  int i = 0;

  sign = Sign::PLUS;
  if ( s[0] == '-' )
  {
    sign = Sign::MINUS;
    ++i;
  }

  for ( ; i < slen; ++i )
  {
    char c = s[i];
    int k = digitToValue( c, radix );
    mul( bradix );
    add( k );
    //add( BigInt( k ) );
  }
}

void BigInt::copyFrom( BigInt const& a )
{
  sign  = a.sign;
  len   = a.len;

  for ( int i = 0; i <= a.len; i++ )
    val[i] = a.val[i];
}


int BigInt::bitsNum() const
{
  int x = len * BIGINT_BPU;
  while ( x > 0 && checkBit( x - 1 ) == 0 )
    --x;
  return x;
}


void BigInt::add( BigInt const& a )
{
  int minlen;
  int maxlen;
  getMinMax( len, a.len, minlen, maxlen );

  int i = 0;

  BIType const * p1 = a.val;
  BIType * p = val;

  u8 carry = 0;
  for ( ; i <= minlen; ++i )
  {
    BIType u = (*p1++) + (*p) + carry;
    
    if ( u < 0 )
    {
      carry = 1;
      u &= BIGINT_MASK;
    } else
      carry = 0;

    *p++ = u;
  }

  if ( len < a.len )
  {
    for ( ; i <= a.len; ++i )
    {
      BIType u = carry + *p1++;

      if ( u < 0 )
      {
        carry = 1;
        u &= BIGINT_MASK;
      } else
        carry = 0;

      *p++ = u;
    }
  } else
  {
    for ( ; i <= len && carry; ++i )
    {
      BIType u = carry + *p;

      if ( u < 0 )
      {
        carry = 1;
        u &= BIGINT_MASK;
      } else
        carry = 0;

      *p++ = u;
    }
  }

  len = maxlen + 1;
  val[len] = 0;
  while ( len > 0 && val[len - 1] == 0 )
    len--;
}


void BigInt::sub( BigInt const& a )
{
  ZgAssert( a.len <= len );

  BIType carry = 0;
  int i = 0;

  for ( ; i < len; i++ )
  {
    BIType ua = carry;
    if ( i < a.len )
      ua += a.val[i];

    if ( ua < carry )
      continue;

    BIType nu = val[i] - ua;
      
    if ( nu < 0 )
    {
      carry = 1;
      nu &= BIGINT_MASK;
    } else
      carry = 0;

    val[i] = nu;
  }

  while ( len > 0 && val[len - 1] == 0 )
    --len;
}

void BigInt::signSub( BigInt const& a )
{
  // sign operations
  if ( sign == Sign::PLUS && a.sign == Sign::PLUS )
  {
    if ( compare( a ) < 0 )
    {
      BigInt t( a );
      t.sub( *this );
      
      copyFrom( t );
      sign = Sign::MINUS;
    } else
    {
      sub( a );
      sign = Sign::PLUS;
    }
    return;
  }

  if ( sign == Sign::PLUS && a.sign == Sign::MINUS )
  {
    add( a );
    sign = Sign::PLUS;
    return;
  }

  if ( sign == Sign::MINUS && a.sign == Sign::PLUS )
  {
    add( a );
    sign = Sign::MINUS;
    return;
  }

  if ( sign == Sign::MINUS && a.sign == Sign::MINUS )
  {
    if ( compare( a ) < 0 )
    {
      BigInt t( a );
      t.sub( *this );
      
      copyFrom( a );
      sign = Sign::PLUS;
    } else
    {
      sub( a );
      sign = Sign::MINUS;
      if ( isZero() )
        sign = Sign::PLUS;
    }
  }
}


void BigInt::add( BIType a )
{
  BIType carry = 0;
  int maxlen = len;

  if ( maxlen < 1 )
    maxlen = 1;

  int i;
  for ( i = len; i <= maxlen; i++ )
    val[i] = 0;

  for ( i = 0; i < maxlen + 1; i++ )
  {
    BIType u = val[i] + carry;
    carry = 0;
    
    if ( u < 0 )
    {
      carry = 1;
      u &= BIGINT_MASK;
    }

    //u += (i == 0) ? a : 0; // TODO: optimize!
    if ( !i )
      u += a;

    if ( u < 0 )
    {
      carry++;
      u &= BIGINT_MASK;
    }

    val[i] = u;
  }

  len = maxlen + 1;
  while ( len > 0 && val[len - 1] == 0 )
    --len;
}


void BigInt::sub( BIType a )
{
  BIType carry = 0;
  int N = len;

  for ( int i = 0; i < N; i++ )
  {
    BIType ua = ((i == 0) ? a : 0) + carry;
    if ( ua >= carry )
    {
      BIType nu = val[i] - ua;
      
      if ( nu < 0 )
      {
        carry = 1;
        nu &= BIGINT_MASK;
      } else
        carry = 0;

      val[i] = nu;
    }
  }
  
  while ( len > 0 && val[len - 1] == 0 )
    --len;
}

void BigInt::printRaw( char const * name )
{
  printf( "printRaw()  len %d   %s\n", len, name ? name : "-" );
  for ( int i = 0; i < len; ++i )
    printf( "%lld\n", val[i] );
  printf( "\n" );
}

void BigInt::print( int radix, char const * name )
{
  if ( name )
    std::cout << name << ": ";

  if ( len == 1 && radix == 10 )
  {
    std::cout << val[0] << std::endl;
    return;
  }

  ZgAssert( radix > 0 );

  const int BUF_SZ = BIGINT_LEN_MAX * 22;
  char buf[BUF_SZ];
  int k = BUF_SZ - 1;
  buf[k--] = 0;

  BigInt a( *this );
  while ( !a.isZero() )
  {
    BigInt reminder;
    a.div( radix, reminder );

    BIType val = reminder.val[0];
    ZgAssert( val < (BIType)radix );
    
    buf[k--] = valueToDigit_upperCase( (u8)val, radix );
  }

  char const * s = buf + k + 1;
  std::string st( s );
  //for ( int i = len - 1; i >= 0; i-- )
  //  std::cout << val[i];
  std::cout << st << std::endl;
}

void BigInt::shiftLeft()
{
  BIType carry = 0;
  for ( int i = 0; i <= len; i++ )
  {
    BIType u = val[i];
    val[i] = ((u << 1) + carry) & BIGINT_MASK;
    carry = u >> (BIGINT_BPU - 1);
  }

  if ( val[len] )
    val[++len] = 0;
}
/*
void BigInt::shiftLeft( int x )
{
  BIType carry = 0;
  for ( int i = 0; i <= len; i++ )
  {
    BIType u = val[i];
    val[i] = ((u << 1) + carry) & BIGINT_MASK;
    carry = u >> (BIGINT_BPU - 1);
  }

  if ( val[len] )
    val[++len] = 0;
}
*/
void BigInt::bigShiftLeft()
{
  for ( int i = len + 1; i > 0; --i )
    val[i] = val[i-1];

  val[0] = 0;
  ++len;
}

void BigInt::shiftRight()
{
  BIType carry = 0;
  int i = len;
  while ( i != 0 )
  {
    i--;
    BIType u = val[i];
    val[i] = (u >> 1) + carry;
    carry = (u << (BIGINT_BPU - 1)) & BIGINT_MASK;
  }

  if ( val[len - 1] == 0 )
    --len;
}

void BigInt::shiftRight( int x )
{
  if ( !x )
    return;

  int delta = x / BIGINT_BPU;
  x %= BIGINT_BPU;

  for ( int i = 0; i < len; i++ )
  {
    BIType u = val[i + delta];
    u >>= x;
    u += val[i + delta + 1] << (BIGINT_BPU - x);
    u &= BIGINT_MASK;
    val[i] = u;
  }

  while ( val[len - 1] == 0 )
    --len;
}


int BigInt::compare( BigInt const& a ) const
{
  if ( len > a.len ) return +1;
  if ( len < a.len ) return -1;
  
  int i = len;
  while ( i-- )
  {
    if ( val[i] > a.val[i] ) return +1;
    if ( val[i] < a.val[i] ) return -1;
  }

  return 0;
}


void BigInt::mul( BigInt const& b )
{
  mul( b, BIGINT_LEN_MAX * 64 );
}

void BigInt::signMul( BigInt const& b )
{
  int flag = 0;
  if ( sign != b.sign )
    flag = 1;

  mul( b, BIGINT_LEN_MAX * 64 );

  sign = flag ? Sign::MINUS : Sign::PLUS;
}

void BigInt::mul( BigInt const& b, int keep )
{
  ZgAssert( len + b.len + 1 <= BIGINT_LEN_MAX );

  BigInt t;
  int i;
  int limit = (keep + BIGINT_BPU - 1) / BIGINT_BPU;
  
  if ( limit > BIGINT_LEN_MAX )
    limit = BIGINT_LEN_MAX;

  for ( i = 0; i < limit; i++ )
    t.val[i] = 0;

  int minlen = len;
  if ( minlen > limit )
    minlen = limit;

  for ( i = 0; i < minlen; i++ )
  {
    BIType m = val[i];
    BIType c = 0; // carry
    BIType w, v, p;
    BIType lo_p, hi_p;
    BIType lo_m, hi_m;

    lo_m = m & BIGINT_LO_MASK;
    hi_m = m >> BIGINT_BPU2;

    int min2 = i + b.len;
    if ( min2 > limit ) min2 = limit;

    int j;
    for ( j = i; j < min2; j++ )
    {
      // {c:a[j]} = a[j] + c + m*b.a[j-i];
      v = t.val[j];
      p = b.val[j - i];

      lo_p = p & BIGINT_LO_MASK;
      hi_p = p >> BIGINT_BPU2;

      v += c;
      c = 0;
      if ( v < 0 )
      {
        c++;
        v &= BIGINT_MASK;
      }

      w = lo_p * lo_m;
      if ( w < 0 )
      {
        c++;
        w &= BIGINT_MASK;
      }

      v += w;
      if ( v < 0 )
      {
        c++;
        v &= BIGINT_MASK;
      }

      //
      w = lo_p * hi_m;
      c += w >> (BIGINT_BPU2 - 1);

      w <<= BIGINT_BPU2;
      w &= BIGINT_MASK;
      v += w;
      if ( v < 0 )
      {
        c++;
        v &= BIGINT_MASK;
      }

      //
      w = hi_p * lo_m;
      c += w >> (BIGINT_BPU2 - 1);

      w <<= BIGINT_BPU2;
      w &= BIGINT_MASK;
      v += w;
      if ( v < 0 )
      {
        c++;
        v &= BIGINT_MASK;
      }

      //
      c += (hi_p * hi_m) << 1;
      t.val[j] = v;
    }

    while ( c != 0 && j < limit )
    {
      t.val[j] += c;
      c = (t.val[j] < c);
      j++;
    }
  }

  // eliminate unwanted bits
  keep %= BIGINT_BPU;
  if ( keep )
    t.val[limit - 1] &= ((BIType)1 << keep) - 1;

  t.len = len + b.len + 1;
  while ( t.len && t.val[t.len - 1] == 0 )
    t.len--;

  copyFrom( t );
}

void BigInt::div( BigInt const& b, BigInt& rem )
{
  BigInt t( 0 );
  rem.copyFrom( *this );

  BigInt m( b );
  BigInt s( 1 );

  while ( rem.len > m.len + 1 )
  {
    m.bigShiftLeft();
    s.bigShiftLeft();
  }

  while ( rem.compare( m ) > 0 )
  {
    m.shiftLeft();
    s.shiftLeft();
  }

  while ( rem.compare( b ) >= 0 )
  {
    while ( rem.compare( m ) < 0 )
    {
      m.shiftRight();
      s.shiftRight();
    }
    rem.sub( m );
    t.add( s );
  }

  copyFrom( t );
  while ( len > 0 && val[len - 1] == 0 )
    --len;
}

void BigInt::calcRemainder( BigInt const& b )
{
  BigInt rem( *this );
  BigInt m( b );

  while ( rem.len > m.len + 1 )
    m.bigShiftLeft();

  while ( rem.compare( m ) > 0 )
    m.shiftLeft();

  while ( rem.compare( b ) >= 0 )
  {
    while ( rem.compare( m ) < 0 )
      m.shiftRight();

    rem.sub( m );
  }

  copyFrom( rem );
}

void BigInt::signDiv( BigInt const& b, BigInt& rem )
{
  int flag = 0;
  if ( sign != b.sign ) flag = 1;

  div( b, rem );

  sign = flag ? Sign::MINUS : Sign::PLUS;
}


void BigInt::divInt( BIType b, BIType& rem )
{
  BigInt b1( b );
  BigInt rem1;

  div( b, rem1 );

  rem = rem1.val[0];
}

void BigInt::pow( int power )
{
  BigInt t( *this );
  set( 1 );

  for ( int i = 0; i < power; i++ )
    mul( t );
}

void BigInt::powModP( BigInt power, BigInt const& p )
{
  BigInt t( 1 );
  calcRemainder( p );
  
  while ( !power.isZero() )
  {
    if ( power.isOdd() )
    {
      t.mul( *this );
      t.calcRemainder( p );
    }

    power.shiftRight();

    mul( *this );
    calcRemainder( p );
  }

  copyFrom( t );
}

void BigInt::modExpSimple( BigInt const& e, BigInt const& M )
{
  BigInt t( 1 );
  if ( e.isZero() )
  {
    copyFrom( t );
    return;
  }

  if ( compare( M ) >= 0 )
    t.calcRemainder( M );
  else
    t.copyFrom( *this );

  int i = e.bitsNum() - 1;

  BigInt t2( t );

  while ( i > 0 )
  {
    i--;
    t2.mul( t2 );
    if ( t2.compare( M ) >= 0 )
      t2.calcRemainder( M );

    if ( e.checkBit( i ) )
    {
      t2.mul( t );
      if ( t2.compare( M ) >= 0 )
        t2.calcRemainder( M );
    }
  }

  copyFrom( t2 );
}

void BigInt::modInverse( BigInt const& m )
{
  BigInt j( 1 );
  BigInt i( 0 );
  BigInt b( m );
  BigInt c( *this );
  BigInt x;
  BigInt y;

  while ( !c.isZero() )
  {
    x.copyFrom( b );
    x.signDiv( c, y );
    b.copyFrom( c );
    c.copyFrom( y );
    y.copyFrom( j );

    j.signMul( x );
    i.signSub( j );
    j.copyFrom( i );

    i.copyFrom( y );
  }

  if ( i.isNegative() )
  {
    // i += m;
    j.copyFrom( m );
    j.sub( i );
    j.sign = Sign::PLUS;

    copyFrom( j );

    return;
  }

  copyFrom( i );
}


BIType BigInt::greatestCommonDivisor( BigInt const& a, BigInt const& b )
{
  BigInt x( a );
  BigInt y( b );

  while ( true )
  {
    if ( y.isZero() )
      return x.val[0];

    BigInt t( x );
    t.div( y, x );

    if ( x.isZero() )
      return y.val[0];

    t.copyFrom( y );
    t.div( x, y );
  }
}

} // namespace zygo
