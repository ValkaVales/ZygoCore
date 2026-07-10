#include "my_random.h"
#include <zygo/math/common/scalar.h>


namespace zygo {

u64 MyRandom::s[4] = { 0x9E3779B97F4A7C15ULL, 0xBF58476D1CE4E5B9ULL, 0x94D049BB133111EBULL, 0x2545F4914F6CDD1DULL };

bool   MyRandom::has_gauss_next = false;
Real MyRandom::gauss_next = REAL_ZERO;


// splitmix64 — стандартный способ развернуть один сид в полное состояние
static u64 splitmix64( u64& x )
{
  u64 z = (x += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

void MyRandom::seed( u64 v )
{
  for ( int i = 0; i < 4; ++i )
    s[i] = splitmix64( v );

  has_gauss_next = false;
  gauss_next = REAL_ZERO;
}


// полярный метод Марсальи: гауссиана без sin/cos, оба значения в дело
Real MyRandom::gauss( Real mean, Real sigma )
{
  if ( sigma <= REAL_ZERO )
    return mean;

  if ( has_gauss_next )
  {
    has_gauss_next = false;
    return mean + sigma * gauss_next;
  }

  Real u, v, s2;
  do
  {
    u  = rand( -REAL_ONE, REAL_ONE );
    v  = rand( -REAL_ONE, REAL_ONE );
    s2 = u * u + v * v;
  } while ( s2 >= REAL_ONE || s2 == REAL_ZERO );

  Real k = std::sqrt( -REAL_TWO * std::log( s2 ) / s2 );

  gauss_next = v * k;
  has_gauss_next = true;

  return mean + sigma * u * k;
}

Real MyRandom::oldGauss( Real mean, Real sigma )
{
  if ( le( sigma, REAL_ZERO, SMALL_EPSILON ) )
    return mean;

  Real u1 = rand01();
  Real u2 = rand01();

  if ( le( u1, REAL_ZERO, SMALL_EPSILON ) )
    return REAL_ZERO;

  Real d2 = PI_MUL_2 * u2;

  Real a = std::log( u1 );
  Real b1 = std::cos( d2 );
  Real mag = sigma * std::sqrt( -REAL_TWO * a );

  Real x1 = mag * b1;
  //Real x2 = mag * b2;

  x1 += mean;

  return x1;
}

Vector3 MyRandom::gauss( Vector3 const& mean, Real sigma )
{
  return Vector3(
    gauss( mean.getX(), sigma ),
    gauss( mean.getY(), sigma ),
    gauss( mean.getZ(), sigma )
  );
}

} // namespace zygo
