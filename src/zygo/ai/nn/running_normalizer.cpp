#include "running_normalizer.h"
#include <zygo/core/assert.h>
#include <zygo/core/io/byte_reader.h>
#include <zygo/core/io/byte_writer.h>
#include <cmath>


namespace zygo {
namespace nn {

static const Real MIN_VARIANCE = (Real)1e-8;


RunningNormalizer::RunningNormalizer( int size, Real clip_sigmas )
  : mean          ( (size_t)size, REAL_ZERO )
  , m2            ( (size_t)size, REAL_ZERO )
  , inv_std_cache ( (size_t)size, REAL_ONE  )
  , count         ( REAL_ZERO )
  , clip_sigmas   ( clip_sigmas )
  , frozen        ( false )
  , cache_valid   ( false )
{
  ZgAssert( size > 0 );
}

void RunningNormalizer::reset()
{
  for ( size_t i = 0; i < mean.size(); ++i )
  {
    mean[i] = REAL_ZERO;
    m2  [i] = REAL_ZERO;
  }

  count = REAL_ZERO;
  cache_valid = false;
}

void RunningNormalizer::update( Real const* x )
{
  if ( frozen )
    return;

  count += REAL_ONE;

  const Real inv_n = REAL_ONE / count;

  for ( size_t i = 0; i < mean.size(); ++i )
  {
    const Real d = x[i] - mean[i];
    mean[i] += d * inv_n;
    m2  [i] += d * ( x[i] - mean[i] ); // uses the UPDATED mean - that is the whole trick
  }

  cache_valid = false;
}

void RunningNormalizer::update( Real const* x, int n )
{
  const int sz = size();

  for ( int k = 0; k < n; ++k )
    update( x + (size_t)k * (size_t)sz );
}

void RunningNormalizer::rebuildCache()
{
  const Real denom = ( count > REAL_ONE ) ? ( count - REAL_ONE ) : REAL_ONE;

  for ( size_t i = 0; i < mean.size(); ++i )
  {
    Real var = m2[i] / denom;

    if ( var < MIN_VARIANCE )
      var = MIN_VARIANCE;

    inv_std_cache[i] = REAL_ONE / std::sqrt( var );
  }

  cache_valid = true;
}

void RunningNormalizer::normalize( Real const* x, Real* out )
{
  // Before the very first samples arrive there is no scale to speak of: pass the input through
  // untouched rather than dividing by a made-up sigma.
  if ( count < REAL_TWO )
  {
    for ( size_t i = 0; i < mean.size(); ++i )
      out[i] = x[i];

    return;
  }

  if ( !cache_valid )
    rebuildCache();

  for ( size_t i = 0; i < mean.size(); ++i )
  {
    Real v = ( x[i] - mean[i] ) * inv_std_cache[i];

    if ( clip_sigmas > REAL_ZERO )
    {
      if ( v >  clip_sigmas ) v =  clip_sigmas;
      if ( v < -clip_sigmas ) v = -clip_sigmas;
    }

    out[i] = v;
  }
}

Real RunningNormalizer::stdDevAt( int i ) const
{
  const Real denom = ( count > REAL_ONE ) ? ( count - REAL_ONE ) : REAL_ONE;
  Real var = m2[(size_t)i] / denom;

  if ( var < MIN_VARIANCE )
    var = MIN_VARIANCE;

  return std::sqrt( var );
}

u32 RunningNormalizer::sizeOfSerialize() const
{
  return (u32)( U32_SZ + DOUBLE_SZ + 2 * (int)mean.size() * DOUBLE_SZ );
}

void RunningNormalizer::serialize( ByteWriter& sr ) const
{
  sr.writeU32( (u32)mean.size() );
  sr.writeDouble( (double)count );

  for ( size_t i = 0; i < mean.size(); ++i ) sr.writeDouble( (double)mean[i] );
  for ( size_t i = 0; i < m2  .size(); ++i ) sr.writeDouble( (double)m2  [i] );
}

bool RunningNormalizer::deserialize( ByteReader& ds )
{
  if ( ds.readU32() != (u32)mean.size() )
    return false;

  count = (Real)ds.readDouble();

  for ( size_t i = 0; i < mean.size(); ++i ) mean[i] = (Real)ds.readDouble();
  for ( size_t i = 0; i < m2  .size(); ++i ) m2  [i] = (Real)ds.readDouble();

  cache_valid = false;

  return ds.isOk();
}

} // namespace nn
} // namespace zygo
