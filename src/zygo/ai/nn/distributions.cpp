#include "distributions.h"
#include <zygo/core/assert.h>
#include <zygo/random/random.h>
#include <zygo/math/common/consts.h>
#include <cmath>


namespace zygo {
namespace nn {

static const Real LOG_2PI = (Real)1.8378770664093454836; // log(2*pi)
static const Real PROB_EPS = (Real)1e-12;


// -------------------------------------------------------------------------------- categorical
void softMax( Real const* logits, Real* probs, int n )
{
  ZgAssert( logits != nullptr );
  ZgAssert( n > 0 );

  Real max_v = logits[0];

  for ( int i = 1; i < n; ++i )
  {
    if ( logits[i] > max_v )
      max_v = logits[i];
  }

  Real sum = REAL_ZERO;

  for ( int i = 0; i < n; ++i )
  {
    Real e = std::exp( logits[i] - max_v );
    probs[i] = e;
    sum += e;
  }

  // sum >= 1 by construction: the largest element contributes exp(0) == 1
  const Real inv_sum = REAL_ONE / sum;

  for ( int i = 0; i < n; ++i )
    probs[i] *= inv_sum;
}

int sampleCategorical( Real const* probs, int n )
{
  ZgAssert( probs != nullptr );
  ZgAssert( n > 0 );

  Real r = Random::rand01();

  for ( int i = 0; i < n - 1; ++i )
  {
    r -= probs[i];

    if ( r < REAL_ZERO )
      return i;
  }

  return n - 1; // also the safety net against accumulated rounding
}

Real categoricalEntropy( Real const* probs, int n )
{
  Real h = REAL_ZERO;

  for ( int i = 0; i < n; ++i )
  {
    const Real p = probs[i];

    if ( p > PROB_EPS )
      h -= p * std::log( p );
  }

  return h;
}

void categoricalPolicyGradient(
    Real const* probs
  , int         action_idx
  , Real        advantage
  , Real        entropy_coeff
  , Real*       out_dlogits
  , int         n
  )
{
  ZgAssert( action_idx >= 0 && action_idx < n );

  for ( int j = 0; j < n; ++j )
    out_dlogits[j] = probs[j] * advantage;

  out_dlogits[action_idx] -= advantage;

  if ( entropy_coeff <= REAL_ZERO )
    return;

  const Real h = categoricalEntropy( probs, n );

  for ( int j = 0; j < n; ++j )
  {
    const Real p = probs[j];

    if ( p > PROB_EPS )
      out_dlogits[j] += entropy_coeff * p * ( std::log( p ) + h );
  }
}


// -------------------------------------------------------------------------------- diagonal gaussian
void sampleGaussian( Real const* mean, Real const* log_std, Real* out_action, Real* out_eps, int n )
{
  for ( int i = 0; i < n; ++i )
  {
    const Real e = Random::gauss( REAL_ZERO, REAL_ONE );

    out_eps   [i] = e;
    out_action[i] = mean[i] + std::exp( log_std[i] ) * e;
  }
}

Real gaussianLogProb( Real const* eps, Real const* log_std, int n )
{
  Real lp = REAL_ZERO;

  for ( int i = 0; i < n; ++i )
    lp -= REAL_HALF * eps[i] * eps[i] + log_std[i] + REAL_HALF * LOG_2PI;

  return lp;
}

Real gaussianEntropy( Real const* log_std, int n )
{
  // H = sum_i ( log sigma_i + 0.5 * log(2*pi*e) )
  Real h = REAL_ZERO;

  for ( int i = 0; i < n; ++i )
    h += log_std[i] + REAL_HALF * ( LOG_2PI + REAL_ONE );

  return h;
}

void gaussianPolicyGradient(
    Real const* eps
  , Real const* log_std
  , Real        advantage
  , Real        entropy_coeff
  , Real*       out_dmean
  , Real*       out_dlog_std
  , int         n
  )
{
  for ( int i = 0; i < n; ++i )
  {
    const Real inv_sigma = std::exp( -log_std[i] );

    // d( -logpi * A ) / d mean = -A * eps / sigma
    out_dmean[i] = -advantage * eps[i] * inv_sigma;

    if ( out_dlog_std )
    {
      // d( -logpi * A ) / d log_sigma = -A * (eps^2 - 1);  entropy adds -coeff (dH/dlog_sigma = 1)
      out_dlog_std[i] = -advantage * ( eps[i] * eps[i] - REAL_ONE ) - entropy_coeff;
    }
  }
}

} // namespace nn
} // namespace zygo
