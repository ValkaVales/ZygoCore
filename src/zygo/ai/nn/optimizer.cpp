#include "optimizer.h"
#include <zygo/core/assert.h>
#include <cmath>


namespace zygo {
namespace nn {

Real calcGlobalGradNorm( std::vector<ParamBlock> const& blocks )
{
  Real sum = REAL_ZERO;

  for ( size_t k = 0; k < blocks.size(); ++k )
  {
    Real const* g = blocks[k].grads;
    int n = blocks[k].count;

    while ( n-- )
    {
      sum += (*g) * (*g);
      ++g;
    }
  }

  return std::sqrt( sum );
}

Real clipGlobalGradNorm( std::vector<ParamBlock> const& blocks, Real max_norm )
{
  const Real norm = calcGlobalGradNorm( blocks );

  if ( max_norm <= REAL_ZERO || norm <= max_norm || norm <= REAL_ZERO )
    return norm;

  const Real k = max_norm / norm;

  for ( size_t idx = 0; idx < blocks.size(); ++idx )
  {
    Real* g = blocks[idx].grads;
    int n = blocks[idx].count;

    while ( n-- )
      *g++ *= k;
  }

  return norm;
}

void zeroGrads( std::vector<ParamBlock> const& blocks )
{
  for ( size_t k = 0; k < blocks.size(); ++k )
  {
    Real* g = blocks[k].grads;
    int n = blocks[k].count;

    while ( n-- )
      *g++ = REAL_ZERO;
  }
}

int IOptimizer::paramsCount() const
{
  int total = 0;

  for ( size_t k = 0; k < params.size(); ++k )
    total += params[k].count;

  return total;
}


// ------------------------------------------------------------------------------------ SgdOptimizer
SgdOptimizer::SgdOptimizer( Real learning_rate, Real momentum, Real weight_decay )
  : IOptimizer    ( learning_rate )
  , momentum      ( momentum )
  , weight_decay  ( weight_decay )
{
}

void SgdOptimizer::reset()
{
  velocity.assign( (size_t)paramsCount(), REAL_ZERO );
}

void SgdOptimizer::step()
{
  ZgAssert( velocity.size() == (size_t)paramsCount() );

  size_t offset = 0;

  for ( size_t k = 0; k < params.size(); ++k )
  {
    ParamBlock const& pb = params[k];

    const Real wd = pb.is_bias ? REAL_ZERO : weight_decay;

    for ( int i = 0; i < pb.count; ++i )
    {
      Real g = pb.grads[i] + wd * pb.values[i];

      if ( momentum > REAL_ZERO )
      {
        Real& vel = velocity[offset + (size_t)i];
        vel = momentum * vel + g;
        g = vel;
      }

      pb.values[i] -= learning_rate * g;
    }

    offset += (size_t)pb.count;
  }
}


// ------------------------------------------------------------------------------------ AdamOptimizer
AdamOptimizer::AdamOptimizer( Real learning_rate, Real beta1, Real beta2, Real eps, Real weight_decay )
  : IOptimizer    ( learning_rate )
  , beta1         ( beta1 )
  , beta2         ( beta2 )
  , eps           ( eps )
  , weight_decay  ( weight_decay )
  , beta1_pow     ( REAL_ONE )
  , beta2_pow     ( REAL_ONE )
{
}

void AdamOptimizer::reset()
{
  const size_t n = (size_t)paramsCount();

  m.assign( n, REAL_ZERO );
  v.assign( n, REAL_ZERO );

  beta1_pow = REAL_ONE;
  beta2_pow = REAL_ONE;
}

int AdamOptimizer::stepsDone() const
{
  // t reconstructed from beta1^t; only used for logging
  if ( beta1_pow >= REAL_ONE || beta1 <= REAL_ZERO )
    return 0;

  return (int)( std::log( beta1_pow ) / std::log( beta1 ) + REAL_HALF );
}

void AdamOptimizer::step()
{
  ZgAssert( m.size() == (size_t)paramsCount() );

  beta1_pow *= beta1;
  beta2_pow *= beta2;

  // Bias correction folded into the step size, exactly as in the reference implementation:
  //   step = lr * sqrt(1 - beta2^t) / (1 - beta1^t)
  const Real bc1 = REAL_ONE - beta1_pow;
  const Real bc2 = REAL_ONE - beta2_pow;
  const Real step_size = learning_rate * std::sqrt( bc2 ) / bc1;

  size_t offset = 0;

  for ( size_t k = 0; k < params.size(); ++k )
  {
    ParamBlock const& pb = params[k];

    for ( int i = 0; i < pb.count; ++i )
    {
      const Real g = pb.grads[i];

      Real& mi = m[offset + (size_t)i];
      Real& vi = v[offset + (size_t)i];

      mi += (REAL_ONE - beta1) * (g      - mi);
      vi += (REAL_ONE - beta2) * (g * g  - vi);

      pb.values[i] -= step_size * mi / (std::sqrt( vi ) + eps);

      // Decoupled weight decay: applied to the parameter, NOT mixed into the gradient,
      // so the adaptive denominator does not distort it.
      if ( !pb.is_bias && weight_decay > REAL_ZERO )
        pb.values[i] -= learning_rate * weight_decay * pb.values[i];
    }

    offset += (size_t)pb.count;
  }
}

} // namespace nn
} // namespace zygo
