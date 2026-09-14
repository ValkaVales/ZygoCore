#include "rollout_buffer.h"
#include <zygo/core/assert.h>
#include <cmath>


namespace zygo {
namespace rl {

RolloutBuffer::RolloutBuffer( int capacity, int obs_size, int action_size )
  : capacity    ( capacity )
  , obs_size    ( obs_size )
  , action_size ( action_size )
  , count       ( 0 )
{
  ZgAssert( capacity    > 0 );
  ZgAssert( obs_size    > 0 );
  ZgAssert( action_size > 0 );

  obs        .assign( (size_t)capacity * (size_t)obs_size   , REAL_ZERO );
  actions    .assign( (size_t)capacity * (size_t)action_size, REAL_ZERO );
  log_probs  .assign( (size_t)capacity, REAL_ZERO );
  values     .assign( (size_t)capacity, REAL_ZERO );
  rewards    .assign( (size_t)capacity, REAL_ZERO );
  advantages .assign( (size_t)capacity, REAL_ZERO );
  returns    .assign( (size_t)capacity, REAL_ZERO );
  terminated .assign( (size_t)capacity, 0 );
  truncated  .assign( (size_t)capacity, 0 );
}

void RolloutBuffer::add(
    Real const* obs_in
  , Real const* action_in
  , Real        log_prob
  , Real        value
  , Real        reward
  , bool        terminated_in
  , bool        truncated_in
  )
{
  ZgAssert( count < capacity );

  Real* p_obs = &obs[(size_t)count * (size_t)obs_size];
  for ( int i = 0; i < obs_size; ++i )
    p_obs[i] = obs_in[i];

  Real* p_act = &actions[(size_t)count * (size_t)action_size];
  for ( int i = 0; i < action_size; ++i )
    p_act[i] = action_in[i];

  log_probs [(size_t)count] = log_prob;
  values    [(size_t)count] = value;
  rewards   [(size_t)count] = reward;
  terminated[(size_t)count] = terminated_in ? 1 : 0;
  truncated [(size_t)count] = truncated_in  ? 1 : 0;

  ++count;
}

void RolloutBuffer::computeGae( Real last_value, Real gamma, Real gae_lambda )
{
  Real gae = REAL_ZERO;

  for ( int i = count - 1; i >= 0; --i )
  {
    // Value of the state that FOLLOWS step i.
    Real v_next;

    if ( i == count - 1 )
      v_next = isTerminated( i ) ? REAL_ZERO : last_value;
    else
      v_next = isTerminated( i ) ? REAL_ZERO : values[(size_t)i + 1];

    // The GAE recursion must not run across an episode boundary of ANY kind.
    const Real cont = ( isTerminated( i ) || isTruncated( i ) ) ? REAL_ZERO : REAL_ONE;

    const Real delta = rewards[(size_t)i] + gamma * v_next - values[(size_t)i];

    gae = delta + gamma * gae_lambda * cont * gae;

    advantages[(size_t)i] = gae;
    returns   [(size_t)i] = gae + values[(size_t)i];
  }
}

void RolloutBuffer::markLastTruncated()
{
  if ( count > 0 )
    truncated[(size_t)count - 1] = 1;
}

void RolloutBuffer::normalizeAdvantages()
{
  if ( count < 2 )
    return;

  Real mean = REAL_ZERO;
  for ( int i = 0; i < count; ++i )
    mean += advantages[(size_t)i];
  mean /= (Real)count;

  Real var = REAL_ZERO;
  for ( int i = 0; i < count; ++i )
  {
    const Real d = advantages[(size_t)i] - mean;
    var += d * d;
  }
  var /= (Real)count;

  const Real inv_std = REAL_ONE / ( std::sqrt( var ) + (Real)1e-8 );

  for ( int i = 0; i < count; ++i )
    advantages[(size_t)i] = ( advantages[(size_t)i] - mean ) * inv_std;
}

Real RolloutBuffer::meanReward() const
{
  if ( count <= 0 )
    return REAL_ZERO;

  Real sum = REAL_ZERO;
  for ( int i = 0; i < count; ++i )
    sum += rewards[(size_t)i];

  return sum / (Real)count;
}

} // namespace rl
} // namespace zygo
