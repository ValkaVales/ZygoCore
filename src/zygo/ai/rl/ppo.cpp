#include "ppo.h"
#include <zygo/ai/nn/distributions.h>
#include <zygo/core/assert.h>
#include <zygo/random/random.h>
#include <cmath>


namespace zygo {
namespace rl {

using namespace zygo::nn;


PpoConfig::PpoConfig()
  : rollout_steps       ( 2048 )
  , epochs              ( 10 )
  , minibatch_size      ( 64 )
  , gamma               ( (Real)0.99 )
  , gae_lambda          ( (Real)0.95 )
  , clip_ratio          ( (Real)0.2 )
  , entropy_coeff       ( (Real)0.01 )
  , value_coeff         ( (Real)0.5 )
  , max_grad_norm       ( (Real)0.5 )
  , learning_rate       ( (Real)3e-4 )
  , normalize_advantage ( true )
  , target_kl           ( (Real)0.02 )
{
}

PpoStats::PpoStats()
  : policy_loss         ( REAL_ZERO )
  , value_loss          ( REAL_ZERO )
  , entropy             ( REAL_ZERO )
  , approx_kl           ( REAL_ZERO )
  , clip_fraction       ( REAL_ZERO )
  , grad_norm           ( REAL_ZERO )
  , mean_episode_return ( REAL_ZERO )
  , mean_episode_length ( REAL_ZERO )
  , episodes_finished   ( 0 )
  , total_env_steps     ( 0 )
{
}


static int storedActionSize( IEnvironment const& env )
{
  return env.isContinuous() ? env.actionSize() : 1;
}


PpoTrainer::PpoTrainer( IEnvironment& env, ActorCriticNet& net, PpoConfig const& config, PolicyKind kind )
  : config              ( config )
  , policy_kind         ( kind )
  , env                 ( env )
  , net                 ( net )
  , optimizer           ( config.learning_rate )
  , buffer              ( config.rollout_steps, env.obsSize(), storedActionSize( env ) )
  , cur_episode_return  ( REAL_ZERO )
  , cur_episode_length  ( 0 )
  , max_steps_in_episode( 1000 )
  , sum_episode_return  ( REAL_ZERO )
  , sum_episode_length  ( 0 )
  , finished_episodes   ( 0 )
  , last_episode_return ( REAL_ZERO )
  , last_episode_length ( 0 )
  , total_episodes      ( 0 )
  , total_env_steps     ( 0 )
  , updates_done        ( 0 )
  , bootstrap_value     ( REAL_ZERO )
  , last_value_pred     ( REAL_ZERO )
  , last_action_idx     ( 0 )
{
  ZgAssert( net.obsSize() == env.obsSize() );

  if ( kind == PolicyKind::CONTINUOUS )
  {
    ZgAssert( net.actionOutputs() == env.actionSize() );
    ZgAssert( net.hasPolicyLogStd() ); // call enablePolicyLogStd() before constructing the trainer
  }

  net.collectParams( param_blocks );
  optimizer.setParams( param_blocks );

  obs       .assign( (size_t)env.obsSize()        , REAL_ZERO );
  next_obs  .assign( (size_t)env.obsSize(), REAL_ZERO );
  env_action.assign( (size_t)( env.isContinuous() ? env.actionSize() : 1 ), REAL_ZERO );

  probs     .assign( (size_t)net.actionOutputs()  , REAL_ZERO );
  d_policy  .assign( (size_t)net.actionOutputs()  , REAL_ZERO );
  d_log_std .assign( (size_t)net.actionOutputs()  , REAL_ZERO );
  action    .assign( (size_t)net.actionOutputs()  , REAL_ZERO );
  eps       .assign( (size_t)net.actionOutputs()  , REAL_ZERO );

  order.resize( (size_t)config.rollout_steps );
  for ( int i = 0; i < config.rollout_steps; ++i )
    order[(size_t)i] = i;

  env.reset( obs.data() );
}


// ------------------------------------------------------------------------------- acting
Real PpoTrainer::sampleAction( Real const* observation, Real* out_action, bool update_normalizer )
{
  net.forward( observation, update_normalizer );

  const int n = net.actionOutputs();

  if ( policy_kind == PolicyKind::DISCRETE )
  {
    softMax( net.actorOutput(), probs.data(), n );

    const int idx = sampleCategorical( probs.data(), n );

    out_action[0] = (Real)idx;

    Real p = probs[(size_t)idx];
    if ( p < (Real)1e-12 )
      p = (Real)1e-12;

    return std::log( p );
  }

  sampleGaussian( net.actorOutput(), net.policyLogStd(), out_action, eps.data(), n );

  return gaussianLogProb( eps.data(), net.policyLogStd(), n );
}

void PpoTrainer::actDeterministic( Real const* observation, Real* out_action )
{
  net.forward( observation, false, false );

  const int n = net.actionOutputs();

  if ( policy_kind == PolicyKind::DISCRETE )
  {
    softMax( net.actorOutput(), probs.data(), n );

    int best = 0;
    for ( int i = 1; i < n; ++i )
    {
      if ( probs[(size_t)i] > probs[(size_t)best] )
        best = i;
    }

    out_action[0] = (Real)best;
    return;
  }

  // The mean of the gaussian, with no exploration noise at all.
  for ( int i = 0; i < n; ++i )
  {
    Real v = net.actorOutput()[i];
    const Real lim = env.actionLimit( i );

    if ( v >  lim ) v =  lim;
    if ( v < -lim ) v = -lim;

    out_action[i] = v;
  }
}


// ------------------------------------------------------------------------------- collecting
bool PpoTrainer::stepOnce()
{
  const Real log_prob = sampleAction( obs.data(), action.data(), true );
  const Real value    = net.value();

  last_value_pred = value;
  last_action_idx = env.isContinuous() ? 0 : (int)action[0];

  // The stored action is the RAW gaussian sample - the log-prob above belongs to it.
  // What the environment receives is the clamped one.
  if ( env.isContinuous() )
  {
    for ( int i = 0; i < env.actionSize(); ++i )
    {
      Real v = action[(size_t)i];
      const Real lim = env.actionLimit( i );

      if ( v >  lim ) v =  lim;
      if ( v < -lim ) v = -lim;

      env_action[(size_t)i] = v;
    }
  } else
  {
    env_action[0] = action[0];
  }

  StepResult sr = env.step( env_action.data(), next_obs.data() );

  ++cur_episode_length;
  ++total_env_steps;
  cur_episode_return += sr.reward;

  if ( !sr.terminated && cur_episode_length >= max_steps_in_episode )
    sr.truncated = true;

  buffer.add( obs.data(), action.data(), log_prob, value, sr.reward, sr.terminated, sr.truncated );

  for ( int i = 0; i < env.obsSize(); ++i )
    obs[(size_t)i] = next_obs[(size_t)i];

  if ( buffer.isFull() )
  {
    // V(s') for the bootstrap must be taken BEFORE any reset.
    if ( sr.terminated )
      bootstrap_value = REAL_ZERO;
    else
    {
      net.forward( obs.data(), false, false );
      bootstrap_value = net.value();
    }
  }

  if ( sr.isDone() )
  {
    last_episode_return = cur_episode_return;
    last_episode_length = cur_episode_length;

    sum_episode_return += cur_episode_return;
    sum_episode_length += cur_episode_length;
    ++finished_episodes;
    ++total_episodes;

    cur_episode_return = REAL_ZERO;
    cur_episode_length = 0;

    env.reset( obs.data() );
  }

  if ( !buffer.isFull() )
    return false;

  buffer.computeGae( bootstrap_value, config.gamma, config.gae_lambda );

  if ( config.normalize_advantage )
    buffer.normalizeAdvantages();

  update();

  ++updates_done;

  last_stats.episodes_finished = finished_episodes;
  last_stats.total_env_steps   = total_env_steps;

  if ( finished_episodes > 0 )
  {
    last_stats.mean_episode_return = sum_episode_return / (Real)finished_episodes;
    last_stats.mean_episode_length = (Real)sum_episode_length / (Real)finished_episodes;
  }

  sum_episode_return = REAL_ZERO;
  sum_episode_length = 0;
  finished_episodes  = 0;

  buffer.clear();

  return true;
}


// ------------------------------------------------------------------------------- evaluating stored data
Real PpoTrainer::evaluateStoredAction( int idx, Real& out_entropy )
{
  net.forward( buffer.obsAt( idx ), false );

  const int n = net.actionOutputs();

  if ( policy_kind == PolicyKind::DISCRETE )
  {
    softMax( net.actorOutput(), probs.data(), n );

    const int a = (int)buffer.actionAt( idx )[0];

    out_entropy = categoricalEntropy( probs.data(), n );

    Real p = probs[(size_t)a];
    if ( p < (Real)1e-12 )
      p = (Real)1e-12;

    return std::log( p );
  }

  // eps is recomputed under the CURRENT mean and sigma - that is what makes the ratio meaningful.
  Real const* mean    = net.actorOutput();
  Real const* log_std = net.policyLogStd();

  for ( int i = 0; i < n; ++i )
    eps[(size_t)i] = ( buffer.actionAt( idx )[i] - mean[i] ) * std::exp( -log_std[i] );

  out_entropy = gaussianEntropy( log_std, n );

  return gaussianLogProb( eps.data(), log_std, n );
}

void PpoTrainer::buildPolicyGradient( int idx, Real effective_advantage )
{
  const int n = net.actionOutputs();

  if ( policy_kind == PolicyKind::DISCRETE )
  {
    const int a = (int)buffer.actionAt( idx )[0];

    categoricalPolicyGradient( probs.data(), a, effective_advantage, config.entropy_coeff, d_policy.data(), n );
    return;
  }

  gaussianPolicyGradient( eps.data(), net.policyLogStd(), effective_advantage, config.entropy_coeff, d_policy.data(), d_log_std.data(), n );
}


// ------------------------------------------------------------------------------- the update
void PpoTrainer::update()
{
  const int n = buffer.size();

  Real sum_policy_loss = REAL_ZERO;
  Real sum_value_loss  = REAL_ZERO;
  Real sum_entropy     = REAL_ZERO;
  Real sum_kl          = REAL_ZERO;
  Real sum_clipped     = REAL_ZERO;
  Real last_grad_norm  = REAL_ZERO;
  int  samples_seen    = 0;

  bool stop = false;

  for ( int epoch = 0; epoch < config.epochs && !stop; ++epoch )
  {
    // Fisher-Yates
    for ( int i = n - 1; i > 0; --i )
    {
      const int j = (int)Random::randInt( (u32)( i + 1 ) );
      const int t = order[(size_t)i];
      order[(size_t)i] = order[(size_t)j];
      order[(size_t)j] = t;
    }

    Real epoch_kl = REAL_ZERO;
    int  epoch_n  = 0;

    for ( int start = 0; start < n; start += config.minibatch_size )
    {
      const int mb_end = ( start + config.minibatch_size < n ) ? ( start + config.minibatch_size ) : n;
      const int mb_n   = mb_end - start;

      net.zeroGrad();

      for ( int k = start; k < mb_end; ++k )
      {
        const int idx = order[(size_t)k];

        Real entropy = REAL_ZERO;
        const Real log_prob_new = evaluateStoredAction( idx, entropy );
        const Real log_prob_old = buffer.logProbAt( idx );

        const Real log_ratio = log_prob_new - log_prob_old;
        const Real ratio     = std::exp( log_ratio );

        const Real adv = buffer.advantageAt( idx );

        // L_clip = -min( ratio*adv, clip(ratio)*adv )
        // The gradient of the min is the gradient of whichever branch is smaller; when the clipped
        // branch wins, the ratio is outside the trust region and the policy gradient is simply zero.
        const Real lo = REAL_ONE - config.clip_ratio;
        const Real hi = REAL_ONE + config.clip_ratio;

        Real clipped_ratio = ratio;
        if ( clipped_ratio < lo ) clipped_ratio = lo;
        if ( clipped_ratio > hi ) clipped_ratio = hi;

        const Real surr_unclipped = ratio         * adv;
        const Real surr_clipped   = clipped_ratio * adv;

        const bool use_unclipped = ( surr_unclipped <= surr_clipped );

        const Real effective_adv = use_unclipped ? ( ratio * adv ) : REAL_ZERO;

        buildPolicyGradient( idx, effective_adv );

        const Real v_pred = net.value();
        const Real v_targ = buffer.returnAt( idx );
        const Real d_value = config.value_coeff * ( v_pred - v_targ );

        net.backward( d_policy.data(), d_value, ( policy_kind == PolicyKind::CONTINUOUS ) ? d_log_std.data() : nullptr );

        // --- statistics ---
        sum_policy_loss += -( use_unclipped ? surr_unclipped : surr_clipped );
        sum_value_loss  += REAL_HALF * ( v_pred - v_targ ) * ( v_pred - v_targ );
        sum_entropy     += entropy;

        // k3 estimator: always non-negative, much lower variance than -log_ratio
        const Real kl = ratio - REAL_ONE - log_ratio;
        sum_kl   += kl;
        epoch_kl += kl;
        ++epoch_n;

        if ( !use_unclipped )
          sum_clipped += REAL_ONE;

        ++samples_seen;
      }

      net.scaleGradients( REAL_ONE / (Real)mb_n );

      last_grad_norm = clipGlobalGradNorm( param_blocks, config.max_grad_norm );

      optimizer.step();
    }

    if ( config.target_kl > REAL_ZERO && epoch_n > 0 )
    {
      if ( epoch_kl / (Real)epoch_n > config.target_kl )
        stop = true; // the policy has moved far enough for one rollout
    }
  }

  const Real inv = ( samples_seen > 0 ) ? ( REAL_ONE / (Real)samples_seen ) : REAL_ZERO;

  last_stats.policy_loss   = sum_policy_loss * inv;
  last_stats.value_loss    = sum_value_loss  * inv;
  last_stats.entropy       = sum_entropy     * inv;
  last_stats.approx_kl     = sum_kl          * inv;
  last_stats.clip_fraction = sum_clipped     * inv;
  last_stats.grad_norm     = last_grad_norm;
}


PpoStats PpoTrainer::runIteration()
{
  while ( !stepOnce() )
    ;

  return last_stats;
}

} // namespace rl
} // namespace zygo
