#include "parallel_ppo.h"
#include <zygo/ai/nn/distributions.h>
#include <zygo/core/assert.h>
#include <zygo/random/random.h>
#include <cmath>


namespace zygo {
namespace rl {

using namespace zygo::nn;


// ------------------------------------------------------------------------------------------------ construction
ParallelPpoTrainer::ParallelPpoTrainer( std::vector<IEnvironment*> const& envs, ActorCriticNet& net, PpoConfig const& config, PolicyKind kind, u64 seed )
  : config              ( config )
  , policy_kind         ( kind )
  , net                 ( net )
  , optimizer           ( config.learning_rate )
  , steps_per_env       ( 0 )
  , max_steps_in_episode( 1000 )
  , adv_mean            ( REAL_ZERO )
  , adv_inv_std         ( REAL_ONE )
  , shuffle_state       ( seed * 0x9E3779B97F4A7C15ULL + 0x2545F4914F6CDD1DULL )
  , total_env_steps     ( 0 )
  , updates_done        ( 0 )
{
  ZgAssertRelease( !envs.empty() );

  const int n_envs = (int)envs.size();

  steps_per_env = ( config.rollout_steps + n_envs - 1 ) / n_envs;
  if ( steps_per_env < 1 )
    steps_per_env = 1;

  IEnvironment const& e0 = *envs[0];
  const int stored_action_size = e0.isContinuous() ? e0.actionSize() : 1;

  ZgAssert( net.obsSize() == e0.obsSize() );

  if ( kind == PolicyKind::CONTINUOUS )
  {
    ZgAssert( net.actionOutputs() == e0.actionSize() );
    ZgAssert( net.hasPolicyLogStd() ); // call enablePolicyLogStd() before constructing the trainer
  }

  net.collectParams( param_blocks );
  optimizer.setParams( param_blocks );

  for ( int i = 0; i < n_envs; ++i )
  {
    ZgAssert( envs[(size_t)i]->obsSize() == e0.obsSize() && envs[(size_t)i]->actionSize() == e0.actionSize() );

    std::unique_ptr<Worker> w( new Worker( steps_per_env, e0.obsSize(), stored_action_size ) );

    w->index = i;
    w->env   = envs[(size_t)i];

    w->net.reset( new ActorCriticNet( net.cfg() ) );
    if ( net.hasPolicyLogStd() )
      w->net->enablePolicyLogStd( REAL_ZERO ); // the value is overwritten by the first sync

    w->net->collectParams( w->params );

    w->obs       .assign( (size_t)e0.obsSize(), REAL_ZERO );
    w->next_obs  .assign( (size_t)e0.obsSize(), REAL_ZERO );
    w->env_action.assign( (size_t)stored_action_size, REAL_ZERO );
    w->action    .assign( (size_t)net.actionOutputs(), REAL_ZERO );
    w->eps       .assign( (size_t)net.actionOutputs(), REAL_ZERO );
    w->probs     .assign( (size_t)net.actionOutputs(), REAL_ZERO );
    w->d_policy  .assign( (size_t)net.actionOutputs(), REAL_ZERO );
    w->d_log_std .assign( (size_t)net.actionOutputs(), REAL_ZERO );

    ZgAssert( w->params.size() == param_blocks.size() );

    workers.push_back( std::move( w ) );
  }

  order.resize( (size_t)( steps_per_env * n_envs ) );

  for ( int i = 0; i < n_envs; ++i )
    threads.emplace_back( &ParallelPpoTrainer::threadMain, this, i, seed );

  // The first reset already happens on the worker's own thread, from its own random stream.
  runOnAllWorkers( []( Worker& w ) { w.env->reset( w.obs.data() ); } );
}

ParallelPpoTrainer::~ParallelPpoTrainer()
{
  {
    std::lock_guard<std::mutex> lock( pool_mutex );
    pool_quit = true;
  }

  pool_start.notify_all();

  for ( auto& t : threads )
    t.join();
}


// ------------------------------------------------------------------------------------------------ thread pool
void ParallelPpoTrainer::threadMain( int index, u64 seed )
{
  Random::seed( seed * 0xD1B54A32D192ED03ULL + (u64)index * 0x9E3779B97F4A7C15ULL + 1 );

  Worker& w = *workers[(size_t)index];
  u64 seen_job = 0;

  for ( ;; )
  {
    std::function<void( Worker& )> const* job = nullptr;

    {
      std::unique_lock<std::mutex> lock( pool_mutex );
      pool_start.wait( lock, [&] { return pool_quit || pool_job_id != seen_job; } );

      if ( pool_quit )
        return;

      seen_job = pool_job_id;
      job = pool_job;
    }

    ( *job )( w );

    {
      std::lock_guard<std::mutex> lock( pool_mutex );
      if ( --pool_pending == 0 )
        pool_done.notify_all();
    }
  }
}

void ParallelPpoTrainer::runOnAllWorkers( std::function<void( Worker& )> const& job )
{
  {
    std::lock_guard<std::mutex> lock( pool_mutex );
    pool_job     = &job;
    pool_pending = (int)workers.size();
    ++pool_job_id;
  }

  pool_start.notify_all();

  std::unique_lock<std::mutex> lock( pool_mutex );
  pool_done.wait( lock, [&] { return pool_pending == 0; } );
  pool_job = nullptr;
}


// ------------------------------------------------------------------------------------------------ collecting
void ParallelPpoTrainer::syncWorkerNet( Worker& w )
{
  // Runs on the worker's thread while the owner waits, so the master cannot change underneath.
  w.net->copyParamsFrom( net );
  w.net->observationNormalizer() = net.observationNormalizer();
}

void ParallelPpoTrainer::collect( Worker& w, int steps )
{
  ActorCriticNet& wn = *w.net;
  IEnvironment&   env = *w.env;

  const int n = wn.actionOutputs();

  for ( int t = 0; t < steps && !w.buffer.isFull(); ++t )
  {
    // update_normalizer = false: the statistics are frozen for the whole rollout (see the header)
    wn.forward( w.obs.data(), false, false );

    Real log_prob;

    if ( policy_kind == PolicyKind::DISCRETE )
    {
      softMax( wn.actorOutput(), w.probs.data(), n );

      const int idx = sampleCategorical( w.probs.data(), n );
      w.action[0] = (Real)idx;

      Real p = w.probs[(size_t)idx];
      if ( p < (Real)1e-12 )
        p = (Real)1e-12;

      log_prob = std::log( p );
      w.env_action[0] = w.action[0];
    }
    else
    {
      sampleGaussian( wn.actorOutput(), wn.policyLogStd(), w.action.data(), w.eps.data(), n );
      log_prob = gaussianLogProb( w.eps.data(), wn.policyLogStd(), n );

      // The stored action is the RAW sample - the log-prob belongs to it. The environment gets the clamped one.
      for ( int i = 0; i < n; ++i )
      {
        Real v = w.action[(size_t)i];
        const Real lim = env.actionLimit( i );

        if ( v >  lim ) v =  lim;
        if ( v < -lim ) v = -lim;

        w.env_action[(size_t)i] = v;
      }
    }

    const Real value = wn.value();

    StepResult sr = env.step( w.env_action.data(), w.next_obs.data() );

    ++w.cur_episode_length;
    w.cur_episode_return += sr.reward;

    if ( !sr.terminated && w.cur_episode_length >= max_steps_in_episode )
      sr.truncated = true;

    w.buffer.add( w.obs.data(), w.action.data(), log_prob, value, sr.reward, sr.terminated, sr.truncated );

    // A time limit is not a failure: the state the robot was cut in still has a future.
    if ( sr.truncated && !sr.terminated )
    {
      wn.forward( w.next_obs.data(), false, false );
      w.buffer.bootstrapLastStep( wn.value(), config.gamma );
    }

    w.obs.swap( w.next_obs );

    if ( w.buffer.isFull() )
    {
      // V(s') for the end of the rollout must be taken BEFORE any reset.
      if ( w.buffer.isTerminated( w.buffer.size() - 1 ) )
        w.bootstrap_value = REAL_ZERO;
      else
      {
        wn.forward( w.obs.data(), false, false );
        w.bootstrap_value = wn.value();
      }
    }

    if ( sr.isDone() )
    {
      w.finished_returns.push_back( w.cur_episode_return );

      w.rollout_return_sum += w.cur_episode_return;
      w.rollout_length_sum += w.cur_episode_length;
      ++w.rollout_episodes;

      w.cur_episode_return = REAL_ZERO;
      w.cur_episode_length = 0;

      env.reset( w.obs.data() );
    }
  }
}

void ParallelPpoTrainer::cutEpisode( Worker& w )
{
  syncWorkerNet( w );

  if ( w.buffer.size() > 0 )
  {
    // The last stored step led to w.obs - the state the episode is being cut in.
    w.net->forward( w.obs.data(), false, false );
    w.buffer.bootstrapLastStep( w.net->value(), config.gamma );
  }

  w.cur_episode_return = REAL_ZERO;
  w.cur_episode_length = 0;

  w.env->reset( w.obs.data() );
}

void ParallelPpoTrainer::resetEpisode( int env_index )
{
  runOnAllWorkers( [&]( Worker& w )
  {
    if ( w.index == env_index )
      cutEpisode( w );
  } );
}

void ParallelPpoTrainer::resetAllEpisodes()
{
  runOnAllWorkers( [&]( Worker& w ) { cutEpisode( w ); } );
}

bool ParallelPpoTrainer::advance( int max_steps )
{
  const int remaining = steps_per_env - workers[0]->buffer.size();
  const int steps = ( max_steps < remaining ) ? max_steps : remaining;

  if ( steps > 0 )
  {
    runOnAllWorkers( [&]( Worker& w )
    {
      syncWorkerNet( w );
      collect( w, steps );
    } );

    total_env_steps += (long long)steps * (long long)workers.size();
  }

  if ( !workers[0]->buffer.isFull() )
    return false;

  update();
  ++updates_done;

  double ret_sum = 0.0;
  long   len_sum = 0;
  int    episodes = 0;

  for ( auto& w : workers )
  {
    ret_sum  += w->rollout_return_sum;
    len_sum  += w->rollout_length_sum;
    episodes += w->rollout_episodes;

    w->rollout_return_sum = 0.0;
    w->rollout_length_sum = 0;
    w->rollout_episodes   = 0;

    w->buffer.clear();
  }

  last_stats.episodes_finished = episodes;
  last_stats.total_env_steps   = total_env_steps;

  if ( episodes > 0 )
  {
    last_stats.mean_episode_return = (Real)( ret_sum / episodes );
    last_stats.mean_episode_length = (Real)len_sum / (Real)episodes;
  }

  return true;
}

PpoStats ParallelPpoTrainer::runIteration()
{
  while ( !advance( steps_per_env ) )
    ;

  return last_stats;
}

int ParallelPpoTrainer::rolloutFill() const
{
  int total = 0;
  for ( auto const& w : workers )
    total += w->buffer.size();
  return total;
}

void ParallelPpoTrainer::takeFinishedEpisodes( std::vector<Real>& out_returns )
{
  for ( auto& w : workers )
  {
    out_returns.insert( out_returns.end(), w->finished_returns.begin(), w->finished_returns.end() );
    w->finished_returns.clear();
  }
}

void ParallelPpoTrainer::actDeterministic( Real const* observation, Real* out_action )
{
  net.forward( observation, false, false );

  const int n = net.actionOutputs();

  if ( policy_kind == PolicyKind::DISCRETE )
  {
    int best = 0;
    for ( int i = 1; i < n; ++i )
      if ( net.actorOutput()[i] > net.actorOutput()[best] )
        best = i;

    out_action[0] = (Real)best;
    return;
  }

  IEnvironment const& env = *workers[0]->env;

  for ( int i = 0; i < n; ++i )
  {
    Real v = net.actorOutput()[i];
    const Real lim = env.actionLimit( i );

    if ( v >  lim ) v =  lim;
    if ( v < -lim ) v = -lim;

    out_action[i] = v;
  }
}


// ------------------------------------------------------------------------------------------------ the update
u32 ParallelPpoTrainer::shuffleRand( u32 to )
{
  // xorshift64*, private to the trainer: the shuffle must not depend on who else on this thread uses Random
  shuffle_state ^= shuffle_state >> 12;
  shuffle_state ^= shuffle_state << 25;
  shuffle_state ^= shuffle_state >> 27;

  const u64 r = shuffle_state * 0x2545F4914F6CDD1DULL;
  return (u32)( ( (u64)(u32)( r >> 32 ) * to ) >> 32 );
}

void ParallelPpoTrainer::gradientSlice( Worker& w, int const* indices, int count )
{
  ActorCriticNet& wn = *w.net;
  const int n = wn.actionOutputs();

  wn.zeroGrad();

  w.policy_loss = w.value_loss = w.entropy = w.kl = w.clipped = 0.0;
  w.samples = 0;

  for ( int k = 0; k < count; ++k )
  {
    const int g = indices[k];
    RolloutBuffer const& buf = workers[(size_t)( g / steps_per_env )]->buffer;
    const int idx = g % steps_per_env;

    wn.forward( buf.obsAt( idx ), false, true );

    // ---- log-prob of the stored action under the CURRENT parameters (PpoTrainer::evaluateStoredAction)
    Real entropy;
    Real log_prob_new;

    if ( policy_kind == PolicyKind::DISCRETE )
    {
      softMax( wn.actorOutput(), w.probs.data(), n );

      const int a = (int)buf.actionAt( idx )[0];
      entropy = categoricalEntropy( w.probs.data(), n );

      Real p = w.probs[(size_t)a];
      if ( p < (Real)1e-12 )
        p = (Real)1e-12;

      log_prob_new = std::log( p );
    }
    else
    {
      Real const* mean    = wn.actorOutput();
      Real const* log_std = wn.policyLogStd();

      for ( int i = 0; i < n; ++i )
        w.eps[(size_t)i] = ( buf.actionAt( idx )[i] - mean[i] ) * std::exp( -log_std[i] );

      entropy      = gaussianEntropy( log_std, n );
      log_prob_new = gaussianLogProb( w.eps.data(), log_std, n );
    }

    const Real log_ratio = log_prob_new - buf.logProbAt( idx );
    const Real ratio     = std::exp( log_ratio );

    const Real adv = ( buf.advantageAt( idx ) - adv_mean ) * adv_inv_std;

    const Real lo = REAL_ONE - config.clip_ratio;
    const Real hi = REAL_ONE + config.clip_ratio;

    Real clipped_ratio = ratio;
    if ( clipped_ratio < lo ) clipped_ratio = lo;
    if ( clipped_ratio > hi ) clipped_ratio = hi;

    const Real surr_unclipped = ratio         * adv;
    const Real surr_clipped   = clipped_ratio * adv;

    const bool use_unclipped = ( surr_unclipped <= surr_clipped );
    const Real effective_adv = use_unclipped ? ( ratio * adv ) : REAL_ZERO;

    if ( policy_kind == PolicyKind::DISCRETE )
      categoricalPolicyGradient( w.probs.data(), (int)buf.actionAt( idx )[0], effective_adv, config.entropy_coeff, w.d_policy.data(), n );
    else
      gaussianPolicyGradient( w.eps.data(), wn.policyLogStd(), effective_adv, config.entropy_coeff, w.d_policy.data(), w.d_log_std.data(), n );

    const Real v_pred  = wn.value();
    const Real v_targ  = buf.returnAt( idx );
    const Real d_value = config.value_coeff * ( v_pred - v_targ );

    wn.backward( w.d_policy.data(), d_value, ( policy_kind == PolicyKind::CONTINUOUS ) ? w.d_log_std.data() : nullptr );

    w.policy_loss += -( use_unclipped ? surr_unclipped : surr_clipped );
    w.value_loss  += REAL_HALF * ( v_pred - v_targ ) * ( v_pred - v_targ );
    w.entropy     += entropy;
    w.kl          += ratio - REAL_ONE - log_ratio; // k3 estimator
    if ( !use_unclipped )
      w.clipped += 1.0;
    ++w.samples;
  }
}

void ParallelPpoTrainer::update()
{
  const int n_workers = (int)workers.size();
  const int n = steps_per_env * n_workers;

  // ---- GAE, each environment's stream on its own
  runOnAllWorkers( [&]( Worker& w ) { w.buffer.computeGae( w.bootstrap_value, config.gamma, config.gae_lambda ); } );

  // ---- advantage normalization over everything collected
  adv_mean    = REAL_ZERO;
  adv_inv_std = REAL_ONE;

  if ( config.normalize_advantage )
  {
    double sum = 0.0, sum2 = 0.0;

    for ( auto const& w : workers )
    {
      for ( int i = 0; i < steps_per_env; ++i )
      {
        const double a = w->buffer.advantageAt( i );
        sum  += a;
        sum2 += a * a;
      }
    }

    const double mean = sum / n;
    const double var  = sum2 / n - mean * mean;

    adv_mean    = (Real)mean;
    adv_inv_std = (Real)( 1.0 / ( std::sqrt( var > 0.0 ? var : 0.0 ) + 1e-8 ) );
  }

  for ( int i = 0; i < n; ++i )
    order[(size_t)i] = i;

  double sum_policy_loss = 0.0, sum_value_loss = 0.0, sum_entropy = 0.0, sum_kl = 0.0, sum_clipped = 0.0;
  long   samples_seen = 0;
  Real   last_grad_norm = REAL_ZERO;

  bool stop = false;

  for ( int epoch = 0; epoch < config.epochs && !stop; ++epoch )
  {
    for ( int i = n - 1; i > 0; --i )
    {
      const int j = (int)shuffleRand( (u32)( i + 1 ) );
      const int t = order[(size_t)i];
      order[(size_t)i] = order[(size_t)j];
      order[(size_t)j] = t;
    }

    double epoch_kl = 0.0;
    long   epoch_n  = 0;

    for ( int start = 0; start < n; start += config.minibatch_size )
    {
      const int mb_end = ( start + config.minibatch_size < n ) ? ( start + config.minibatch_size ) : n;
      const int mb_n   = mb_end - start;

      int const* mb = order.data() + start;

      // Every worker takes its slice of the minibatch, on the weights of the previous Adam step.
      runOnAllWorkers( [&]( Worker& w )
      {
        const int b = mb_n * w.index / n_workers;
        const int e = mb_n * ( w.index + 1 ) / n_workers;

        w.net->copyParamsFrom( net );
        gradientSlice( w, mb + b, e - b );
      } );

      // Sum of the slices = the gradient of the whole minibatch.
      net.zeroGrad();

      for ( auto const& w : workers )
      {
        for ( size_t bl = 0; bl < param_blocks.size(); ++bl )
        {
          Real      * dst = param_blocks[bl].grads;
          Real const* src = w->params[bl].grads;

          for ( int k = 0; k < param_blocks[bl].count; ++k )
            dst[k] += src[k];
        }

        sum_policy_loss += w->policy_loss;
        sum_value_loss  += w->value_loss;
        sum_entropy     += w->entropy;
        sum_kl          += w->kl;
        sum_clipped     += w->clipped;
        samples_seen    += w->samples;

        epoch_kl += w->kl;
        epoch_n  += w->samples;
      }

      net.scaleGradients( REAL_ONE / (Real)mb_n );

      last_grad_norm = clipGlobalGradNorm( param_blocks, config.max_grad_norm );

      optimizer.step();
    }

    if ( config.target_kl > REAL_ZERO && epoch_n > 0 && epoch_kl / (double)epoch_n > config.target_kl )
      stop = true; // the policy has moved far enough for one rollout
  }

  const double inv = ( samples_seen > 0 ) ? 1.0 / (double)samples_seen : 0.0;

  last_stats.policy_loss   = (Real)( sum_policy_loss * inv );
  last_stats.value_loss    = (Real)( sum_value_loss  * inv );
  last_stats.entropy       = (Real)( sum_entropy     * inv );
  last_stats.approx_kl     = (Real)( sum_kl          * inv );
  last_stats.clip_fraction = (Real)( sum_clipped     * inv );
  last_stats.grad_norm     = last_grad_norm;

  // ---- the normalizer learns from this rollout only now, AFTER the update (see the header)
  RunningNormalizer& normalizer = net.observationNormalizer();

  if ( net.cfg().normalize_observations )
    for ( auto const& w : workers )
      normalizer.update( w->buffer.obsAt( 0 ), w->buffer.size() );
}

} // namespace rl
} // namespace zygo
