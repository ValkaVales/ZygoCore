#pragma once

// Proximal Policy Optimization.
//
// Why PPO and not the online actor-critic that trains the cart-pole:
//
// The online A2C does exactly one gradient step per environment step.
// On the cart-pole an environment step costs nothing, so that is a bargain.
// On the robot dog one step of the physics simulator costs four legs' worth of constraint solving,
// and a real hardware step costs 2 ms of wall clock that cannot be bought back.
// 
// The whole game becomes "how much learning can I extract from each collected sample", and the answer is:
// collect a rollout once, then reuse it for several epochs of mini-batch updates.
// 
// Doing that naively destroys the policy, because after the first epoch the data is no longer on-policy.
// PPO's clipped objective is precisely the fix - it refuses to move the policy far from the one that collected the data, so the reuse stays valid.
//
// Works with both head types:
//   DISCRETE   - softmax over actionOutputs() logits
//   CONTINUOUS - gaussian with a learnable per-dimension log-sigma; the sample is squashed into the environment's action limits by clamping,
//                and the LOG-PROB IS COMPUTED BEFORE THE CLAMP.
//                (Computing it after would make the density wrong at the boundaries;
//                tanh squashing with the proper Jacobian correction is the alternative, but for a residual joint-angle policy the action almost never sits at the limit,
//                and clamping keeps the maths honest and simple.)

#include <zygo/ai/nn/actor_critic.h>
#include <zygo/ai/nn/optimizer.h>
#include "environment.h"
#include "rollout_buffer.h"
#include <vector>


namespace zygo {
namespace rl {

enum class PolicyKind
{
  DISCRETE,
  CONTINUOUS
};


struct PpoConfig
{
  int  rollout_steps;   // samples collected before each update
  int  epochs;          // passes over the rollout
  int  minibatch_size;

  Real gamma;
  Real gae_lambda;

  Real clip_ratio;      // 0.2 is the value everyone uses and few beat
  Real entropy_coeff;
  Real value_coeff;
  Real max_grad_norm;

  Real learning_rate;

  // Rescales the advantages of every rollout to zero mean / unit variance.
  // The standard default,/ and the right one when the reward scale is unknown or drifts.
  // 
  // But it is NOT free: once a policy already succeeds on most of a rollout, the true advantages are ~0,
  // and normalization inflates what is left - noise - back to unit size, so the policy keeps taking full-size random steps and never sharpens.
  // 
  // On the cart-pole that alone was a 30x difference in how long the SAMPLING policy took to stop falling.
  // Turn it off when rewards are already well scaled.
  bool normalize_advantage;

  // Early stop: if the policy has already moved this far in KL, stop the remaining epochs.
  // The single most useful safety net when the learning rate is slightly too high. <= 0 disables.
  Real target_kl;

  PpoConfig();
};


struct PpoStats
{
  Real policy_loss;
  Real value_loss;
  Real entropy;
  Real approx_kl;
  Real clip_fraction;
  Real grad_norm;
  Real mean_episode_return;
  Real mean_episode_length;
  int  episodes_finished;
  long long total_env_steps;

  PpoStats();
};


class PpoTrainer
{
private:
  PpoConfig config;
  PolicyKind policy_kind;

  IEnvironment&      env;
  nn::ActorCriticNet& net;
  nn::AdamOptimizer  optimizer;

  RolloutBuffer buffer;

  std::vector<nn::ParamBlock> param_blocks;

  // scratch
  std::vector<Real> obs;
  std::vector<Real> next_obs;
  std::vector<Real> env_action;
  std::vector<Real> probs;
  std::vector<Real> d_policy;
  std::vector<Real> d_log_std;
  std::vector<Real> action;
  std::vector<Real> eps;
  std::vector<int>  order;

  // episode bookkeeping
  Real cur_episode_return;
  int  cur_episode_length;
  int  max_steps_in_episode;

  Real sum_episode_return;
  int  sum_episode_length;
  int  finished_episodes;

  Real last_episode_return;
  int  last_episode_length;
  int  total_episodes;

  long long total_env_steps;
  int       updates_done;

  // Bootstrap value of the state that follows the last stored step. Latched inside stepOnce(),
  // because by the time the update runs the environment may already have been reset.
  Real bootstrap_value;

  // Last acted-upon quantities, kept for display and for real-time control loops.
  Real last_value_pred;
  int  last_action_idx;

  PpoStats last_stats;

public:
  PpoTrainer( IEnvironment& env, nn::ActorCriticNet& net, PpoConfig const& config, PolicyKind kind );

  PpoTrainer( PpoTrainer const& ) = delete;
  PpoTrainer& operator=( PpoTrainer const& ) = delete;

  void setMaxStepsInEpisode( int n ) { max_steps_in_episode = n; }

  nn::AdamOptimizer& opt() { return optimizer; }

  // Collects one rollout and performs one full PPO update on it.
  PpoStats runIteration();

  // Advances the environment by EXACTLY ONE step. When that step fills the rollout, the full PPO update runs and the function returns true.
  //
  // runIteration() is just a loop over this. The step-wise form exists because a real-time loop - a renderer that wants to show every frame,
  // or a hardware control loop that must return within its period - cannot afford to disappear inside a whole rollout.
  bool stepOnce();

  // Ends the current episode from outside and starts a fresh one.
  //
  // The environment cannot simply be reset behind the trainer's back: the trainer caches the current observation,
  // and a reset it does not know about leaves it acting on a state that no longer exists.
  // 
  // This does both halves - cuts the stored trajectory, then resets the env into the trainer's own observation buffer.
  void resetEpisode();

  //
  PpoStats const& lastStats() const { return last_stats; }

  inline long long totalEnvSteps() const { return total_env_steps; }
  inline int       updatesDone  () const { return updates_done; }

  // --- observability, for renderers and logging ---------------------------------------------
  inline Real const* currentObs() const { return obs.data(); }

  inline int rolloutFill    () const { return buffer.size(); }
  inline int rolloutCapacity() const { return config.rollout_steps; }

  inline Real const* lastActionProbs() const { return probs.data(); }  // discrete policies
  inline Real const* lastAction     () const { return action.data(); }
  inline int         lastActionIdx  () const { return last_action_idx; }
  inline Real        lastValue      () const { return last_value_pred; }

  inline int  currentEpisodeLength() const { return cur_episode_length; }
  inline Real currentEpisodeReturn() const { return cur_episode_return; }

  inline int  lastEpisodeLength() const { return last_episode_length; }
  inline Real lastEpisodeReturn() const { return last_episode_return; }
  inline int  totalEpisodes    () const { return total_episodes; }

  // Greedy / mean action, no sampling - for evaluating or for running on the robot.
  void actDeterministic( Real const* observation, Real* out_action );

private:
  void update();

  // Runs the net on one observation and produces the sampled action plus its log-prob.
  Real sampleAction( Real const* observation, Real* out_action, bool update_normalizer );

  // Re-evaluates a STORED action under the current parameters.
  Real evaluateStoredAction( int idx, Real& out_entropy );

  void buildPolicyGradient( int idx, Real effective_advantage );
};

} // namespace rl
} // namespace zygo
