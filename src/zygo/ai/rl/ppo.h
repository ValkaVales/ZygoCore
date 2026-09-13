#pragma once

// Proximal Policy Optimization.
//
// Why PPO and not the online actor-critic that trains the cart-pole:
//
// The online A2C does exactly one gradient step per environment step. On the cart-pole an
// environment step costs nothing, so that is a bargain. On the robot dog one step of the physics
// simulator costs four legs' worth of constraint solving, and a real hardware step costs 2 ms of
// wall clock that cannot be bought back. The whole game becomes "how much learning can I extract
// from each collected sample", and the answer is: collect a rollout once, then reuse it for several
// epochs of mini-batch updates. Doing that naively destroys the policy, because after the first
// epoch the data is no longer on-policy. PPO's clipped objective is precisely the fix - it refuses
// to move the policy far from the one that collected the data, so the reuse stays valid.
//
// Works with both head types:
//   DISCRETE   - softmax over actionOutputs() logits
//   CONTINUOUS - gaussian with a learnable per-dimension log-sigma; the sample is squashed into
//                the environment's action limits by clamping, and the LOG-PROB IS COMPUTED BEFORE
//                THE CLAMP. (Computing it after would make the density wrong at the boundaries;
//                tanh squashing with the proper Jacobian correction is the alternative, but for a
//                residual joint-angle policy the action almost never sits at the limit, and
//                clamping keeps the maths honest and simple.)

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

  long long total_env_steps;

  PpoStats last_stats;

public:
  PpoTrainer( IEnvironment& env, nn::ActorCriticNet& net, PpoConfig const& config, PolicyKind kind );

  PpoTrainer( PpoTrainer const& ) = delete;
  PpoTrainer& operator=( PpoTrainer const& ) = delete;

  void setMaxStepsInEpisode( int n ) { max_steps_in_episode = n; }

  nn::AdamOptimizer& opt() { return optimizer; }

  // Collects one rollout and performs one full PPO update on it.
  PpoStats runIteration();

  PpoStats const& lastStats() const { return last_stats; }

  inline long long totalEnvSteps() const { return total_env_steps; }

  // Greedy / mean action, no sampling - for evaluating or for running on the robot.
  void actDeterministic( Real const* observation, Real* out_action );

private:
  void collectRollout();
  void update();

  // Runs the net on one observation and produces the sampled action plus its log-prob.
  Real sampleAction( Real const* observation, Real* out_action, bool update_normalizer );

  // Re-evaluates a STORED action under the current parameters.
  Real evaluateStoredAction( int idx, Real& out_entropy );

  void buildPolicyGradient( int idx, Real effective_advantage );
};

} // namespace rl
} // namespace zygo
