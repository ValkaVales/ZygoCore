#pragma once

// Actor-critic network: a shared trunk feeding a policy head and a value head.
//
// This is the shape used by every on-policy algorithm worth running (A2C, PPO).
// It is explicit and works for any depth.
//
// Both discrete and continuous policies are supported:
//
//   discrete   - actionOutputs() logits; use softMax() + categoricalPolicyGradient()
//   continuous - actionOutputs() means plus a learnable per-dimension log-sigma vector (enablePolicyLogStd()); use sampleGaussian() + gaussianPolicyGradient()
//
// The observation normalizer is built in and ON by default, because forgetting it is the most common reason a robot policy silently refuses to learn.
//
// Update loop for a mini-batch of stored transitions:
//
//     zeroGrad();
//     for ( each sample )
//     {
//         forward( obs );
//         ... compute d_policy[] and d_value from the stored action / advantage / return ...
//         backward( d_policy, d_value );     // accumulates
//     }
//     scaleGradients( 1 / batch_size );
//     clipGlobalGradNorm( paramBlocks(), 0.5 );
//     optimizer->step();

#include "net.h"
#include "running_normalizer.h"
#include <vector>


namespace zygo {
namespace nn {

struct ActorCriticConfig
{
  int obs_size;
  int action_outputs; // number of discrete actions, or number of continuous action dimensions

  std::vector<int> trunk_hidden;  // e.g. { 64, 64 }; may be empty - then the heads see the observation directly
  std::vector<int> actor_hidden;  // usually empty: one linear layer on top of the trunk
  std::vector<int> critic_hidden; // usually empty

  Activation hidden_activation;

  // The policy head is initialized small so that the starting policy is nearly uniform (discrete) or nearly zero-mean (continuous).
  // Starting from a confident random policy wastes the first few thousand episodes unlearning it.
  Real actor_output_scale;

  bool normalize_observations;

  ActorCriticConfig();
};


class ActorCriticNet
{
private:
  ActorCriticConfig config;

  Net trunk;
  Net actor;
  Net critic;

  bool has_trunk;

  // Continuous policies only: a free parameter vector, not a network output.
  std::vector<Real> log_std;
  std::vector<Real> d_log_std;
  bool use_log_std;

  RunningNormalizer obs_normalizer;

  std::vector<Real> obs_norm_buf;
  std::vector<Real> d_head_actor;
  std::vector<Real> d_head_critic;
  std::vector<Real> d_trunk;

public:
  explicit ActorCriticNet( ActorCriticConfig const& cfg );

  ActorCriticNet( ActorCriticNet const& ) = delete;
  ActorCriticNet& operator=( ActorCriticNet const& ) = delete;

  ActorCriticConfig const& cfg() const { return config; }

  inline int obsSize      () const { return config.obs_size; }
  inline int actionOutputs() const { return config.action_outputs; }

  // Continuous policies: allocate the log-sigma vector.
  // init_log_std = log of the initial sigma in ACTION UNITS - for a policy whose action is a joint-angle offset in radians,
  // sigma ~ 0.1 rad
  // (init_log_std ~ -2.3) is a sane start.
  void enablePolicyLogStd( Real init_log_std );

  inline bool hasPolicyLogStd() const { return use_log_std; }
  inline Real const* policyLogStd() const { return log_std.data(); }

  RunningNormalizer      & observationNormalizer()       { return obs_normalizer; }
  RunningNormalizer const& observationNormalizer() const { return obs_normalizer; }

  // Runs the trunk and both heads.
  // update_normalizer = false during evaluation and on the real robot.
  void forward( Real const* obs, bool update_normalizer = true, bool with_derivative = true );

  // Policy head output: logits (discrete) or means (continuous).
  Real const* actorOutput() const { return actor.output(); }
  Real value() const { return critic.output()[0]; }

  // One backward pass for the sample that was last forward()ed.
  // Accumulates into all gradients.
  //   d_actor_out - dL/d(actor output), size actionOutputs()
  //   d_value     - dL/d(value), a single number
  //   d_log_std_in- dL/d(log_std), size actionOutputs(); nullptr when there is no log-sigma
  void backward( Real const* d_actor_out, Real d_value, Real const* d_log_std_in = nullptr );

  void zeroGrad();
  void scaleGradients( Real k ); // divide by the batch size before stepping

  void collectParams( std::vector<ParamBlock>& out );

  Net      & trunkNet()        { return trunk; }
  Net      & actorNet()        { return actor; }
  Net      & criticNet()       { return critic; }
  Net const& trunkNet()  const { return trunk; }
  Net const& actorNet()  const { return actor; }
  Net const& criticNet() const { return critic; }

  void copyParamsFrom( ActorCriticNet const& other );

  int parametersCount() const;

  bool saveToFile  ( char const* filename ) const;
  bool loadFromFile( char const* filename );
};

} // namespace nn
} // namespace zygo
