#pragma once

// Output-head mathematics for policy networks.
//
// Two families, both needed:
//
//   CATEGORICAL -  a finite set of actions. What cartpole uses.
//                  Cheap, robust, and enough whenever the action is a choice ("which gait", "which foothold"),
//                  but it scales as (levels)^(joints), so it cannot drive twelve joints directly.
//
//   DIAGONAL GAUSSIAN - a continuous action vector. What the robot dog needs:
//                  the network emits a mean per joint and a learnable log-sigma per joint, and the action is sampled around that mean.
//                  The sigma is state-INDEPENDENT (a plain parameter vector, not a network output) - the standard choice for locomotion,
//                  because a state-dependent sigma tends to collapse to zero early and kill exploration.
//
// Everything here writes into caller-supplied buffers.

#include <zygo/core/types.h>


namespace zygo {
namespace nn {

// -------------------------------------------------------------------------------- categorical
// Numerically stable softmax. probs may alias logits.
void softMax( Real const* logits, Real* probs, int n );

// Index sampled with the given probabilities.
int sampleCategorical( Real const* probs, int n );

Real categoricalEntropy( Real const* probs, int n );

// Gradient of the policy-gradient loss w.r.t. the LOGITS:
//
//   L = -log( p[action_idx] ) * advantage   -   entropy_coeff * H(p)
//
//   dL/dz_j = (p_j - [j == action_idx]) * advantage  +  entropy_coeff * p_j * (log p_j + H)
//
// Feed the result straight into the actor head's backward().
// Note the sign convention: this is a LOSS gradient for gradient DESCENT.
void categoricalPolicyGradient(
    Real const* probs
  , int         action_idx
  , Real        advantage
  , Real        entropy_coeff
  , Real*       out_dlogits
  , int         n
  );


// -------------------------------------------------------------------------------- diagonal gaussian
// Samples action[i] = mean[i] + exp(log_std[i]) * eps[i], and reports the eps it used.
// Keeping eps is what makes the gradient below a one-liner - do not recompute it from the action.
void sampleGaussian(
    Real const* mean
  , Real const* log_std
  , Real*       out_action
  , Real*       out_eps
  , int         n
  );

Real gaussianLogProb( Real const* eps, Real const* log_std, int n );

Real gaussianEntropy( Real const* log_std, int n );

// Gradients of  L = -log pi(a|s) * advantage  -  entropy_coeff * H
// w.r.t. the mean vector (a network output) and w.r.t. log_std (a free parameter vector).
// out_dlog_std may be nullptr if the sigma is frozen.
void gaussianPolicyGradient(
    Real const* eps
  , Real const* log_std
  , Real        advantage
  , Real        entropy_coeff
  , Real*       out_dmean
  , Real*       out_dlog_std
  , int         n
  );

} // namespace nn
} // namespace zygo
