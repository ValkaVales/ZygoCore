#pragma once

// Optimizers.
//
// The old code had exactly one update rule, hard-wired inside the layer:
//     w -= learning_coeff * grad_j * a_i
// i.e. plain SGD with no momentum, no state, applied immediately, one sample at a time.
// For supervised toy problems that is fine. For reinforcement learning it is the single biggest reason training is slow and fragile:
// the gradient of a policy loss is extremely noisy, its scale changes by orders of magnitude during training, and a fixed step size cannot serve both ends.
// 
// Adam fixes exactly that by normalizing each coordinate by its own running RMS.
//
// An optimizer owns no parameters. It holds a list of ParamBlocks - raw pointers into the layers - plus whatever state it needs, indexed the same way.
// Build the network FIRST, then call setParams(); after that the network must not be resized.

#include <zygo/core/types.h>
#include <vector>


namespace zygo {
namespace nn {

// A contiguous run of parameters and the matching run of gradients.
struct ParamBlock
{
  Real* values;
  Real* grads;
  int   count;
  bool  is_bias; // biases are excluded from weight decay - decaying them just shifts the function for no benefit
};


// L2 norm of the whole gradient, across every block.
// Cheap, and the single most useful number to watch while debugging training: if it explodes, nothing downstream will save you.
Real calcGlobalGradNorm( std::vector<ParamBlock> const& blocks );

// Scales every gradient so that the global norm does not exceed max_norm.
// Returns the norm BEFORE clipping (so you can log how often it actually fires).
// max_norm <= 0 disables clipping and only measures.
Real clipGlobalGradNorm( std::vector<ParamBlock> const& blocks, Real max_norm );

void zeroGrads( std::vector<ParamBlock> const& blocks );


// ------------------------------------------------------------------------------------ IOptimizer
class IOptimizer
{
protected:
  std::vector<ParamBlock> params;
  Real learning_rate;

public:
  explicit IOptimizer( Real learning_rate ) : learning_rate( learning_rate ) {}
  virtual ~IOptimizer() {}

  virtual void setParams( std::vector<ParamBlock> const& blocks ) { params = blocks; reset(); }

  std::vector<ParamBlock> const& paramBlocks() const { return params; }

  inline Real learningRate() const { return learning_rate; }
  inline void setLearningRate( Real lr ) { learning_rate = lr; }

  // Applies one update using the gradients currently sitting in the blocks.
  // Does NOT zero them - call zeroGrad() explicitly, so that accumulating over a mini-batch stays an obvious, deliberate act rather than an accident.
  virtual void step() = 0;

  virtual void reset() = 0; // clears momentum / moment estimates

  void zeroGrad() { zeroGrads( params ); }

  int paramsCount() const;
};


// ------------------------------------------------------------------------------------ SgdOptimizer
class SgdOptimizer : public IOptimizer
{
private:
  Real momentum;
  Real weight_decay;

  std::vector<Real> velocity;

public:
  explicit SgdOptimizer( Real learning_rate, Real momentum = REAL_ZERO, Real weight_decay = REAL_ZERO );

  virtual void step();
  virtual void reset();

  void setMomentum    ( Real v ) { momentum     = v; }
  void setWeightDecay ( Real v ) { weight_decay = v; }
};


// ------------------------------------------------------------------------------------ AdamOptimizer
// Adam with decoupled weight decay (AdamW).
// Defaults are the ones everyone uses for RL: beta1 = 0.9, beta2 = 0.999, eps = 1e-8,
// and the eps INSIDE the sqrt-denominator, not added to it, which is the variant that behaves better when gradients are tiny.
class AdamOptimizer : public IOptimizer
{
private:
  Real beta1;
  Real beta2;
  Real eps;
  Real weight_decay;

  std::vector<Real> m; // first moment
  std::vector<Real> v; // second moment

  Real beta1_pow; // beta1^t, kept incrementally for the bias correction
  Real beta2_pow;

public:
  explicit AdamOptimizer(
      Real learning_rate = (Real)3e-4
    , Real beta1         = (Real)0.9
    , Real beta2         = (Real)0.999
    , Real eps           = (Real)1e-8
    , Real weight_decay  = REAL_ZERO
    );

  virtual void step();
  virtual void reset();

  void setBetas       ( Real b1, Real b2 ) { beta1 = b1; beta2 = b2; }
  void setWeightDecay ( Real v )           { weight_decay = v; }

  int stepsDone() const;
};

} // namespace nn
} // namespace zygo
