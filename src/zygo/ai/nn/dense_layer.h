#pragma once

// A fully connected layer:  y = activation( W * x + b ).
//
// The bias is a REAL VECTOR.
//
// Gradients are ACCUMULATED into dw/db instead of being applied to the weights immediately.
// Mini-batching, multiple backward calls per update (actor + critic sharing a trunk), and any optimizer with state (Adam) all require this separation.
//
// The layer does not know its neighbours.
// backward() takes dL/dy and optionally writes dL/dx, so composing layers - or whole networks - is the caller's business.
// That is what lets a trunk feed two heads without any of the three knowing about the others.

#include "activation.h"
#include <zygo/math/matrix/matrix.h>
#include <vector>


namespace zygo {

class ByteReader;
class ByteWriter;

namespace nn {

struct ParamBlock; // optimizer.h


class DenseLayer
{
private:
  Matrix w;  // [out_size, in_size]
  Matrix dw; // gradient accumulator, same shape

  std::vector<Real> b;  // [out_size]
  std::vector<Real> db;

  std::vector<Real> x_cache; // input of the last forward(), needed by backward()
  std::vector<Real> y;       // output
  std::vector<Real> dy_dz;   // activation derivative at the last pre-activation

  Activation activation;
  char const* name;

public:
  DenseLayer( int in_size, int out_size, Activation activation, char const* name = nullptr );

  inline int inSize () const { return w.dimX(); }
  inline int outSize() const { return w.dimY(); }

  inline Activation  activationType() const { return activation; }
  inline char const* layerName     () const { return name ? name : "layer"; }

  inline Real const* output() const { return y.data(); }

  // Computes the layer output. Returns output().
  // with_derivative = false skips filling dy_dz - use it in a pure inference path (on the robot).
  Real const* forward( Real const* in, bool with_derivative = true );

  // ACCUMULATES the gradient of the loss w.r.t. w and b.
  // dy - dL/dy, size outSize().
  // dx - dL/dx, size inSize(); pass nullptr for the first layer of a net, where nobody needs it.
  void backward( Real const* dy, Real* dx );

  void zeroGrad();

  // --- weights ---------------------------------------------------------------------------------
  void initWeights( Real gain = REAL_ONE ); // gaussian; He for the ReLU family, Xavier otherwise; biases = 0
  void scaleWeights( Real k );              // used to shrink the policy head so the initial policy is near-uniform

  inline Matrix const& weights() const { return w; }
  inline Matrix      & weights()       { return w; }
  inline std::vector<Real> const& biases() const { return b; }

  Real getWeight( int out_idx, int in_idx ) const { return w.get( out_idx, in_idx ); }

  void copyParamsFrom( DenseLayer const& other );
  void lerpParamsFrom( DenseLayer const& other, Real tau ); // p = (1-tau)*p + tau*other.p  (target networks)

  void collectParams( std::vector<ParamBlock>& out );

  Real calcGradSquaresSum() const;
  Real calcWeightSquaresSum() const; // biases excluded on purpose

  void printWeights( int max_dim ) const;

  // --- serialization ---------------------------------------------------------------------------
  // Only w and b are stored. values/errors/gradients are per-step scratch and have no business in a file.
  u32  sizeOfSerialize() const;
  void serialize( ByteWriter& sr ) const;
  void deserialize( ByteReader& ds );
};

} // namespace nn
} // namespace zygo
