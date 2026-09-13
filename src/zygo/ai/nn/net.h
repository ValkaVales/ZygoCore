#pragma once

// A sequential stack of DenseLayers.
//
// Here a Net is a closed box with two ports:
//     forward( x )          -> output()
//     backward( dL/dy, dx ) -> accumulates parameter gradients, optionally returns dL/dx
//
// Because backward() can hand back dL/dx, an actor-critic with a shared trunk is just:
//
//     trunk.forward( obs );
//     actor .forward( trunk.output() );
//     critic.forward( trunk.output() );
//     ...
//     trunk.zeroGrad(); actor.zeroGrad(); critic.zeroGrad();   // once per update
//     actor .backward( d_actor , d_trunk_a.data() );
//     critic.backward( d_critic, d_trunk_c.data() );
//     for ( i ) d_trunk[i] = d_trunk_a[i] + d_trunk_c[i];
//     trunk.backward( d_trunk.data(), nullptr );
//
// - no class in the chain knows about the others.
// 
// Use ActorCriticNet (actor_critic.h) to avoid writing that by hand.

#include "dense_layer.h"
#include <memory>
#include <vector>


namespace zygo {

class ByteReader;
class ByteWriter;

namespace nn {

class Net
{
private:
  std::vector<std::unique_ptr<DenseLayer>> layers;

  // Two alternating scratch buffers for the backward pass, sized to the widest layer.
  mutable std::vector<Real> back_buf[2];

public:
  Net();
  ~Net();

  Net( Net const& ) = delete;
  Net& operator=( Net const& ) = delete;

  // First call needs in_size; later ones take it from the previous layer.
  DenseLayer& addLayer( int in_size, int out_size, Activation act, char const* name = nullptr );
  DenseLayer& addLayer(              int out_size, Activation act, char const* name = nullptr );

  inline int layersCount() const { return (int)layers.size(); }

  DenseLayer      & layer( int idx )       { return *layers[(size_t)idx]; }
  DenseLayer const& layer( int idx ) const { return *layers[(size_t)idx]; }

  DenseLayer      & lastLayer()       { return *layers.back(); }
  DenseLayer const& lastLayer() const { return *layers.back(); }

  int inSize () const;
  int outSize() const;

  Real const* output() const;

  Real const* forward( Real const* in, bool with_derivative = true );

  // dy - dL/d(output), size outSize()
  // dx - dL/d(input), size inSize(); nullptr when the caller does not need it
  void backward( Real const* dy, Real* dx = nullptr );

  void zeroGrad();

  void initWeights( Real gain = REAL_ONE );          // re-rolls every layer
  void scaleLastLayerWeights( Real k );              // policy head: k ~ 0.01 keeps the initial policy near-uniform

  void collectParams( std::vector<ParamBlock>& out );

  void copyParamsFrom( Net const& other );
  void lerpParamsFrom( Net const& other, Real tau );

  Real calcGradSquaresSum  () const;
  Real calcWeightSquaresSum() const;

  int parametersCount() const;

  void printWeights( int max_dim ) const;

  // --- serialization ---------------------------------------------------------------------------
  // Self-describing: a magic word, a format version and the full shape of every layer go into the stream, and deserialize() refuses a file whose shape does not match this net.
  u32  sizeOfSerialize() const;
  void serialize( ByteWriter& sr ) const;
  bool deserialize( ByteReader& ds ); // false = wrong magic / version / shape

  bool saveToFile  ( char const* filename ) const;
  bool loadFromFile( char const* filename );

private:
  void ensureBackBuffers();
};

} // namespace nn
} // namespace zygo
