#include "actor_critic.h"
#include "optimizer.h"
#include <zygo/core/assert.h>
#include <zygo/core/io/byte_reader.h>
#include <zygo/core/io/byte_writer.h>


namespace zygo {
namespace nn {

static const u32 AC_MAGIC   = 0x31434120; // ' AC1'
static const u32 AC_VERSION = 1;


ActorCriticConfig::ActorCriticConfig()
  : obs_size                ( 0 )
  , action_outputs          ( 0 )
  , hidden_activation       ( Activation::TANH )
  , actor_output_scale      ( (Real)0.01 )
  , normalize_observations  ( true )
{
}


static void buildStack( Net& net, int in_size, std::vector<int> const& hidden, int out_size, Activation act, char const* name )
{
  int prev = in_size;

  for ( size_t k = 0; k < hidden.size(); ++k )
  {
    net.addLayer( prev, hidden[k], act, name );
    prev = hidden[k];
  }

  if ( out_size > 0 )
    net.addLayer( prev, out_size, Activation::LINEAR, name );
}


ActorCriticNet::ActorCriticNet( ActorCriticConfig const& cfg )
  : config          ( cfg )
  , has_trunk       ( !cfg.trunk_hidden.empty() )
  , use_log_std     ( false )
  , obs_normalizer  ( cfg.obs_size )
{
  ZgAssert( cfg.obs_size       > 0 );
  ZgAssert( cfg.action_outputs > 0 );

  int head_in = cfg.obs_size;

  if ( has_trunk )
  {
    // The trunk has no linear output layer of its own: its last hidden layer IS the shared feature vector.
    buildStack( trunk, cfg.obs_size, cfg.trunk_hidden, 0, cfg.hidden_activation, "trunk" );
    head_in = trunk.outSize();
  }

  buildStack( actor , head_in, cfg.actor_hidden , cfg.action_outputs, cfg.hidden_activation, "actor"  );
  buildStack( critic, head_in, cfg.critic_hidden, 1                 , cfg.hidden_activation, "critic" );

  actor.scaleLastLayerWeights( cfg.actor_output_scale );

  obs_norm_buf .assign( (size_t)cfg.obs_size      , REAL_ZERO );
  d_head_actor .assign( (size_t)head_in           , REAL_ZERO );
  d_head_critic.assign( (size_t)head_in           , REAL_ZERO );
  d_trunk      .assign( (size_t)head_in           , REAL_ZERO );

  if ( !cfg.normalize_observations )
    obs_normalizer.freeze();
}

void ActorCriticNet::enablePolicyLogStd( Real init_log_std )
{
  log_std  .assign( (size_t)config.action_outputs, init_log_std );
  d_log_std.assign( (size_t)config.action_outputs, REAL_ZERO    );

  use_log_std = true;
}

void ActorCriticNet::forward( Real const* obs, bool update_normalizer, bool with_derivative )
{
  Real const* x = obs;

  if ( config.normalize_observations )
  {
    if ( update_normalizer )
      obs_normalizer.update( obs );

    obs_normalizer.normalize( obs, obs_norm_buf.data() );
    x = obs_norm_buf.data();
  }

  Real const* features = x;

  if ( has_trunk )
    features = trunk.forward( x, with_derivative );

  actor .forward( features, with_derivative );
  critic.forward( features, with_derivative );
}

void ActorCriticNet::backward( Real const* d_actor_out, Real d_value, Real const* d_log_std_in )
{
  if ( use_log_std && d_log_std_in )
  {
    for ( size_t i = 0; i < d_log_std.size(); ++i )
      d_log_std[i] += d_log_std_in[i];
  }

  if ( !has_trunk )
  {
    actor .backward( d_actor_out, nullptr );
    critic.backward( &d_value   , nullptr );
    return;
  }

  actor .backward( d_actor_out, d_head_actor .data() );
  critic.backward( &d_value   , d_head_critic.data() );

  // The trunk receives the SUM of what the two heads want from it. This is the step that the old
  // wiring performed implicitly, by having both heads += into the same layer's gradient array.
  for ( size_t i = 0; i < d_trunk.size(); ++i )
    d_trunk[i] = d_head_actor[i] + d_head_critic[i];

  trunk.backward( d_trunk.data(), nullptr );
}

void ActorCriticNet::zeroGrad()
{
  if ( has_trunk )
    trunk.zeroGrad();

  actor .zeroGrad();
  critic.zeroGrad();

  for ( size_t i = 0; i < d_log_std.size(); ++i )
    d_log_std[i] = REAL_ZERO;
}

void ActorCriticNet::scaleGradients( Real k )
{
  std::vector<ParamBlock> blocks;
  collectParams( blocks );

  for ( size_t b = 0; b < blocks.size(); ++b )
  {
    Real* g = blocks[b].grads;
    int n = blocks[b].count;

    while ( n-- )
      *g++ *= k;
  }
}

void ActorCriticNet::collectParams( std::vector<ParamBlock>& out )
{
  if ( has_trunk )
    trunk.collectParams( out );

  actor .collectParams( out );
  critic.collectParams( out );

  if ( use_log_std )
    out.push_back( ParamBlock{ log_std.data(), d_log_std.data(), (int)log_std.size(), true } );
}

void ActorCriticNet::copyParamsFrom( ActorCriticNet const& other )
{
  if ( has_trunk )
    trunk.copyParamsFrom( other.trunk );

  actor .copyParamsFrom( other.actor  );
  critic.copyParamsFrom( other.critic );

  log_std = other.log_std;
}

int ActorCriticNet::parametersCount() const
{
  int total = actor.parametersCount() + critic.parametersCount() + (int)log_std.size();

  if ( has_trunk )
    total += trunk.parametersCount();

  return total;
}

bool ActorCriticNet::saveToFile( char const* filename ) const
{
  ByteWriter sr;

  sr.writeU32( AC_MAGIC );
  sr.writeU32( AC_VERSION );
  sr.writeU32( (u32)config.obs_size );
  sr.writeU32( (u32)config.action_outputs );
  sr.writeBool( has_trunk );
  sr.writeBool( use_log_std );

  if ( has_trunk )
    trunk.serialize( sr );

  actor .serialize( sr );
  critic.serialize( sr );

  if ( use_log_std )
  {
    for ( size_t i = 0; i < log_std.size(); ++i )
      sr.writeDouble( (double)log_std[i] );
  }

  // The normalizer travels WITH the weights. A policy shipped without its input statistics
  // is not a policy, it is a random number generator.
  obs_normalizer.serialize( sr );

  return sr.isOk() && sr.saveToFile( filename );
}

bool ActorCriticNet::loadFromFile( char const* filename )
{
  ByteReader ds( filename );

  if ( !ds.isOk() )                                 return false;
  if ( ds.readU32() != AC_MAGIC )                   return false;
  if ( ds.readU32() != AC_VERSION )                 return false;
  if ( ds.readU32() != (u32)config.obs_size )       return false;
  if ( ds.readU32() != (u32)config.action_outputs ) return false;

  const bool file_has_trunk   = ds.readBool();
  const bool file_use_log_std = ds.readBool();

  if ( file_has_trunk != has_trunk )
    return false;

  if ( has_trunk && !trunk.deserialize( ds ) ) return false;
  if ( !actor .deserialize( ds ) )             return false;
  if ( !critic.deserialize( ds ) )             return false;

  if ( file_use_log_std )
  {
    if ( !use_log_std )
      enablePolicyLogStd( REAL_ZERO );

    for ( size_t i = 0; i < log_std.size(); ++i )
      log_std[i] = (Real)ds.readDouble();
  }

  if ( !obs_normalizer.deserialize( ds ) )
    return false;

  return ds.isOk();
}

} // namespace nn
} // namespace zygo
