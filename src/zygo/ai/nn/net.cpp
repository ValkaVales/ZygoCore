#include "net.h"
#include "optimizer.h"
#include <zygo/core/assert.h>
#include <zygo/core/io/byte_reader.h>
#include <zygo/core/io/byte_writer.h>


namespace zygo {
namespace nn {

static const u32 NET_MAGIC   = 0x314e4e5a; // 'ZNN1'
static const u32 NET_VERSION = 1;


Net::Net()
{
}

Net::~Net()
{
}

DenseLayer& Net::addLayer( int in_size, int out_size, Activation act, char const* name )
{
  ZgAssert( layers.empty() || layers.back()->outSize() == in_size );

  layers.push_back( std::unique_ptr<DenseLayer>( new DenseLayer( in_size, out_size, act, name ) ) );
  ensureBackBuffers();

  return *layers.back();
}

DenseLayer& Net::addLayer( int out_size, Activation act, char const* name )
{
  ZgAssert( !layers.empty() ); // the first layer must declare its input size

  return addLayer( layers.back()->outSize(), out_size, act, name );
}

void Net::ensureBackBuffers()
{
  size_t widest = 0;

  for ( size_t k = 0; k < layers.size(); ++k )
  {
    if ( (size_t)layers[k]->inSize () > widest ) widest = (size_t)layers[k]->inSize ();
    if ( (size_t)layers[k]->outSize() > widest ) widest = (size_t)layers[k]->outSize();
  }

  back_buf[0].assign( widest, REAL_ZERO );
  back_buf[1].assign( widest, REAL_ZERO );
}

int Net::inSize() const
{
  ZgAssert( !layers.empty() );
  return layers.front()->inSize();
}

int Net::outSize() const
{
  ZgAssert( !layers.empty() );
  return layers.back()->outSize();
}

Real const* Net::output() const
{
  ZgAssert( !layers.empty() );
  return layers.back()->output();
}

Real const* Net::forward( Real const* in, bool with_derivative )
{
  ZgAssert( !layers.empty() );

  Real const* cur = in;

  for ( size_t k = 0; k < layers.size(); ++k )
    cur = layers[k]->forward( cur, with_derivative );

  return cur;
}

void Net::backward( Real const* dy, Real* dx )
{
  ZgAssert( !layers.empty() );

  const int n = (int)layers.size();

  Real const* cur_dy = dy;

  for ( int k = n - 1; k >= 0; --k )
  {
    // For every layer but the first, dL/dx goes into a scratch buffer and becomes the next dL/dy.
    // Buffers alternate by parity of k, so the source and the destination are never the same one.
    Real* dst = ( k > 0 ) ? back_buf[(size_t)( k & 1 )].data() : dx;

    layers[(size_t)k]->backward( cur_dy, dst );

    cur_dy = dst;
  }
}

void Net::zeroGrad()
{
  for ( size_t k = 0; k < layers.size(); ++k )
    layers[k]->zeroGrad();
}

void Net::initWeights( Real gain )
{
  for ( size_t k = 0; k < layers.size(); ++k )
    layers[k]->initWeights( gain );
}

void Net::scaleLastLayerWeights( Real k )
{
  ZgAssert( !layers.empty() );
  layers.back()->scaleWeights( k );
}

void Net::collectParams( std::vector<ParamBlock>& out )
{
  for ( size_t k = 0; k < layers.size(); ++k )
    layers[k]->collectParams( out );
}

void Net::copyParamsFrom( Net const& other )
{
  ZgAssert( layersCount() == other.layersCount() );

  for ( size_t k = 0; k < layers.size(); ++k )
    layers[k]->copyParamsFrom( *other.layers[k] );
}

void Net::lerpParamsFrom( Net const& other, Real tau )
{
  ZgAssert( layersCount() == other.layersCount() );

  for ( size_t k = 0; k < layers.size(); ++k )
    layers[k]->lerpParamsFrom( *other.layers[k], tau );
}

Real Net::calcGradSquaresSum() const
{
  Real sum = REAL_ZERO;

  for ( size_t k = 0; k < layers.size(); ++k )
    sum += layers[k]->calcGradSquaresSum();

  return sum;
}

Real Net::calcWeightSquaresSum() const
{
  Real sum = REAL_ZERO;

  for ( size_t k = 0; k < layers.size(); ++k )
    sum += layers[k]->calcWeightSquaresSum();

  return sum;
}

int Net::parametersCount() const
{
  int total = 0;

  for ( size_t k = 0; k < layers.size(); ++k )
    total += layers[k]->weights().getSize() + layers[k]->outSize();

  return total;
}

void Net::printWeights( int max_dim ) const
{
  for ( size_t k = 0; k < layers.size(); ++k )
    layers[k]->printWeights( max_dim );
}


// ------------------------------------------------------------------------------------ serialization
u32 Net::sizeOfSerialize() const
{
  u32 total = 3 * U32_SZ; // magic, version, layers count

  for ( size_t k = 0; k < layers.size(); ++k )
    total += 3 * U32_SZ + layers[k]->sizeOfSerialize(); // in, out, activation + payload

  return total;
}

void Net::serialize( ByteWriter& sr ) const
{
  sr.writeU32( NET_MAGIC );
  sr.writeU32( NET_VERSION );
  sr.writeU32( (u32)layers.size() );

  for ( size_t k = 0; k < layers.size(); ++k )
  {
    sr.writeU32( (u32)layers[k]->inSize () );
    sr.writeU32( (u32)layers[k]->outSize() );
    sr.writeU32( (u32)layers[k]->activationType() );

    layers[k]->serialize( sr );
  }
}

bool Net::deserialize( ByteReader& ds )
{
  if ( ds.readU32() != NET_MAGIC   ) return false;
  if ( ds.readU32() != NET_VERSION ) return false;

  const u32 count = ds.readU32();
  if ( count != (u32)layers.size() )
    return false;

  for ( size_t k = 0; k < layers.size(); ++k )
  {
    const u32 in_size  = ds.readU32();
    const u32 out_size = ds.readU32();
    const u32 act      = ds.readU32();

    if ( in_size  != (u32)layers[k]->inSize ()          ) return false;
    if ( out_size != (u32)layers[k]->outSize()          ) return false;
    if ( act      != (u32)layers[k]->activationType()   ) return false;

    layers[k]->deserialize( ds );
  }

  return ds.isOk();
}

bool Net::saveToFile( char const* filename ) const
{
  ByteWriter sr;
  sr.reserve( sizeOfSerialize() );

  serialize( sr );

  if ( !sr.isOk() )
    return false;

  return sr.saveToFile( filename );
}

bool Net::loadFromFile( char const* filename )
{
  ByteReader ds( filename );

  if ( !ds.isOk() )
    return false;

  return deserialize( ds );
}

} // namespace nn
} // namespace zygo
