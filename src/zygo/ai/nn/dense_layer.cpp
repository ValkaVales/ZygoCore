#include "dense_layer.h"
#include "optimizer.h"
#include <zygo/core/assert.h>
#include <zygo/core/io/byte_reader.h>
#include <zygo/core/io/byte_writer.h>
#include <zygo/random/random.h>
#include <zygo/debug/print_matrix.h>
#include <cmath>


namespace zygo {
namespace nn {

DenseLayer::DenseLayer( int in_size, int out_size, Activation activation, char const* name )
  : w           ( out_size, in_size, name )
  , dw          ( out_size, in_size )
  , b           ( (size_t)out_size, REAL_ZERO )
  , db          ( (size_t)out_size, REAL_ZERO )
  , x_cache     ( (size_t)in_size , REAL_ZERO )
  , y           ( (size_t)out_size, REAL_ZERO )
  , dy_dz       ( (size_t)out_size, REAL_ZERO )
  , activation  ( activation )
  , name        ( name )
{
  ZgAssert( in_size  > 0 );
  ZgAssert( out_size > 0 );

  dw.makeAllZero();
  initWeights();
}

void DenseLayer::initWeights( Real gain )
{
  // He for the ReLU family (variance 2/fan_in), Xavier otherwise (1/fan_in).
  //
  // NOTE: the old code did  fan_in *= 2; sigma = sqrt(1/fan_in)  for ReLU, which gives sqrt(1/(2*fan_in))
  // - HALF the intended variance instead of double. Signal amplitude then decayed layer by layer.
  // Harmless in a 2-layer net, fatal in a 4-layer one.
  const Real fan_in   = (Real)inSize();
  const Real variance = isReluFamily( activation ) ? (REAL_TWO / fan_in) : (REAL_ONE / fan_in);

  w.makeAllGaussRandom( REAL_ZERO, gain * std::sqrt( variance ) );

  for ( size_t i = 0; i < b.size(); ++i )
    b[i] = REAL_ZERO;
}

void DenseLayer::scaleWeights( Real k )
{
  Real* p = w.getArr();
  int n = w.getSize();

  while ( n-- )
    *p++ *= k;
}

Real const* DenseLayer::forward( Real const* in, bool with_derivative )
{
  ZgAssert( in != nullptr );

  const int in_n  = inSize();
  const int out_n = outSize();

  // Cache the input: backward() needs it, and the caller is free to reuse its buffer meanwhile.
  for ( int i = 0; i < in_n; ++i )
    x_cache[i] = in[i];

  // z = W*x + b, written straight into y and then activated in place.
  Real const* p_w = w.getArr();
  Real      * p_y = y.data();

  for ( int o = 0; o < out_n; ++o )
  {
    Real sum = b[o];

    for ( int i = 0; i < in_n; ++i )
      sum += p_w[i] * in[i];

    p_w += in_n;
    p_y[o] = sum;
  }

  applyActivation( activation, p_y, p_y, with_derivative ? dy_dz.data() : nullptr, out_n );

  return p_y;
}

void DenseLayer::backward( Real const* dy, Real* dx )
{
  ZgAssert( dy != nullptr );

  const int in_n  = inSize();
  const int out_n = outSize();

  if ( dx )
  {
    for ( int i = 0; i < in_n; ++i )
      dx[i] = REAL_ZERO;
  }

  Real const* p_w  = w .getArr();
  Real      * p_dw = dw.getArr();
  Real const* p_x  = x_cache.data();

  for ( int o = 0; o < out_n; ++o )
  {
    // dL/dz = dL/dy * dy/dz
    const Real dz = dy[o] * dy_dz[o];

    db[o] += dz;

    if ( dx )
    {
      for ( int i = 0; i < in_n; ++i )
      {
        p_dw[i] += dz * p_x[i];
        dx  [i] += dz * p_w [i];
      }
    }
    else
    {
      for ( int i = 0; i < in_n; ++i )
        p_dw[i] += dz * p_x[i];
    }

    p_w  += in_n;
    p_dw += in_n;
  }
}

void DenseLayer::zeroGrad()
{
  dw.makeAllZero();

  for ( size_t i = 0; i < db.size(); ++i )
    db[i] = REAL_ZERO;
}

void DenseLayer::copyParamsFrom( DenseLayer const& other )
{
  ZgAssert( inSize () == other.inSize () );
  ZgAssert( outSize() == other.outSize() );

  w.copyFrom( other.w );
  b = other.b;
}

void DenseLayer::lerpParamsFrom( DenseLayer const& other, Real tau )
{
  ZgAssert( inSize () == other.inSize () );
  ZgAssert( outSize() == other.outSize() );

  Real      * p_dst = w.getArr();
  Real const* p_src = other.w.getArr();

  int n = w.getSize();
  while ( n-- )
  {
    *p_dst += tau * (*p_src - *p_dst);
    ++p_dst;
    ++p_src;
  }

  for ( size_t i = 0; i < b.size(); ++i )
    b[i] += tau * (other.b[i] - b[i]);
}

void DenseLayer::collectParams( std::vector<ParamBlock>& out )
{
  out.push_back( ParamBlock{ w.getArr(), dw.getArr(), w.getSize(), false } );
  out.push_back( ParamBlock{ b.data()  , db.data()  , (int)b.size(), true } );
}

Real DenseLayer::calcGradSquaresSum() const
{
  Real sum = REAL_ZERO;

  Real const* p = dw.getArr();
  int n = dw.getSize();

  while ( n-- )
  {
    sum += (*p) * (*p);
    ++p;
  }

  for ( size_t i = 0; i < db.size(); ++i )
    sum += db[i] * db[i];

  return sum;
}

Real DenseLayer::calcWeightSquaresSum() const
{
  Real sum = REAL_ZERO;

  Real const* p = w.getArr();
  int n = w.getSize();

  while ( n-- )
  {
    sum += (*p) * (*p);
    ++p;
  }

  return sum;
}

void DenseLayer::printWeights( int max_dim ) const
{
  printMatrix( w, layerName(), max_dim );
}


// ------------------------------------------------------------------------------------ serialization
u32 DenseLayer::sizeOfSerialize() const
{
  return (u32)( ( w.getSize() + (int)b.size() ) * DOUBLE_SZ );
}

void DenseLayer::serialize( ByteWriter& sr ) const
{
  Real const* p = w.getArr();
  int n = w.getSize();

  while ( n-- )
    sr.writeDouble( (double)*p++ );

  for ( size_t i = 0; i < b.size(); ++i )
    sr.writeDouble( (double)b[i] );
}

void DenseLayer::deserialize( ByteReader& ds )
{
  Real* p = w.getArr();
  int n = w.getSize();

  while ( n-- )
    *p++ = (Real)ds.readDouble();

  for ( size_t i = 0; i < b.size(); ++i )
    b[i] = (Real)ds.readDouble();
}

} // namespace nn
} // namespace zygo
