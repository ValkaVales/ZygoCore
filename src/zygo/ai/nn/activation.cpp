#include "activation.h"
#include <zygo/math/common/scalar.h>
#include <cmath>


namespace zygo {
namespace nn {

char const* activationName( Activation a )
{
  switch ( a )
  {
  case Activation::LINEAR:        return "linear";
  case Activation::RELU:          return "relu";
  case Activation::LEAKY_RELU:    return "leaky_relu";
  case Activation::CLAMPED:       return "clamped";
  case Activation::CLAMPED_LEAKY: return "clamped_leaky";
  case Activation::ELU:           return "elu";
  case Activation::SIGMOID:       return "sigmoid";
  case Activation::TANH:          return "tanh";
  case Activation::ARCTAN:        return "arctan";
  case Activation::SOFTSIGN:      return "softsign";
  case Activation::SWISH:         return "swish";
  default:                        return "?";
  }
}

bool isReluFamily( Activation a )
{
  return a == Activation::RELU
      || a == Activation::LEAKY_RELU
      || a == Activation::CLAMPED
      || a == Activation::CLAMPED_LEAKY
      || a == Activation::ELU;
}


// ---------------------------------------------------------------------------------- single-element kernels
static inline void fLinear( Real x, Real& f, Real& df )
{
  f  = x;
  df = REAL_ONE;
}

static inline void fRelu( Real x, Real& f, Real& df )
{
  if ( x > REAL_ZERO )
  {
    f  = x;
    df = REAL_ONE;
    return;
  }

  f  = REAL_ZERO;
  df = REAL_ZERO;
}

static inline void fLeakyRelu( Real x, Real& f, Real& df )
{
  if ( x > REAL_ZERO )
  {
    f  = x;
    df = REAL_ONE;
    return;
  }

  f  = LEAKY_RELU_SLOPE * x;
  df = LEAKY_RELU_SLOPE;
}

static inline void fClamped( Real x, Real& f, Real& df )
{
  if ( x >= REAL_HALF )
  {
    f  = REAL_ONE;
    df = REAL_ZERO;
    return;
  }

  if ( x <= -REAL_HALF )
  {
    f  = REAL_ZERO;
    df = REAL_ZERO;
    return;
  }

  f  = x + REAL_HALF; // shifted so that the band maps onto [0, 1] continuously
  df = REAL_ONE;
}

static inline void fClampedLeaky( Real x, Real& f, Real& df )
{
  if ( x >= REAL_HALF )
  {
    f  = REAL_ONE + (x - REAL_HALF) * LEAKY_RELU_SLOPE;
    df = LEAKY_RELU_SLOPE;
    return;
  }

  if ( x <= -REAL_HALF )
  {
    f  = (x + REAL_HALF) * LEAKY_RELU_SLOPE;
    df = LEAKY_RELU_SLOPE;
    return;
  }

  f  = x + REAL_HALF;
  df = REAL_ONE;
}

static inline void fElu( Real x, Real& f, Real& df )
{
  if ( x >= REAL_ZERO )
  {
    f  = x;
    df = REAL_ONE;
    return;
  }

  f  = ELU_ALPHA * (std::exp( x ) - REAL_ONE);
  df = f + ELU_ALPHA; // == alpha * exp(x)
}

static inline void fSigmoid( Real x, Real& f, Real& df )
{
  f  = REAL_ONE / (REAL_ONE + std::exp( -x ));
  df = f * (REAL_ONE - f);
}

static inline void fTanh( Real x, Real& f, Real& df )
{
  f  = std::tanh( x );
  df = REAL_ONE - sqr( f );
}

static inline void fArctan( Real x, Real& f, Real& df )
{
  f  = std::atan( x );
  df = REAL_ONE / (REAL_ONE + sqr( x ));
}

static inline void fSoftsign( Real x, Real& f, Real& df )
{
  Real ax = REAL_ONE + std::fabs( x );
  f  = x / ax;
  df = REAL_ONE / sqr( ax );
}

static inline void fSwish( Real x, Real& f, Real& df )
{
  Real s = REAL_ONE / (REAL_ONE + std::exp( -x ));

  f  = x * s;
  df = f + s * (REAL_ONE - f);
}


// ---------------------------------------------------------------------------------- dispatch
// The switch sits OUTSIDE the loop: one branch per layer instead of one branch per neuron, and the body of each loop is small enough to be vectorized.
#define ZG_NN_ACTIVATION_LOOP( kernel )                 \
  {                                                     \
    if ( dy_dz )                                        \
    {                                                   \
      for ( int i = 0; i < n; ++i )                     \
        kernel( z[i], y[i], dy_dz[i] );                 \
    }                                                   \
    else                                                \
    {                                                   \
      Real scratch;                                     \
      for ( int i = 0; i < n; ++i )                     \
        kernel( z[i], y[i], scratch );                  \
    }                                                   \
    return;                                             \
  }

void applyActivation( Activation a, Real const* z, Real* y, Real* dy_dz, int n )
{
  switch ( a )
  {
  case Activation::LINEAR:        ZG_NN_ACTIVATION_LOOP( fLinear       );
  case Activation::RELU:          ZG_NN_ACTIVATION_LOOP( fRelu         );
  case Activation::LEAKY_RELU:    ZG_NN_ACTIVATION_LOOP( fLeakyRelu    );
  case Activation::CLAMPED:       ZG_NN_ACTIVATION_LOOP( fClamped      );
  case Activation::CLAMPED_LEAKY: ZG_NN_ACTIVATION_LOOP( fClampedLeaky );
  case Activation::ELU:           ZG_NN_ACTIVATION_LOOP( fElu          );
  case Activation::SIGMOID:       ZG_NN_ACTIVATION_LOOP( fSigmoid      );
  case Activation::TANH:          ZG_NN_ACTIVATION_LOOP( fTanh         );
  case Activation::ARCTAN:        ZG_NN_ACTIVATION_LOOP( fArctan       );
  case Activation::SOFTSIGN:      ZG_NN_ACTIVATION_LOOP( fSoftsign     );
  case Activation::SWISH:         ZG_NN_ACTIVATION_LOOP( fSwish        );
  default:                        ZG_NN_ACTIVATION_LOOP( fLinear       );
  }
}

#undef ZG_NN_ACTIVATION_LOOP

} // namespace nn
} // namespace zygo
