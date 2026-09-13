#pragma once

// Element-wise activation functions and their derivatives.
//
// Every function is evaluated as a PAIR (value, derivative) in one call:
// the derivative is almost always cheaper to obtain from the value than from scratch (tanh, sigmoid, elu, swish all reuse it),
// and the backward pass needs it at exactly the same point.

#include <zygo/core/types.h>


namespace zygo {
namespace nn {

enum class Activation
{
  LINEAR = 0,   // f = x                  - output layers (value head, action means, logits)
  RELU,         // f = max(0, x)
  LEAKY_RELU,   // f = x, or k*x for x<0  - the safe default for hidden layers
  CLAMPED,      // saturating linear on [-1/2, +1/2] -> [0, 1]      (DRELU)
  CLAMPED_LEAKY,// same, with a leaky slope outside the band        (DLRELU)
  ELU,
  SIGMOID,
  TANH,         // the usual choice for robot policies: bounded, smooth, zero-centred
  ARCTAN,
  SOFTSIGN,
  SWISH,

  COUNT
};

char const* activationName( Activation a );

// Leaky slope shared by LEAKY_RELU and CLAMPED_LEAKY, and the ELU coefficient.
constexpr Real LEAKY_RELU_SLOPE = (Real)0.01;
constexpr Real ELU_ALPHA        = (Real)1.0;

// Applies the activation element-wise.
//   z     - pre-activation values, size n
//   y     - output,                size n; may alias z
//   dy_dz - derivative at z,       size n; pass nullptr for an inference-only pass
void applyActivation( Activation a, Real const* z, Real* y, Real* dy_dz, int n );

// True for the ReLU family, where He initialization (sigma^2 = 2/fan_in) is the right default instead of Xavier (sigma^2 = 1/fan_in).
bool isReluFamily( Activation a );

} // namespace nn
} // namespace zygo
