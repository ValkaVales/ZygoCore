#include "pid.h"
#include <zygo/math/common/consts.h>
#include <zygo/math/common/scalar.h>
#include <zygo/core/assert.h>
#include <cmath>


namespace zygo {

namespace
{
  // Below this the derivative would amplify by more than 1e6 and the result is noise, not
  // a control signal. A dt this small is a bug at the call site, not a regime to support.
  const Real PID_MIN_DT = Real( 1e-6 );
}


PidController::PidController( Real param_p, Real param_d, Real param_d2, Real param_i )
  : param_p  ( param_p )
  , param_d  ( param_d )
  , param_d2 ( param_d2 )
  , param_i  ( param_i )

  , param_ad ( REAL_ZERO )
  , ad_err_epsilon ( PID_EPSILON )

  , output_limit        ( REAL_ZERO )
  , integral_term_limit ( REAL_ZERO )

  , derivative_smoothing_time ( REAL_ZERO )

  , integral_error  ( REAL_ZERO )
  , prev_error      ( REAL_ZERO )
  , prev_measurement ( REAL_ZERO )
  , prev_derivative ( REAL_ZERO )

  , last_output ( REAL_ZERO )
  , saturated   ( false )

  , first ( true )
{
}

void PidController::setAdaptiveD( Real ad, Real ad_err_epsilon )
{
  ZgAssert( ad_err_epsilon > REAL_ZERO ); // it is a divisor guard, zero defeats the point

  param_ad = ad;
  this->ad_err_epsilon = ad_err_epsilon;
}

void PidController::setOutputLimit( Real max_abs_output )
{
  ZgAssert( max_abs_output >= REAL_ZERO );

  output_limit = max_abs_output;
}

void PidController::setIntegralTermLimit( Real max_abs_i_term )
{
  ZgAssert( max_abs_i_term >= REAL_ZERO );

  integral_term_limit = max_abs_i_term;
}

void PidController::setDerivativeSmoothingTime( Real tau_seconds )
{
  ZgAssert( tau_seconds >= REAL_ZERO );

  derivative_smoothing_time = tau_seconds;
}

void PidController::reset()
{
  integral_error   = REAL_ZERO;
  prev_error       = REAL_ZERO;
  prev_measurement = REAL_ZERO;
  prev_derivative  = REAL_ZERO;

  last_output = REAL_ZERO;
  saturated   = false;

  first = true;
}

Real PidController::rawOutput( Real cur_error, Real derivative, Real second_derivative, Real adaptive ) const
{
  return
    - param_p  * cur_error
    - param_d  * derivative
    - param_d2 * second_derivative
    - param_i  * integral_error
    - param_ad * adaptive
    ;
}

Real PidController::calc1( Real cur_error, Real dt )
{
  bool const is_first = first;

  // On the very first call there is no previous sample, so there is no derivative either.
  // Reporting one computed against prev_error == 0 would be a large fictitious spike.
  Real raw_derivative = REAL_ZERO;

  if ( !is_first )
    raw_derivative = ( cur_error - prev_error ) / max2( dt, PID_MIN_DT );

  prev_error = cur_error;

  return step( cur_error, raw_derivative, dt, is_first );
}

Real PidController::calc( Real setpoint, Real measurement, Real dt )
{
  Real const cur_error = setpoint - measurement;

  bool const is_first = first;

  // d(error)/dt with the setpoint treated as constant:
  //   error = setpoint - measurement  =>  d(error)/dt = -d(measurement)/dt
  // A setpoint step therefore contributes nothing to the derivative.
  Real raw_derivative = REAL_ZERO;

  if ( !is_first )
    raw_derivative = -( measurement - prev_measurement ) / max2( dt, PID_MIN_DT );

  prev_error      = cur_error;
  prev_measurement = measurement;

  return step( cur_error, raw_derivative, dt, is_first );
}

Real PidController::step( Real cur_error, Real raw_derivative, Real dt, bool is_first )
  {
  ZgAssert( dt > PID_MIN_DT );

  // Clamped rather than trusted: a zero or negative dt from a stalled clock would otherwise
  // turn the derivative into an infinity and poison the integral for good.
  applyMin( dt, PID_MIN_DT );

    first       = false;

  // ------------------------------------------------------------------ derivative
  Real derivative = raw_derivative;

  if ( derivative_smoothing_time > REAL_ZERO && !is_first )
  {
    // One-pole low-pass, exact for a varying dt: alpha = dt / (tau + dt).
    Real const alpha = dt / ( derivative_smoothing_time + dt );

    derivative = prev_derivative + alpha * ( raw_derivative - prev_derivative );
  }

  // ------------------------------------------------------------------ second derivative
  // Per second squared - NOT the raw difference of successive derivatives, which would
  // make param_d2 scale with the loop rate.
  Real second_derivative = REAL_ZERO;

  if ( !is_first )
    second_derivative = ( derivative - prev_derivative ) / dt;

  prev_derivative = derivative;

  // ------------------------------------------------------------------ adaptive damping
  // The relative approach rate, 1/s: how fast the error is closing, measured against how
  // much of it is left. The epsilon guards the division near the target, where the ratio
  // is meaningless and would blow up.
  Real adaptive = REAL_ZERO;

  Real const cur_error_abs = std::abs( cur_error );
  
  if ( cur_error_abs > ad_err_epsilon )
    adaptive = derivative / cur_error_abs;

  // ------------------------------------------------------------------ integral
  // Integrate tentatively, so that the step can be rolled back below if it turns out to
  // push an already-saturated output further into saturation.
  Real const old_integral = integral_error;

  integral_error += cur_error * dt;

  if ( integral_term_limit > REAL_ZERO && std::abs( param_i ) > EPSILON )
  {
    Real const max_state = integral_term_limit / std::abs( param_i );

    toRange( integral_error, -max_state, max_state );
  }

  // ------------------------------------------------------------------ output
  Real out = rawOutput( cur_error, derivative, second_derivative, adaptive );

  saturated = false;

  if ( output_limit > REAL_ZERO && std::abs( out ) > output_limit )
{
    saturated = true;

    Real const clamped = ( out > REAL_ZERO ) ? output_limit : -output_limit;

    // Conditional integration ("integrator clamping"): while the output is pinned at its
    // limit, integrating further cannot move the actuator - it only builds up a charge
    // that is released as an overshoot once the load lets go. So the integration is undone
    // whenever it pushed in the same direction the output is already saturated in.
    //
    // Undoing it rather than freezing the integral outright matters: the integral is still
    // free to shrink, so the controller recovers the instant the error changes sign.
    Real const i_term_delta = -param_i * ( integral_error - old_integral );

    bool const pushing_deeper =
         ( clamped > REAL_ZERO && i_term_delta > REAL_ZERO )
      || ( clamped < REAL_ZERO && i_term_delta < REAL_ZERO );

    if ( pushing_deeper )
  {
      integral_error = old_integral;

      out = rawOutput( cur_error, derivative, second_derivative, adaptive );

      if ( std::abs( out ) > output_limit )
        out = clamped;
    else
        saturated = false;
    }
    else
    {
      out = clamped;
    }
  }

  last_output = out;
  return out;
}

} // namespace zygo
