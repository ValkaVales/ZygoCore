#pragma once

// PID controller with derivative, second-derivative and adaptive-damping terms.
//
// Sign convention (unchanged): the output OPPOSES the error, so a positive error produces
// a negative output. Feed the output straight to the actuator.
//
// ------------------------------------------------------------------ units
//
// Every gain works on a quantity expressed per second, so the tuning does NOT depend on
// the loop rate. Halving dt does not change the effective gain of any term:
//
//   param_p   * error                    [error]
//   param_d   * d(error)/dt              [error/s]
//   param_d2  * d2(error)/dt2            [error/s^2]
//   param_i   * integral( error dt )     [error*s]
//   param_ad  * (d(error)/dt) / |error|  [1/s]
//
// This was NOT true of the previous version: it used the raw DIFFERENCE of successive
// derivatives for param_d2, and divided the derivative by dt a second time for param_ad.
// Both gains therefore scaled with the loop rate, so any tuning silently broke the moment
// the timestep changed. To reproduce the old behaviour at a nominal step dt0:
//
//   param_d2_new = param_d2_old * dt0
//   param_ad_new = param_ad_old / dt0
//
// ------------------------------------------------------------------ anti-windup
//
// The integral term is what turns a working controller into a dangerous one: while the
// actuator is saturated the error does not shrink, the integral keeps growing, and when
// the load finally releases the controller slams the output the other way.
//
// Two independent guards, both off by default:
//   setOutputLimit()       - saturates the output AND stops the integral growing while
//                            saturated (conditional integration). This is the guard that
//                            actually matters.
//   setIntegralTermLimit() - a hard cap on the contribution of the I term, for when there
//                            is no meaningful output limit to declare.
//
// For a robot joint, setOutputLimit( max_torque ) is almost always the right call.

#include <zygo/core/types.h>


namespace zygo {

class PidController
{
private:
  Real param_p;
  Real param_d;
  Real param_d2;
  Real param_i;

  Real param_ad;
  Real ad_err_epsilon;

  // ---- optional limits ( <= 0 means "not set" ) ----
  Real output_limit;
  Real integral_term_limit;

  // ---- optional first-order low-pass on the derivative ( <= 0 means "off" ) ----
  Real derivative_smoothing_time; // seconds

  // ---- state ----
  Real integral_error;
  Real prev_error;
  Real prev_measurement;
  Real prev_derivative;

  Real last_output;
  bool saturated;

  bool first;

public:
  PidController( Real param_p, Real param_d, Real param_d2, Real param_i );

  // ------------------------------------------------------------------ tuning
  void setAdaptiveD( Real ad, Real ad_err_epsilon );

  // Saturates the output at +/- max_abs_output and freezes the integral while saturated.
  // Pass 0 to disable. For a joint controller this is the actuator torque limit.
  void setOutputLimit( Real max_abs_output );

  // Caps the contribution of the I term itself (same units as the output).
  // Pass 0 to disable. Independent of, and usually redundant with, setOutputLimit().
  void setIntegralTermLimit( Real max_abs_i_term );

  // First-order low-pass on the derivative, as a time constant in seconds.
  // Differentiating a quantized or noisy measurement amplifies the noise by 1/dt, so a
  // real encoder almost always wants a few milliseconds here. Pass 0 to disable - the
  // default, i.e. the raw difference, exactly as before.
  void setDerivativeSmoothingTime( Real tau_seconds );

  // ------------------------------------------------------------------ use
  // Clears the integral and the derivative history. Call it whenever the loop is
  // interrupted - a new episode, a re-enabled actuator, a large setpoint jump, a robot
  // waking up - so that stale state from the previous life does not leak into the first
  // output of the new one.
  void reset();

  // The error-based entry point (unchanged signature and meaning).
  Real calc1( Real cur_error, Real dt );

  // Setpoint/measurement entry point: identical, except that the derivative is taken from
  // the MEASUREMENT rather than from the error. A step change of the setpoint then produces
  // no derivative spike ("derivative kick"), which for a position-controlled joint is the
  // difference between a smooth move and a bang on every new command.
  Real calc( Real setpoint, Real measurement, Real dt );

  // ------------------------------------------------------------------ diagnostics
  // For tuning and logging: an integral term creeping toward its limit, or an output that
  // is permanently saturated, says far more than the output value alone.
  Real lastOutput()   const { return last_output; }
  Real integralTerm() const { return -param_i * integral_error; }
  bool isSaturated()  const { return saturated; }

private:
  Real step( Real cur_error, Real raw_derivative, Real dt, bool is_first );
  Real rawOutput( Real cur_error, Real derivative, Real second_derivative, Real adaptive ) const;
};

} // namespace zygo
