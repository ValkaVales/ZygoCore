#pragma once

#include <zygo/core/types.h>


namespace zygo {

//#define USE_MY_ARR_PID

class PidController
{
private:
  Real param_p;
  Real param_d;
  Real param_d2;
  Real param_i;

  Real param_ad;
  Real ad_err_epsilon;

  //Real total_err;
  Real integral_error;
  Real prev_error;
  Real prev_diff_error;

  bool first;

#ifdef USE_MY_ARR_PID
  CyclicArray<Real> all_errors;
#endif

public:
  PidController( Real param_p, Real param_d, Real param_d2, Real param_i );

  void setAdaptiveD( Real ad, Real ad_err_epsilon ) { param_ad = ad; this->ad_err_epsilon = ad_err_epsilon; }

  Real calc1( Real cur_error, Real dt );

#ifdef USE_MY_ARR_PID
  Real calc2( Real cur_error );
#endif
};

} // namespace zygo
