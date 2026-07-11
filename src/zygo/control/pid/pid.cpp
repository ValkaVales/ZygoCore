#include "pid.h"
#include <zygo/math/common/consts.h>
#include <zygo/core/assert.h>
#include <cmath> // abs


namespace zygo {

#ifdef USE_MY_ARR_PID
const int ERRORS_COUNT = 4000;
#endif


PidController::PidController( Real param_p, Real param_d, Real param_d2, Real param_i )
  : param_p  ( param_p )
  , param_d  ( param_d )
  , param_d2 ( param_d2 )
  , param_i  ( param_i )

  , param_ad ( REAL_ZERO )
  , ad_err_epsilon ( PID_EPSILON )

  , integral_error  ( REAL_ZERO )
  , prev_error      ( REAL_ZERO )
  , prev_derivative ( REAL_ZERO )

  , first ( true )

#ifdef USE_MY_ARR_PID
  , all_errors ( ERRORS_COUNT )
#endif
{
}

Real PidController::calc1( Real cur_error, Real dt )
{
  ZgAssert( dt > EPSILON );

  Real derivative       = (cur_error  - prev_error) / dt;
  Real delta_derivative = derivative - prev_derivative;

  prev_error      = cur_error;
  prev_derivative = derivative;

  if ( first )
  {
    first       = false;
    derivative  = REAL_ZERO;
    delta_derivative = REAL_ZERO;
    prev_derivative  = REAL_ZERO; // otherwise the garbage first derivative (computed against prev_error == 0) leaks into delta_derivative on the SECOND step
  }

  Real speed_to_goal = derivative / dt;
  Real adaptive_diff_error = REAL_ZERO;
  Real cur_error_abs = std::abs( cur_error );
  //applyMin( cur_error_abs, ad_err_epsilon );
  
  if ( cur_error_abs > ad_err_epsilon )
  {
    //adaptive_diff_error = sqr( speed_to_goal ) / cur_error_abs;
    //if ( derivative < REAL_ZERO )
    //  adaptive_diff_error = -adaptive_diff_error;

    adaptive_diff_error = speed_to_goal / cur_error_abs;
  }

  integral_error += cur_error * dt;

  return 
    - param_p  * cur_error
    - param_d  * derivative
    - param_d2 * delta_derivative
    - param_i  * integral_error
    //+ param_ad * adaptive_diff_error
    - param_ad * adaptive_diff_error
    ;
}

#ifdef USE_MY_ARR_PID
Real PidController::calc2( Real cur_error )
{
  all_errors.push( cur_error );

  Real res = REAL_ZERO;

  int cnt = all_errors.curCount();
  for ( int i = 0; i < cnt; ++i )
  {
    Real err = all_errors.get( i );

    if ( i == cnt - 1 )
      res += err * (param_p + param_d);
    else
    if ( i == cnt - 2 )
      res -= err * param_d;
    else
      res += err * param_i;
  }

  return -res;
}
#endif

} // namespace zygo
