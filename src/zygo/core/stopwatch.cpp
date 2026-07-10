#include "stopwatch.h"


namespace zygo
{

Stopwatch::Stopwatch()
{
  restart();
}

void Stopwatch::restart()
{
  start_time     = Clock::now();
  pause_start    = start_time;
  time_in_pauses = 0.0;
  is_paused      = false;
}

void Stopwatch::pause()
{
  if ( is_paused )
    return;

  is_paused   = true;
  pause_start = Clock::now();
}

void Stopwatch::resume()
{
  if ( !is_paused )
    return;

  is_paused = false;
  time_in_pauses += std::chrono::duration<double>( Clock::now() - pause_start ).count();
}

double Stopwatch::elapsed() const
{
  Clock::time_point now = Clock::now();

  double total = std::chrono::duration<double>( now - start_time ).count();

  if ( is_paused )
    total -= std::chrono::duration<double>( now - pause_start ).count(); // current, still open pause

  return total - time_in_pauses;
}

} // namespace zygo
