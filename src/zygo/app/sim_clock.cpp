#include "sim_clock.h"

namespace zygo
{

SimClock::SimClock( double dt_, int ticks_per_frame_ )
  : dt              ( dt_ )
  , sim_time        ( 0.0 )
  , step_index      ( 0 )
  , pending_steps   ( 0 )
  , ticks_per_frame ( ticks_per_frame_ > 0 ? ticks_per_frame_ : 1 )
{
  wall.restart();
  wall.pause(); // start paused
}

void SimClock::pause()
{
  wall.pause();
}

void SimClock::resume()
{
  pending_steps = 0;
  wall.resume();
}

void SimClock::togglePause()
{
  if ( wall.isPaused() )
    resume();
  else
    pause();
}

void SimClock::requestSteps( int count )
{
  if ( count > 0 )
    pending_steps += count; // pressing the step key N times queues N steps
}

void SimClock::pauseOrStepOnce()
{
  if ( !wall.isPaused() )
    pause();
  else
    requestSteps( 1 );
}

void SimClock::beginStep()
{
  if ( pending_steps > 0 )
    --pending_steps;

  ++step_index;
  sim_time += dt;
}

void SimClock::setTicksPerFrame( int count )
{
  ticks_per_frame = count > 0 ? count : 1;
}

} // namespace zygo
