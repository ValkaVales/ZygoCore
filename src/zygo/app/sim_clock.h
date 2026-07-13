#pragma once

#include <zygo/core/stopwatch.h>

namespace zygo
{

// =================================================================== SimClock
// Simulation clock and stepping control.
// Owns: fixed dt, sim time, step counter, pause state, step-by-step debugging mode, and a stopwatch-clock stopwatch that ignores pauses.
//
// Typical GLUT idle/loop usage:
//
//   for ( int i = 0; i < clock.ticksPerFrame(); ++i )
//   {
//     if ( !clock.stepAllowed() )
//       break;
//
//     clock.beginStep();
//     world.step( clock.getDT() );
//   }
//
class SimClock
{
private:
  double    dt;
  double    sim_time;
  long long step_index;

  int pending_steps;   // steps allowed while paused (step-by-step debugging)
  int ticks_per_frame; // simulation steps per one rendered frame

  Stopwatch stopwatch; // pause state lives here (single source of truth)

public:
  explicit SimClock( double dt_, int ticks_per_frame_ = 1 );

  // --- pause & step-by-step mode ---
  void pause      ();
  void resume     ();
  void togglePause();

  void requestSteps( int count = 1 ); // allow N steps without leaving pause
  void pauseOrStepOnce();             // was: Globals::oneStep()

  // --- main loop ---
  bool stepAllowed() const { return !stopwatch.isPaused() || pending_steps > 0; }
  void beginStep();                   // was: Globals::onIterationStart()

  // --- queries ---
  bool      isPaused () const { return stopwatch.isPaused(); }
  double    getDT    () const { return dt        ; }
  double    simTime  () const { return sim_time  ; } // was: allDtsSum()
  long long stepIndex() const { return step_index; } // was: curIteration()

  double stopwatchTime() const { return stopwatch.elapsed(); } // was: timeSinceAppStart_withoutPauses()

  int  ticksPerFrame() const { return ticks_per_frame; }
  void setTicksPerFrame( int count );
};

} // namespace zygo
