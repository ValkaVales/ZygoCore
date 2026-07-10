#pragma once

#include <chrono>


namespace zygo {

// Real-time stopwatch based on steady_clock (monotonic: never jumps on NTP sync or DST change).
// Supports pause; elapsed() excludes pauses.
class Stopwatch
{
private:
  using Clock = std::chrono::steady_clock;

  Clock::time_point start_time;
  Clock::time_point pause_start;

  double time_in_pauses; // seconds
  bool   is_paused;

public:
  Stopwatch();

  void restart();

  void pause ();
  void resume();

  bool isPaused() const { return is_paused; }

  double elapsed() const; // seconds, excluding pauses
};

} // namespace zygo
