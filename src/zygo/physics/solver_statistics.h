#pragma once

// Iteration statistics of an iterative solver (velocity or position).
// Self-contained: counts its own updates, no external clock needed.


namespace zygo {
namespace phys {

struct SolverStatistics
{
  int iterations_count = 0;             // iterations of the last update
  int max_iterations_count = 0;

  int count_of_max_iterations_count = 0;
  long long global_step_when_max_iterations = 0;

  long long total_iterations_count = 0LL;
  long long updates_count = 0LL;        // number of solver runs (one per step or substep)

  void update(); // call once per solver run, with iterations_count already set

  double averageIterationsCount() const;
};

} // namespace phys
} // namespace zygo
