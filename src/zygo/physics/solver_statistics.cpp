#include "solver_statistics.h"


namespace zygo {
namespace phys {

void SolverStatistics::update()
{
  ++updates_count;
  total_iterations_count += iterations_count;

  if ( iterations_count > max_iterations_count )
  {
    count_of_max_iterations_count = 1;
    max_iterations_count = iterations_count;
    global_step_when_max_iterations = updates_count;
  } else
  if ( iterations_count == max_iterations_count )
  {
    ++count_of_max_iterations_count;
  }
}

double SolverStatistics::averageIterationsCount() const
{
  if ( updates_count == 0 )
    return 0.0;

  return (double)total_iterations_count / (double)updates_count;
}

} // namespace phys
} // namespace zygo
