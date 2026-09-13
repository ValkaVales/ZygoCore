#pragma once

// Storage for one on-policy rollout, plus the advantage computation.
//
// Flat contiguous arrays, sized once at construction, never reallocated during training.
// Deliberately not a vector of per-step structs: the update loop walks one field at a time, and on the robot this buffer will hold tens of thousands of steps.

#include <zygo/core/types.h>
#include <vector>


namespace zygo {
namespace rl {

class RolloutBuffer
{
private:
  int capacity;
  int obs_size;
  int action_size; // 1 for discrete (the index), action dimension for continuous

  int count;

  std::vector<Real> obs;        // [capacity * obs_size]
  std::vector<Real> actions;    // [capacity * action_size]
  std::vector<Real> log_probs;  // log pi(a|s) under the policy that COLLECTED the data
  std::vector<Real> values;
  std::vector<Real> rewards;
  std::vector<Real> advantages;
  std::vector<Real> returns;

  std::vector<u8> terminated;
  std::vector<u8> truncated;

public:
  RolloutBuffer( int capacity, int obs_size, int action_size );

  inline int size    () const { return count; }
  inline int maxSize () const { return capacity; }
  inline bool isFull () const { return count >= capacity; }

  inline int obsSize   () const { return obs_size; }
  inline int actionSize() const { return action_size; }

  void clear() { count = 0; }

  void add(
      Real const* obs_in
    , Real const* action_in
    , Real        log_prob
    , Real        value
    , Real        reward
    , bool        terminated_in
    , bool        truncated_in
    );

  Real const* obsAt   ( int i ) const { return &obs    [(size_t)i * (size_t)obs_size   ]; }
  Real const* actionAt( int i ) const { return &actions[(size_t)i * (size_t)action_size]; }

  inline Real logProbAt  ( int i ) const { return log_probs [(size_t)i]; }
  inline Real valueAt    ( int i ) const { return values    [(size_t)i]; }
  inline Real rewardAt   ( int i ) const { return rewards   [(size_t)i]; }
  inline Real advantageAt( int i ) const { return advantages[(size_t)i]; }
  inline Real returnAt   ( int i ) const { return returns   [(size_t)i]; }

  inline bool isTerminated( int i ) const { return terminated[(size_t)i] != 0; }
  inline bool isTruncated ( int i ) const { return truncated [(size_t)i] != 0; }

  // Generalized Advantage Estimation.
  //   last_value - V(s') after the final stored step; pass 0 only if that step TERMINATED.
  //
  // Note how terminated and truncated are used differently: a terminated step zeroes both the bootstrap and the GAE recursion,
  // a truncated one zeroes only the recursion (the trajectory ends there, but the value of the state it ended in is real).
  void computeGae( Real last_value, Real gamma, Real gae_lambda );

  void normalizeAdvantages();

  Real meanReward() const;
};

} // namespace rl
} // namespace zygo
