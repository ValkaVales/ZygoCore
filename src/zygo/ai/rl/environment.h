#pragma once

// The seam between "a thing that can be simulated" and "a thing that can be trained".
//
// Right now the cart-pole and the network are welded into one class, so the training code cannot be
// pointed at anything else. This interface is what lets the very same trainer drive the cart-pole
// today and RobotDogSim tomorrow.
//
// TERMINATED vs TRUNCATED - this distinction is not pedantry, it is a correctness requirement:
//
//   terminated - the episode genuinely ended (the robot fell). V(s') is zero BY DEFINITION, so the
//                bootstrap must be cut off.
//   truncated  - the episode was cut off by a step limit while the state was still perfectly fine.
//                V(s') is NOT zero and must still be bootstrapped, otherwise every time limit
//                teaches the policy that surviving long is somehow punished.
//
// Conflating the two is one of the classic silent RL bugs, and the original cart-pole conflated
// them: DoneType::TRUNCATED was computed and then never used differently from TERMINATED.
//
// Action encoding:
//   discrete   - actionSize() is the number of actions; action[0] holds the chosen index as a Real.
//   continuous - actionSize() is the dimension; action[] holds the raw values, already squashed to
//                whatever range the environment declares.

#include <zygo/core/types.h>


namespace zygo {
namespace rl {

struct StepResult
{
  Real reward;
  bool terminated;
  bool truncated;

  StepResult()
    : reward      ( REAL_ZERO )
    , terminated  ( false )
    , truncated   ( false )
  {}

  inline bool isDone() const { return terminated || truncated; }
};


class IEnvironment
{
public:
  virtual ~IEnvironment() {}

  virtual int obsSize     () const = 0;
  virtual int actionSize  () const = 0;
  virtual bool isContinuous() const = 0;

  // Range of a continuous action, symmetric: [-actionLimit(i), +actionLimit(i)].
  // The trainer squashes the gaussian sample into it. Meaningless for discrete environments.
  virtual Real actionLimit( int i ) const { (void)i; return REAL_ONE; }

  virtual void reset( Real* out_obs ) = 0;

  virtual StepResult step( Real const* action, Real* out_obs ) = 0;

  // Optional: how many environment steps one call to step() advances. Only used for logging.
  virtual Real stepTime() const { return REAL_ZERO; }
};

} // namespace rl
} // namespace zygo
