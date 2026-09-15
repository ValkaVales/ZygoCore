#pragma once

#include <zygo/core/types.h>
#include <zygo/math/vector/vec3.h>


namespace zygo {

// One generator PER THREAD: the state is thread_local.
//
// Every thread starts from the same built-in state, so two threads that never call seed() produce the SAME sequence.
// A thread that needs its own stream (a worker collecting a rollout, see rl/parallel_ppo.h) calls seed() with its own value first.
// Within one thread the behaviour is exactly the old global generator: same seed, same numbers.
//
// The state is defined HERE, inline, with a constant initializer - not in random.cpp. That is what keeps thread_local cheap:
// a definition the compiler cannot see might need dynamic initialization, so every access in another .cpp goes through
// an init-check wrapper. Measured with g++ -O2, Random::next(): ~1.1 ns as a plain global, ~6 ns thread_local defined in the .cpp,
// ~1.1 ns thread_local defined inline here - no difference from the global.
class Random
{
private:
  static inline thread_local u64 s[4] = { 0x9E3779B97F4A7C15ULL, 0xBF58476D1CE4E5B9ULL, 0x94D049BB133111EBULL, 0x2545F4914F6CDD1DULL };
  static inline thread_local bool has_gauss_next = false;
  static inline thread_local Real gauss_next = REAL_ZERO;

  static inline u64 rotl( u64 v, int k ) { return (v << k) | (v >> (64 - k)); }

public:
  static void seed( u64 seed_value );

  static inline u64 next()
  {
    const u64 result = rotl( s[0] + s[3], 23 ) + s[0];
    const u64 t = s[1] << 17;

    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = rotl( s[3], 45 );

    return result;
  }

  // равномерное в [0, 1): старшие 53 бита -> вся мантисса double
  static inline Real rand01()
  {
    return (Real)(next() >> 11) * (Real)(1.0 / 9007199254740992.0); // 2^-53
  }

  static inline Real rand( Real from, Real to )
  {
    return from + (to - from) * rand01();
  }

  // целое в [0, to) без модуло-биаса (метод Лемира, упрощённый)
  static inline u32 randInt( u32 to )
  {
    return (u32)(((u64)(u32)(next() >> 32) * to) >> 32);
  }

  static Real  gauss( Real mean, Real sigma );
  static Vector3 gauss( Vector3 const& mean, Real sigma );

  static Real oldGauss( Real mean, Real sigma ); // slow
};

} // namespace zygo
