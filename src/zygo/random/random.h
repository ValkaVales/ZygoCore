#pragma once

#include <zygo/core/types.h>
#include <zygo/math/vector/vec3.h>


namespace zygo {

class Random // NOT multi-threaded
{
private:
  static u64 s[4];
  static bool has_gauss_next;
  static Real gauss_next;

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
