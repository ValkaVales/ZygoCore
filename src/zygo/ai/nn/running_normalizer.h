#pragma once

// Running mean/variance normalization of a vector, by Welford's online algorithm.
//
// It is the difference between a policy that trains and one that does not.
// The cartpole gets away without it because its four inputs were hand-divided by hand-picked constants (MAX_DISTANCE, MAX_SPEED, ...).
// A robot observation vector cannot be: joint angles are order 1 rad, angular rates order 10 rad/s, IMU accelerations order 10 m/s^2, a contact flag is 0 or 1.
// Feed those raw into a net and the first layer spends its whole training budget learning the scale factors that this class computes exactly, for free, in one pass.
//
// Usage during training:   update( obs ); normalize( obs, obs_n );   then feed obs_n to the net.
// Usage on the robot:      loadFromFile(...); freeze(); normalize( obs, obs_n );
//
// NOTE BEFORE DEPLOYING.
// A normalizer that keeps adapting on the real robot changes the meaning of the policy's inputs while it is flying.

#include <zygo/core/types.h>
#include <vector>


namespace zygo {

class ByteReader;
class ByteWriter;

namespace nn {

class RunningNormalizer
{
private:
  std::vector<Real> mean;
  std::vector<Real> m2;   // sum of squared deviations
  std::vector<Real> inv_std_cache;

  Real count;
  Real clip_sigmas; // hard clamp of the normalized value, in sigmas; <= 0 disables
  bool frozen;
  bool cache_valid;

public:
  explicit RunningNormalizer( int size, Real clip_sigmas = (Real)5.0 );

  inline int size() const { return (int)mean.size(); }
  inline Real samplesCount() const { return count; }

  inline bool isFrozen() const { return frozen; }
  inline void freeze()   { frozen = true;  }
  inline void unfreeze() { frozen = false; }

  void reset();

  void update( Real const* x );            // one sample
  void update( Real const* x, int count ); // a batch of `count` vectors laid out contiguously

  // out may alias x
  void normalize( Real const* x, Real* out );

  Real meanAt  ( int i ) const { return mean[(size_t)i]; }
  Real stdDevAt( int i ) const;

  u32  sizeOfSerialize() const;
  void serialize( ByteWriter& sr ) const;
  bool deserialize( ByteReader& ds );

private:
  void rebuildCache();
};

} // namespace nn
} // namespace zygo
