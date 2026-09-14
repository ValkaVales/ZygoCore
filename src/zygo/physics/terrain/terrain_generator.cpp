#include "terrain_generator.h"
#include <zygo/math/common/consts.h>
#include <zygo/math/common/scalar.h>
#include <cmath>
#include <vector>


namespace zygo {
namespace phys {

namespace
{
  // A private generator, deliberately not zygo::Random.
  //
  // Terrain generation must be reproducible from its seed ALONE.
  // Drawing from the global RNG would couple the terrain to however many random numbers the policy happened to consume
  // before the reset - so the "same" seed would produce a different world on the second run.
  class SplitMix64
  {
  private:
    u64 state;

  public:
    explicit SplitMix64( u64 seed ) : state( seed + 0x9E3779B97F4A7C15ull ) {}

    u64 nextU64()
    {
      u64 z = ( state += 0x9E3779B97F4A7C15ull );
      z = ( z ^ ( z >> 30 ) ) * 0xBF58476D1CE4E5B9ull;
      z = ( z ^ ( z >> 27 ) ) * 0x94D049BB133111EBull;
      return z ^ ( z >> 31 );
    }

    Real next01()
    {
      return (Real)( ( nextU64() >> 11 ) * ( 1.0 / 9007199254740992.0 ) );
    }

    Real range( Real a, Real b )
    {
      return a + ( b - a ) * next01();
    }
  };

  struct Wave
  {
    Real kx;
    Real ky;
    Real phase;
    Real amplitude;
  };

  // Smooth 0 -> 1 over [0,1], zero derivative at both ends.
  inline Real smooth01( Real t )
  {
    if ( t <= REAL_ZERO ) return REAL_ZERO;
    if ( t >= REAL_ONE  ) return REAL_ONE;

    return t * t * ( (Real)3 - (Real)2 * t );
  }
}


TerrainParams::TerrainParams()
  : seed                ( 1 )
  , roughness_amplitude ( REAL_ZERO )
  , wavelength_min      ( (Real)0.25 )
  , wavelength_max      ( (Real)0.90 )
  , waves_count         ( 4 )
  , slope_x             ( REAL_ZERO )
  , slope_y             ( REAL_ZERO )
  , step_height         ( REAL_ZERO )
  , step_size           ( (Real)0.40 )
  , bumps_count         ( 0 )
  , bump_height         ( (Real)0.05 )
  , bump_radius         ( (Real)0.12 )
  , flat_start_radius   ( (Real)0.60 )
  , flat_start_blend    ( REAL_ZERO )
{
}

TerrainParams TerrainParams::byDifficulty( Real difficulty01, u64 seed )
{
  Real d = difficulty01;
  if ( d < REAL_ZERO ) d = REAL_ZERO;
  if ( d > REAL_ONE  ) d = REAL_ONE;

  TerrainParams p;
  p.seed = seed;

  // Undulation comes in first and grows all the way: it is the difficulty the robot can always make some progress against.
  p.roughness_amplitude = (Real)0.13 * d;

  // A slope only starts to matter once the ground is already uneven, so it comes in later.
  //
  // Kept small on purpose.
  // Slope is a CONSTANT gradient, so over a 24 m field even a gentle one moves the far corners by more than a metre
  // - it dominates every statistic of the field while changing almost nothing under the robot.
  // 
  // A big number here makes difficulty look like it is doing a lot when it is not.
  const Real slope_k = smooth01( ( d - (Real)0.3 ) / (Real)0.7 );
  p.slope_x = (Real)0.06 * slope_k;

  // Steps and bumps are the hard part.
  // They come in at 0.35 rather than 0.5, because they are the only features whose ARRIVAL is visible:
  // raising an amplitude from 3 cm to 8 cm is a change you have to look for, while a plateau edge appearing is a change you cannot miss.
  const Real hard_k = smooth01( ( d - (Real)0.35 ) / (Real)0.65 );
  p.step_height = (Real)0.06 * hard_k;
  p.step_size   = (Real)0.45;

  p.bumps_count = (int)( (Real)60 * hard_k );
  p.bump_height = (Real)0.09 * hard_k;

  return p;
}


char const* terrainTypeName( TerrainType type )
{
  switch ( type )
  {
  case TerrainType::FLAT        : return "flat";
  case TerrainType::ROUGH       : return "rough";
  case TerrainType::SLOPE_UP    : return "slope up";
  case TerrainType::SLOPE_DOWN  : return "slope down";
  case TerrainType::SLOPE_LEFT  : return "slope left";
  case TerrainType::SLOPE_RIGHT : return "slope right";
  case TerrainType::STEPS       : return "steps";
  case TerrainType::BUMPS       : return "bumps";
  case TerrainType::MIXED       : return "mixed";
  default:
    return "?";
  }
}

bool isSlopeType( TerrainType type )
{
  return type == TerrainType::SLOPE_UP
      || type == TerrainType::SLOPE_DOWN
      || type == TerrainType::SLOPE_LEFT
      || type == TerrainType::SLOPE_RIGHT;
}

Real slopeAngleDegrees( Real difficulty01 )
{
  Real d = difficulty01;
  if ( d < REAL_ZERO ) d = REAL_ZERO;
  if ( d > REAL_ONE  ) d = REAL_ONE;

  return (Real)30.0 * d;
}

TerrainParams TerrainParams::byType( TerrainType type, Real difficulty01, u64 seed )
{
  Real d = difficulty01;
  if ( d < REAL_ZERO ) d = REAL_ZERO;
  if ( d > REAL_ONE  ) d = REAL_ONE;

  if ( type == TerrainType::MIXED )
    return byDifficulty( d, seed );

  TerrainParams p;
  p.seed = seed;

  switch ( type )
  {
  case TerrainType::FLAT:
    break;

  case TerrainType::ROUGH:
    p.roughness_amplitude = (Real)0.12 * d;
    break;

  case TerrainType::SLOPE_UP:
  case TerrainType::SLOPE_DOWN:
  case TerrainType::SLOPE_LEFT:
  case TerrainType::SLOPE_RIGHT:
  {
    // Pure incline, expressed as an ANGLE rather than a gradient: a gradient of 0.25 tells you
    // nothing, 14 degrees tells you everything. The stored value is still the gradient, which is
    // what the height function needs.
    const Real gradient = std::tan( slopeAngleDegrees( d ) * (Real)( PI / 180.0 ) );

    // The robot spawns facing +X, so +Y is its left.
    // "falls away to the left" means the height DECREASES towards +Y.
    switch ( type )
    {
    case TerrainType::SLOPE_UP:    p.slope_x =  gradient; break;
    case TerrainType::SLOPE_DOWN:  p.slope_x = -gradient; break;
    case TerrainType::SLOPE_LEFT:  p.slope_y = -gradient; break;
    default:                       p.slope_y =  gradient; break;
    }

    // Spawn disc: 0.45 m keeps all four feet (at a 0.30 m radius) on the flat part, and a short
    // 0.2 m blend puts the robot on the true incline after two thirds of a metre. It was tried at
    // 0.25 m: that put the feet INSIDE the blend, on a bump, and every episode began by falling
    // over within half a second, adaptation or not. The default 0.6 + 0.6 blend is the opposite
    // problem - a long convex ramp on which the local gradient under the trunk is never the slope.
    p.flat_start_radius = (Real)0.45;
    p.flat_start_blend  = (Real)0.20;
    break;
  }

  case TerrainType::STEPS:
    p.step_height = (Real)0.07 * d;
    p.step_size   = (Real)0.45;
    break;

  case TerrainType::BUMPS:
    p.bumps_count = (int)( (Real)60 * d );
    p.bump_height = (Real)0.10 * d;
    p.bump_radius = (Real)0.13;
    break;

  default:
    break;
  }

  return p;
}


void generateTerrain( HeightField& field, TerrainParams const& params )
{
  SplitMix64 rng( params.seed );

  // --- smooth undulation: a sum of sine waves with random directions and phases -----------------
  std::vector<Wave> waves;

  const int n_waves = ( params.roughness_amplitude > REAL_ZERO ) ? params.waves_count : 0;

  for ( int i = 0; i < n_waves; ++i )
  {
    const Real wavelength = rng.range( params.wavelength_min, params.wavelength_max );
    const Real direction  = rng.range( REAL_ZERO, (Real)( 2.0 * PI ) );
    const Real k          = (Real)( 2.0 * PI ) / wavelength;

    Wave w;
    w.kx        = k * std::cos( direction );
    w.ky        = k * std::sin( direction );
    w.phase     = rng.range( REAL_ZERO, (Real)( 2.0 * PI ) );
    w.amplitude = params.roughness_amplitude / (Real)n_waves;

    waves.push_back( w );
  }

  // --- plateau heights: one random level per step cell ------------------------------------------
  // Sampled through a hash of the cell index rather than a table, so the field can be any size and the result still depends only on the seed.
  const bool has_steps = ( params.step_height > REAL_ZERO && params.step_size > REAL_ZERO );

  // --- isolated bumps ---------------------------------------------------------------------------
  struct Bump { Real x, y, h, r; };
  std::vector<Bump> bumps;

  if ( params.bumps_count > 0 && params.bump_height > REAL_ZERO )
  {
    // Bumps are placed over the field's actual extent.
    const Real x0 = (Real)0.0;
    (void)x0;

    for ( int i = 0; i < params.bumps_count; ++i )
    {
      Bump b;
      b.x = rng.range( (Real)-1, (Real)1 );  // normalized, mapped below
      b.y = rng.range( (Real)-1, (Real)1 );
      b.h = params.bump_height * rng.range( (Real)0.5, (Real)1.0 );
      b.r = params.bump_radius * rng.range( (Real)0.7, (Real)1.4 );
      bumps.push_back( b );
    }
  }

  const Real span_x = (Real)( field.sizeX() - 1 ) * (Real)field.cellSize();
  const Real span_y = (Real)( field.sizeY() - 1 ) * (Real)field.cellSize();

  field.fillFromFunction( [&]( double x, double y ) -> double
  {
    Real h = REAL_ZERO;

    for ( size_t i = 0; i < waves.size(); ++i )
      h += waves[i].amplitude * std::sin( waves[i].kx * (Real)x + waves[i].ky * (Real)y + waves[i].phase );

    h += params.slope_x * (Real)x + params.slope_y * (Real)y;

    if ( has_steps )
    {
      const long long ix = (long long)std::floor( (Real)x / params.step_size );
      const long long iy = (long long)std::floor( (Real)y / params.step_size );

      // Hash the plateau index; the seed goes in so steps move with the rest of the terrain.
      u64 hh = (u64)( ix * 73856093LL ) ^ (u64)( iy * 19349663LL ) ^ params.seed;
      SplitMix64 cell_rng( hh );

      h += params.step_height * cell_rng.range( (Real)-1, (Real)1 );
    }

    for ( size_t i = 0; i < bumps.size(); ++i )
    {
      const Real bx = bumps[i].x * span_x * (Real)0.5;
      const Real by = bumps[i].y * span_y * (Real)0.5;

      const Real dx = (Real)x - bx;
      const Real dy = (Real)y - by;

      const Real d2 = dx * dx + dy * dy;
      const Real r2 = bumps[i].r * bumps[i].r;

      if ( d2 < r2 )
      {
        // A smooth dome, so the sides are climbable rather than vertical.
        const Real t = REAL_ONE - d2 / r2;
        h += bumps[i].h * t * t;
      }
    }

    // Flatten the spawn area and blend it into the terrain over the same radius again.
    if ( params.flat_start_radius > REAL_ZERO )
    {
      const Real d = std::sqrt( (Real)( x * x + y * y ) );
      const Real blend = ( params.flat_start_blend > REAL_ZERO ) ? params.flat_start_blend : params.flat_start_radius;
      const Real k = smooth01( ( d - params.flat_start_radius ) / blend );

      h *= k;
    }

    return (double)h;
  } );
}


HeightField makeTerrain( Real size_x, Real size_y, Real cell, TerrainParams const& params )
{
  const int nx = (int)( size_x / cell ) + 1;
  const int ny = (int)( size_y / cell ) + 1;

  HeightField field( -0.5 * (double)size_x, -0.5 * (double)size_y, (double)cell, nx, ny, 0.0 );

  generateTerrain( field, params );

  return field;
}

} // namespace phys
} // namespace zygo
