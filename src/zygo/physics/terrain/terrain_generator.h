#pragma once

// Procedural generation of HeightField terrain.
//
// Written for locomotion training, so the knobs are the ones that actually change how hard the ground is to walk on,
// and every one of them is a single number that can be turned up over the course of a training run:
//
//   roughness  - amplitude of smooth undulation. The basic "not quite flat" difficulty.
//   slope      - constant incline. Costs the policy energy and changes the pitch it must hold.
//   steps      - discrete plateaus. Qualitatively harder than undulation: a foot can land on an edge,
//                and the height under a foot can change by more than a step height between one footfall and the next.
//   bumps      - isolated obstacles. Rare, local, and the main reason blind locomotion has to learn to recover rather than to plan.
//
// Everything is driven from an explicit seed, so a terrain is exactly reproducible - which an episode reset and an A/B comparison of two policies both depend on.
//
// The wavelengths deliberately start around a quarter of a metre. Undulation much shorter than the foot spacing is not terrain,
// it is noise: the feet average over it and the policy learns nothing.
// 
// Undulation much longer than the robot is not terrain either, it is a slope that changes slowly.

#include <zygo/physics/terrain/terrain.h>
#include <zygo/core/types.h>


namespace zygo {
namespace phys {

// Named terrain families.
//
// byDifficulty() blends all of them together, which is what a training curriculum wants but a poor way to find out WHICH feature a policy cannot handle.
// These let you isolate one at a time: a policy that walks over ROUGH and dies on STEPS is telling you something specific.
enum class TerrainType
{
  FLAT,     // the reference. If the robot falls here, nothing else is worth measuring
  ROUGH,    // smooth undulation only - the basic "not quite flat"

  // Constant inclines, no undulation at all: one clean question each.
  // Directions are in the robot's own frame at spawn - it starts facing +X, so +Y is its left.
  SLOPE_UP,    // uphill ahead      - the robot must push harder and pitch nose-up
  SLOPE_DOWN,  // downhill ahead    - a different problem entirely: it must brake, not push
  SLOPE_LEFT,  // ground falls away to the left  - a roll disturbance, constant in one direction
  SLOPE_RIGHT, // ground falls away to the right

  STEPS,    // discrete plateaus: the height under a foot can jump between one footfall and the next
  BUMPS,    // isolated obstacles on otherwise flat ground
  MIXED,    // everything at once, scaled by difficulty - what training uses

  COUNT
};

char const* terrainTypeName( TerrainType type );

bool isSlopeType( TerrainType type );

// The incline the difficulty knob maps to, in degrees, for the four SLOPE_* types.
// 0 -> flat, 1 -> 30 degrees, which is far past what a blind quadruped can manage and is meant to be:
// the point of the test is to find where it stops working, not to stay inside the easy part.
Real slopeAngleDegrees( Real difficulty01 );


struct TerrainParams
{
  u64 seed;

  // --- smooth undulation ---
  Real roughness_amplitude; // meters, peak-to-zero of the summed waves
  Real wavelength_min;      // meters
  Real wavelength_max;      // meters
  int  waves_count;

  // --- constant incline ---
  Real slope_x;             // meters of rise per meter along +X
  Real slope_y;

  // --- discrete plateaus ---
  Real step_height;         // meters; 0 disables
  Real step_size;           // meters, side of one plateau

  // --- isolated bumps ---
  int  bumps_count;
  Real bump_height;         // meters
  Real bump_radius;         // meters

  // A flat disc around the origin, where the robot is spawned.
  // Without it the very first contact of an episode can be a step edge, and the reset pose is no longer the pose the robot was designed to stand in.
  Real flat_start_radius;   // meters; 0 disables
  Real flat_start_blend;    // meters, width of the transition from flat to terrain; <= 0 means = radius

  TerrainParams();

  // One difficulty knob, 0 = flat, 1 = as rough as the presets go.
  // Keeps the relative proportion of the individual features fixed, which is what a curriculum wants: one number to raise, not six to re-balance.
  static TerrainParams byDifficulty( Real difficulty01, u64 seed );

  // One family, scaled by the same 0..1 knob.
  static TerrainParams byType( TerrainType type, Real difficulty01, u64 seed );
};


// Fills the height field from the parameters. The field's own origin/cell/size are respected.
void generateTerrain( HeightField& field, TerrainParams const& params );


// Convenience: allocates a field centred on the origin and fills it.
//   size_x, size_y - extent in meters
//   cell           - cell size in meters (0.05 is a good default: finer than a foot, coarse enough that a 20x20 m field is a few hundred thousand nodes)
HeightField makeTerrain( Real size_x, Real size_y, Real cell, TerrainParams const& params );

} // namespace phys
} // namespace zygo
