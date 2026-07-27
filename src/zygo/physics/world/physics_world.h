#pragma once

// The physics world: holds references to ArticulatedBody-s and the terrain, applies gravity,
// detects foot-vs-ground contacts and runs the shared velocity solve (joints + contacts together) with substepping.
//
// Bodies and the terrain are NOT owned by the world (never deleted by it) - the world stores raw pointers, the caller guarantees the lifetimes.

#include <zygo/physics/body/articulated_body.h>
#include <zygo/physics/world/contact_point.h>
#include <zygo/physics/world/world_state.h>
#include <zygo/physics/terrain/terrain.h>
#include <zygo/physics/solver_statistics.h>
#include <zygo/physics/solver_settings.h>
#include <zygo/math/vector/vec3.h>
#include <vector>


namespace zygo {
namespace phys {

class PhysicsWorld
{
private:
  std::vector<ArticulatedBody*> objects; // not owned
  ITerrain * terrain;                    // not owned

  std::vector<ContactSphere> contact_spheres;
  std::vector<ContactPoint>  contacts;   // rebuilt every substep

  SolverStatistics velocity_solver_statistics;
  SolverStatistics position_solver_statistics;

  long long total_calc_time = 0LL; // microseconds

public:
  // All runtime tuning: gravity, stepping, joint/contact solver parameters.
  // Tune directly: world.settings.gravity.enabled = true;
  SolverSettings settings;

  explicit PhysicsWorld( ITerrain * terrain );

  // Draws the registered contact spheres. Bodies and the terrain draw themselves.
  void draw( IPhysicsDrawer const& drawer ) const;

  // Convenience wrappers over `settings` (brevity at the call sites).
  void setGravity( Vector3 const & g ) { settings.gravity.g = g; settings.gravity.enabled = true; }
  void setGravityEnabled( bool on )    { settings.gravity.enabled = on; }
  void setTerrain( ITerrain * t )      { terrain = t; }

  void setFrictionMu      ( double mu ) { settings.contacts.friction_mu = mu; }
  void setRestitutionCoeff( double e )  { settings.contacts.restitution = e; }

  void setVelocityIterations( int n ) { settings.step.max_velocity_iterations = n; }
  void setPositionIterations( int n ) { settings.step.position_iterations = n; }
  void setSubsteps          ( int n ) { settings.step.substeps = n; }

  //
  Vector3 const & getGravity() const { return settings.gravity.g; }
  bool isGravityEnabled()      const { return settings.gravity.enabled; }

  void addObject( ArticulatedBody & obj );

  // center_world_mm - the current world position of the sphere center, in millimeters
  // (construction units, see physics.h); the world remembers it relative to the body
  // center of mass once, at registration time.
  void addContactSphere( RigidBody & body, Vector3 center_world_mm, double radius_mm );

  //
  void processTick( double dt );

  // Diagnostics.
  int activeContactsCount() const { return (int)contacts.size(); }

  SolverStatistics const & velocitySolverStatistics() const { return velocity_solver_statistics; }
  SolverStatistics const & positionSolverStatistics() const { return position_solver_statistics; }

  inline long long totalCalcTime() const { return total_calc_time; }

  // ------------------------------------------------------------------ state snapshot
  //
  // Cheap save / exact restore of the whole simulation - see world_state.h.
  // Reuse one WorldState across calls: after the first save the vectors keep their capacity and no allocation happens at all.
  //
  //   WorldState start;
  //   world.saveState( start );          // once, after building the scene
  //   ...
  //   world.restoreState( start );       // every episode reset
  //
  // restoreState() returns false and changes nothing if the snapshot does not match the world it is applied to
  // (bodies, joints, assemblies or contact spheres added or removed since it was taken).
  void saveState( WorldState & out ) const;
  bool restoreState( WorldState const & in );

private:
  void subStep( double dt );

  bool solveContactPositionsOnce();

  // speculative_dt > 0 enables the velocity-derived query margin (see ContactSettings::speculative_contacts).
  // The position phase passes 0: by then the bodies have already been integrated and only real overlaps are of interest.
  void detectContacts( double speculative_dt );

  void warmStartContacts();

  bool solveContactsCollisionOnce( double dt ); // returns true, if still has error
  bool solveContactsFrictionOnce();             // returns true, if still has error
};

} // namespace phys
} // namespace zygo
