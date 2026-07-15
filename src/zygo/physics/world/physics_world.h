#pragma once

// The physics world: holds references to ArticulatedBody-s and the terrain,
// applies gravity, detects foot-vs-ground contacts and runs the shared
// velocity solve (joints + contacts together) with substepping.
//
// Bodies and the terrain are NOT owned by the world (never deleted by it) -
// the world stores raw pointers, the caller guarantees the lifetimes.

#include <zygo/physics/body/articulated_body.h>
#include <zygo/physics/world/contact_point.h>
#include <zygo/physics/terrain/terrain.h>
#include <zygo/physics/solver_statistics.h>
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

  Vector3 gravity;
  double  friction_mu;
  double  restitution_coeff;
  int     velocity_iterations;
  int     position_iterations;
  int     substeps;

  SolverStatistics velocity_solver_statistics;
  SolverStatistics position_solver_statistics;

  long long total_calc_time = 0LL; // microseconds

public:
  explicit PhysicsWorld( ITerrain * terrain );

  // Draws the registered contact spheres. Bodies and the terrain draw themselves.
  void draw( IPhysicsDrawer const& drawer ) const;

  //
  void setGravity( Vector3 const & g ) { gravity = g; }
  void setTerrain( ITerrain * t )      { terrain = t; }

  void setFrictionMu      ( double mu ) { friction_mu = mu; }
  void setRestitutionCoeff( double e )  { restitution_coeff = e; }

  void setVelocityIterations( int n ) { velocity_iterations = n; }
  void setPositionIterations( int n ) { position_iterations = n; }
  void setSubsteps          ( int n ) { substeps = n; }

  //
  Vector3 const & getGravity() const { return gravity; }

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

private:
  void subStep( double dt );

  bool solveContactPositionsOnce();

  void detectContacts();
  void warmStartContacts();

  bool solveContactsCollisionOnce( double dt ); // returns true, if still has error
  bool solveContactsFrictionOnce();             // returns true, if still has error
};

} // namespace phys
} // namespace zygo
