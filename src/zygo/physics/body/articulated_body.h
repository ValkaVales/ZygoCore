#pragma once

// A body assembly: rigid bodies connected by hinge joints (e.g. a robot).
// Owns its bodies and joints. Registered in a PhysicsWorld, which drives the granular step phases.

#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/joint/hinge_joint.h>
#include <zygo/physics/solver_statistics.h>
#include <memory>
#include <vector>


namespace zygo {
namespace phys {

class PhysicsWorld;


class ArticulatedBody
{
private:
  bool initialized = false;
  PhysicsWorld * phys_world = nullptr;

  std::vector<std::unique_ptr<RigidBody>>  objects;
  std::vector<std::unique_ptr<HingeJoint>> joints;

  Vector3 initial_total_center_of_mass;
  Vector3 cur_total_center_of_mass;

  bool   is_sleeping  = false;
  double idle_time    = 0.0; // how long every body has been under the sleep thresholds

  SolverStatistics velocity_solver_statistics;
  SolverStatistics position_solver_statistics;

  long long total_calc_time = 0LL; // microseconds

public:
  explicit ArticulatedBody( PhysicsWorld * phys_world );
  virtual ~ArticulatedBody() {}

  ArticulatedBody( ArticulatedBody const & ) = delete;
  ArticulatedBody & operator=( ArticulatedBody const & ) = delete;

  PhysicsWorld * physWorld() { return phys_world; }

  RigidBody & createRigidBody( uint color );

  // anchor_mm - the world joint point in millimeters (construction units, see physics.h).
  HingeJoint & createJoint(
    RigidBody * a,
    RigidBody * b,
    Vector3 anchor_mm,
    Vector3 axis_world
  );

  void rebuildPhysicalParameters_afterAllBodiesCreating(); // once, after all bodies have been assembled

  // Granular step phases - the PhysicsWorld interleaves joint and contact passes in one solve loop.
  void applyGravity( Vector3 const & g, double dt );
  void prepareSolve   ( double dt );      // updateInertia + prepareVelocitySolve for the joints
  void warmStartJoints( double dt );      // re-applies the joints' accumulated impulses
  void updateInertia();

  bool solveVelocitiesOnce( double dt );  // one Gauss-Seidel pass over the joints
  void integrateVelocities( double dt );  // integrate velocities of all bodies
  bool solvePositionsOnce();

  void finalizeStep();                    // quaternion normalization + world inertia + the total center of mass

  // ------------------------------------------------------------------ sleeping
  // A sleeping assembly is skipped by every step phase, so a standing robot costs
  // nothing. See SleepSettings for what wakes it - in short: a motor command on any of
  // its joints, or an explicit wakeUp(). Nothing else does, so call wakeUp() yourself
  // after applying an impulse, moving a body or changing the terrain under it.
  inline bool isSleeping() const { return is_sleeping; }

  void wakeUp();

  // Called once per full step (not per substep) by the PhysicsWorld.
  void updateSleepState( SleepSettings const & sleep_settings, double dt );

  // Draws all bodies; joint axes are drawn only when joints_axis_length > 0 (meters).
  virtual void draw( IPhysicsDrawer const& drawer, double joints_axis_length = 0.0 ) const;

  Vector3 const & initialTotalCenterOfMass() const { return initial_total_center_of_mass; }
  Vector3 const & curTotalCenterOfMass    () const { return cur_total_center_of_mass; }

  SolverStatistics const & velocitySolverStatistics() const { return velocity_solver_statistics; }
  SolverStatistics const & positionSolverStatistics() const { return position_solver_statistics; }

  inline long long totalCalcTime() const { return total_calc_time; }

  void calcMainPhysicalParameters( Vector3 const & total_center_of_mass_pos, Vector3 & momentum, Vector3 & angular_momentum, double & kinetic_energy ) const;

protected:
  void finishInitializing();

  // Legacy standalone step (no gravity, no contacts) - kept for simple joint-only tests.
  void processTick( double dt );

private:
  void updateTotalCenterOfMass();
};

} // namespace phys
} // namespace zygo
