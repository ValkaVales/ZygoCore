#include "articulated_body.h"
#include <zygo/physics/world/physics_world.h>

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <chrono>


namespace zygo {
namespace phys {

namespace
{
  // These are the MAXIMUM ALLOWED values for the legacy standalone step.
  // Real values are 1 or 2, and 10 as a practical maximum (at the start).
  const int VELOCITY_MAX_ITERATIONS_COUNT = 1000;
}


ArticulatedBody::ArticulatedBody( PhysicsWorld * phys_world )
  : phys_world ( phys_world )
{
  ZgAssert( phys_world != nullptr ); // the joints read the solver settings from the world
}

RigidBody & ArticulatedBody::createRigidBody( uint color )
{
  ZgAssert( !initialized );
  objects.push_back( std::make_unique<RigidBody>( color ) );
  return *objects.back();
}

HingeJoint & ArticulatedBody::createJoint(
  RigidBody * a,
  RigidBody * b,
  Vector3 anchor_mm,
  Vector3 axis_world
)
{
  ZgAssert( !initialized );
  joints.push_back( std::make_unique<HingeJoint>( a, b, anchor_mm, axis_world, &phys_world->settings ) );
  return *joints.back();
}

void ArticulatedBody::updateInertia()
{
  ZgAssert( initialized );
  for ( auto & obj : objects )
    obj->updateWorldInertia();
}

void ArticulatedBody::applyGravity( Vector3 const & g, double dt )
{
  ZgAssert( initialized );

  for ( auto & obj : objects )
    obj->applyGravity( g, dt );
}

void ArticulatedBody::prepareSolve( double dt )
{
  ZgAssert( initialized );

  updateInertia();

  for ( auto & joint : joints )
    joint->prepareVelocitySolve( dt );
}

void ArticulatedBody::warmStartJoints( double dt )
{
  ZgAssert( initialized );

  for ( auto & joint : joints )
    joint->warmStartVelocitySolve( dt );
}

bool ArticulatedBody::solveVelocitiesOnce( double dt )
{
  bool has_error = false;

#ifdef USE_VELOCITY_SOLVER
  for ( auto & joint : joints )
    if ( joint->solveVelocityConstraint( dt ) )
      has_error = true;
#endif

  return has_error;
}

void ArticulatedBody::integrateVelocities( double dt )
{
  for ( auto & obj : objects )
    obj->integrateVelocities( dt );
}

bool ArticulatedBody::solvePositionsOnce()
{
  bool has_error = false;

#ifdef USE_POSITION_SOLVER
  for ( auto & joint : joints )
    if ( joint->solvePositionConstraint() )
      has_error = true;
#endif

  return has_error;
}

void ArticulatedBody::finalizeStep()
{
  for ( auto & obj : objects )
  {
    obj->normalizeQuaternion();
    obj->updateWorldInertia();
  }

  updateTotalCenterOfMass();
}

// Legacy path (no gravity and no contacts): the granular phases in one call.
void ArticulatedBody::processTick( double dt )
{
  ZgAssert( dt > 0.0 );
  ZgAssert( initialized );

  auto start = std::chrono::high_resolution_clock::now();

  prepareSolve( dt );
  warmStartJoints( dt );

#ifdef USE_VELOCITY_SOLVER
  for ( int i = 0; i < VELOCITY_MAX_ITERATIONS_COUNT; ++i )
  {
    velocity_solver_statistics.iterations_count = i + 1;
    if ( !solveVelocitiesOnce( dt ) )
      break;
  }
  velocity_solver_statistics.update();
#endif

  integrateVelocities( dt );
  finalizeStep();

  auto end = std::chrono::high_resolution_clock::now();
  total_calc_time += std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();
}

void ArticulatedBody::updateTotalCenterOfMass()
{
  ZgAssert( initialized );

  cur_total_center_of_mass.reset();
  double total_mass = 0.0;

  for ( auto const & obj : objects )
  {
    cur_total_center_of_mass += obj->centerOfMassPos() * obj->getMass();
    total_mass += obj->getMass();
  }

  ZgAssert( total_mass > BIG_EPSILON );
  cur_total_center_of_mass /= total_mass;
}

void ArticulatedBody::rebuildPhysicalParameters_afterAllBodiesCreating()
{
  ZgAssertRelease( !initialized );
  for ( auto & obj : objects )
    obj->rebuildPhysicalParameters_afterAllShapesAdded();
}

void ArticulatedBody::finishInitializing()
{
  ZgAssertRelease( !initialized );
  initialized = true;

  updateTotalCenterOfMass();
  initial_total_center_of_mass = cur_total_center_of_mass;
}

void ArticulatedBody::draw( IPhysicsDrawer const& drawer, double joints_axis_length ) const
{
  ZgAssert( initialized );

  for ( auto const & obj : objects )
    obj->draw( drawer );

  if ( joints_axis_length > 0.0 )
  {
    for ( auto const & joint : joints )
      joint->draw( drawer, joints_axis_length );
  }
}

void ArticulatedBody::calcMainPhysicalParameters( Vector3 const & total_center_of_mass_pos, Vector3 & momentum, Vector3 & angular_momentum, double & kinetic_energy ) const
{
  ZgAssert( initialized );

  momentum        .reset();
  angular_momentum.reset();
  kinetic_energy = 0.0;

  for ( auto const & obj : objects )
  {
    Vector3 obj_momentum;
    Vector3 obj_angular_momentum;
    double  obj_kinetic_energy;

    obj->calcMainPhysicalParameters( total_center_of_mass_pos, obj_momentum, obj_angular_momentum, obj_kinetic_energy );

    momentum          += obj_momentum;
    angular_momentum  += obj_angular_momentum;
    kinetic_energy    += obj_kinetic_energy;
  }
}

} // namespace phys
} // namespace zygo
