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

void ArticulatedBody::applyExternalActuatorImpulses( double dt )
{
  ZgAssert( initialized );

  for ( auto & joint : joints )
    joint->applyExternalActuatorImpulse( dt );
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
  applyExternalActuatorImpulses( dt );
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

// --------------------------------------------------------------------------- sleeping
void ArticulatedBody::wakeUp()
{
  is_sleeping = false;
  idle_time   = 0.0;

  for ( auto & obj : objects )
    obj->setSleeping( false );
}

void ArticulatedBody::updateSleepState( SleepSettings const & sleep_settings, double dt )
{
  ZgAssert( initialized );

  // A joint command always wins, even while the feature is off - the flags have to be
  // consumed either way, or a stale one would wake the assembly much later.
  bool commanded = false;

  for ( auto & joint : joints )
    if ( joint->consumeWakeRequest() )
      commanded = true;

  if ( !sleep_settings.enabled )
  {
    if ( is_sleeping )
      wakeUp();

    idle_time = 0.0;
    return;
  }

  if ( commanded )
  {
    wakeUp();
    return;
  }

  if ( is_sleeping )
    return;

  // Every body has to be quiet. One that is not resets the timer for the whole assembly:
  // the bodies are rigidly coupled, so a moving shin means the trunk is not at rest
  // either, it just happens to be near the instantaneous center of the motion.
  double const lin_sqr = sqr( sleep_settings.linear_velocity_threshold );
  double const ang_sqr = sqr( sleep_settings.angular_velocity_threshold );

  for ( auto const & obj : objects )
  {
    if ( obj->isStatic() )
      continue;

    if ( obj->linearSpeed ().lengthSqr() > lin_sqr
      || obj->angularSpeed().lengthSqr() > ang_sqr )
    {
      idle_time = 0.0;
      return;
    }
  }

  idle_time += dt;

  if ( idle_time < sleep_settings.time_to_sleep )
    return;

  is_sleeping = true;

  for ( auto & obj : objects )
    obj->setSleeping( true );
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

// ------------------------------------------------------------------------ state snapshot
void ArticulatedBody::saveState( WorldState & out ) const
{
  ZgAssert( initialized );

  ArticulatedBodyState st;
  st.is_sleeping = is_sleeping;
  st.idle_time   = idle_time;
  out.assemblies.push_back( st );

  for ( auto const & obj : objects )
  {
    RigidBodyState bs;
    obj->saveState( bs );
    out.bodies.push_back( bs );
  }

  for ( auto const & joint : joints )
  {
    HingeJointState js;
    joint->saveState( js );
    out.joints.push_back( js );
  }
}

bool ArticulatedBody::restoreState( WorldState const & in, size_t & body_index, size_t & joint_index )
{
  ZgAssert( initialized );

  if ( body_index  + objects.size() > in.bodies.size() ) return false;
  if ( joint_index + joints .size() > in.joints.size() ) return false;

  for ( auto & obj : objects )
    obj->restoreState( in.bodies[ body_index++ ] );

  for ( auto & joint : joints )
    joint->restoreState( in.joints[ joint_index++ ] );

  // The assembly-level flags are written by PhysicsWorld::restoreState(), which knows this assembly's index; everything below is derived and is rebuilt from the bodies.
  updateTotalCenterOfMass();

  return true;
}

void ArticulatedBody::restoreSleepState( ArticulatedBodyState const & st )
{
  is_sleeping = st.is_sleeping;
  idle_time   = st.idle_time;

  for ( auto & obj : objects )
    obj->setSleeping( is_sleeping );
}

} // namespace phys
} // namespace zygo
