#include "physics_world.h"
#include <zygo/physics/phys_consts.h>

#include <zygo/math/common/scalar.h>
#include <cmath>
#include <chrono>


namespace zygo {
namespace phys {

namespace
{
  const uint CONTACT_SPHERE_DRAW_COLOR = 0xcccccc;
}


PhysicsWorld::PhysicsWorld( ITerrain * terrain )
  : terrain ( terrain )
{
  // All tuning defaults live in SolverSettings; gravity is OFF by default.
}

void PhysicsWorld::addObject( ArticulatedBody & obj )
{
  objects.push_back( &obj );
}

void PhysicsWorld::addContactSphere( RigidBody & body, Vector3 center_world_mm, double radius_mm )
{
  // worldPointToLocal() is only meaningful once the body knows its center of mass and its orientation, i.e. after rebuildPhysicalParameters_afterAllShapesAdded()
  // (which ArticulatedBody::rebuildPhysicalParameters_afterAllBodiesCreating() runs for the whole assembly).
  // Called earlier it silently returns a center measured from the wrong origin, and the foot ends up somewhere else entirely - with nothing visibly wrong anywhere.
  //
  // Release-level on purpose: this is one-off setup code, it cannot be triggered by runtime data, and the failure it guards against is invisible and very expensive.
  ZgAssertRelease( body.isInitialized() );

  Vector3 center_world = center_world_mm / MILLIMETERS_IN_METER;
  double  radius       = radius_mm       / MILLIMETERS_IN_METER;

  ContactSphere cs;
  cs.body         = &body;
  cs.local_center = body.worldPointToLocal( center_world );
  cs.radius       = radius;

  contact_spheres.push_back( cs );
}

void PhysicsWorld::draw( IPhysicsDrawer const& drawer ) const
{
  for ( auto const & cs : contact_spheres )
  {
    Vector3 p = cs.body->localPointToWorld( cs.local_center );

    drawer.sphere( p, cs.radius, CONTACT_SPHERE_DRAW_COLOR );
  }
}

void PhysicsWorld::processTick( double dt )
{
  if ( dt <= 0.0 )
    return;

  auto start = std::chrono::high_resolution_clock::now();

  int n = max2( 1, settings.step.substeps );
  double h = dt / n;

  for ( int i = 0; i < n; ++i )
    subStep( h );

  // Once per full step, not per substep: the sleep timer is measured in real time and the thresholds are about the motion of the assembly, not about one substep of it.
  for ( auto * co : objects )
    co->updateSleepState( settings.sleep, dt );

  auto end = std::chrono::high_resolution_clock::now();
  total_calc_time += std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();
}

void PhysicsWorld::subStep( double dt )
{
  // A sleeping assembly is skipped by every phase below.
  // If they are ALL asleep there is nothing left to solve at all - not even a contact, since detectContacts() ignores their spheres too.
  int awake_count = 0;

  for ( auto * co : objects )
    if ( !co->isSleeping() )
      ++awake_count;

  if ( awake_count == 0 )
    return;

  // 1) External forces -> velocities (gravity BEFORE the solve).
  if ( settings.gravity.enabled )
  {
    for ( auto * co : objects )
      if ( !co->isSleeping() )
        co->applyGravity( settings.gravity.g, dt );
  }

  // 2) Joint preparation (refreshes the world inertia) + contact detection.
  for ( auto * co : objects )
    if ( !co->isSleeping() )
      co->prepareSolve( dt );

  // Contacts are detected BEFORE any warm start:
  // detectContacts() samples the approach velocity vn0 for restitution,
  // and that has to be the speed the foot actually arrives with, not the speed left over after last substep's impulses have been re-applied.
  // dt is passed on so the query margin can grow with the approach speed (ContactSettings::speculative_contacts).
  detectContacts( dt );

  for ( auto * co : objects )
    if ( !co->isSleeping() )
      co->warmStartJoints( dt );

  warmStartContacts();

#ifdef USE_VELOCITY_SOLVER
  // 3) The shared velocity solve: joints and contacts in one Gauss-Seidel loop.
  for ( int i = 0; i < settings.step.velocity_iterations; ++i )
  {
    velocity_solver_statistics.iterations_count = i + 1;

    for ( auto * co : objects )
      if ( !co->isSleeping() )
        co->solveVelocitiesOnce( dt );

    solveContactsCollisionOnce( dt );
    solveContactsFrictionOnce();

    // No early-out here: the joint and contact passes disturb each other,
    // so the loop always runs the full velocity_iterations count.
  }

  velocity_solver_statistics.update();
#endif

  // 4) Integration.
  for ( auto * co : objects )
    if ( !co->isSleeping() )
      co->integrateVelocities( dt );

  // 5) Position correction for the contacts and, optionally, the joints.
  for ( auto * co : objects )
    if ( !co->isSleeping() )
      co->updateInertia();

  for ( int i = 0; i < settings.step.position_iterations; ++i )
  {
    position_solver_statistics.iterations_count = i + 1;
    bool has_error = false;

    // Important: every position correction moves the geometry, so the contacts are re-detected.
    // speculative_dt == 0 - no speculation here: the bodies have already been integrated, only real overlaps matter.
    detectContacts( 0.0 );

    if ( solveContactPositionsOnce() )
      has_error = true;

#ifdef USE_POSITION_SOLVER
    for ( auto * co : objects )
      if ( !co->isSleeping() && co->solvePositionsOnce() )
        has_error = true;
#endif

    if ( !has_error )
      break;
  }

  position_solver_statistics.update();

  // 6) Finalization (quaternion normalization, world inertia, the total center of mass).
  for ( auto * co : objects )
    if ( !co->isSleeping() )
      co->finalizeStep();
}

bool PhysicsWorld::solveContactPositionsOnce()
{
  bool has_error = false;

  for ( auto & c : contacts )
  {
    ContactSphere & cs = *c.sphere;
    RigidBody & b = *cs.body;

    if ( b.isStatic() )
      continue;

    double penetration = c.penetration - settings.contacts.position_slop;

    if ( penetration <= 0.0 )
      continue;

    double correction = settings.contacts.position_beta * penetration;
    applyMax( correction, settings.contacts.max_position_correction );

    double k = b.invEffectiveMassAlong( c.point, c.normal );

    if ( k <= PHYS_EPSILON )
      continue;

    // We need the contact point to move outward by about `correction`.
    // The point displacement along the normal from a pseudo-impulse lambda:
    // delta = k * lambda, hence lambda = correction / k.
    double lambda = correction / k;

    Vector3 impulse = c.normal * lambda;

    b.applyPositionImpulseAtWorldPoint( impulse, c.point, settings.contacts.max_position_angular_correction );

    if ( correction > settings.contacts.min_position_correction )
      has_error = true;
  }

  return has_error;
}

void PhysicsWorld::detectContacts( double speculative_dt )
{
  contacts.clear();

  if ( terrain == nullptr )
  {
    for ( auto & cs : contact_spheres )
      cs.was_in_contact = false;
    return;
  }

  bool const speculative =
       settings.contacts.speculative_contacts
    && speculative_dt > 0.0;

  for ( auto & cs : contact_spheres )
  {
    // A sleeping body is not integrated, so its feet cannot have moved.
    // Leave the accumulators alone: they are exactly the state it has to wake up into.
    if ( cs.body->isSleeping() )
      continue;

    Vector3 world_center = cs.body->localPointToWorld( cs.local_center );

    // ------------------------------------------------------------------ query margin
    // The base margin only asks "is the sphere already touching?".
    // A foot landing at 2 m/s crosses 4 mm during a 2 ms substep, so with a 0.1 mm margin it is never seen approaching - only afterwards, from the inside.
    //
    // The speculative margin is simply how far the sphere can travel toward the ground before the substep ends, so a contact that WILL happen is found before it happens.
    double margin = settings.contacts.margin;

    if ( speculative )
    {
      // The normal is not known until the query returns, so the previous one is used as the estimate.
      // It defaults to +Z and only changes slowly - and being wrong here costs a slightly wrong margin, nothing else.
      Vector3 v  = cs.body->pointVelocityWorld( world_center );
      double  vn = v * cs.prev_normal; // < 0 => approaching the ground

      if ( vn < 0.0 )
      {
        double speculative_margin = -vn * speculative_dt;
        applyMax( speculative_margin, settings.contacts.speculative_margin_max );
        applyMin( margin, speculative_margin ); // margin = max( margin, speculative )
      }
    }

    TerrainContact tc = terrain->querySphere( world_center, cs.radius, margin );

    if ( !tc.hit )
    {
      cs.was_in_contact = false;

      cs.accumulated_normal_impulse = 0.0;
      cs.accumulated_t1_impulse     = 0.0;
      cs.accumulated_t2_impulse     = 0.0;
      cs.accumulated_spin_impulse   = 0.0;
      continue;
    }

    ContactPoint c;
    c.sphere      = &cs;
    c.point       = tc.point;
    c.normal      = tc.normal;
    c.penetration = tc.penetration;
    c.separation  = tc.separation;

    c.normal.buildOrthonormalBasisFromAxis( c.t1, c.t2 );

    // The approach speed before the contact response (after gravity, before warm-start).
    Vector3 v = cs.body->pointVelocityWorld( c.point );
    c.vn0 = v * c.normal;

    if ( !cs.was_in_contact )
    {
      // The contact has just appeared - there is nothing to warm-start, reset the accumulators.
      cs.accumulated_normal_impulse = 0.0;
      cs.accumulated_t1_impulse     = 0.0;
      cs.accumulated_t2_impulse     = 0.0;
      cs.accumulated_spin_impulse   = 0.0;

      // Latch the arrival speed for restitution.
      // With speculative contacts the sphere is braked while it is still in the air, so by the time it truly touches,
      // the instantaneous speed is no longer the speed it arrived with - this is.
      cs.impact_vn = c.vn0;
    } else
    {
      // If the normal has turned considerably, the old tangent friction belongs
      // to a different tangent plane. Reset only the friction.
      double normal_dot = cs.prev_normal * tc.normal;

      if ( normal_dot < settings.contacts.normal_reset_dot )
      {
        cs.accumulated_t1_impulse = 0.0;
        cs.accumulated_t2_impulse = 0.0;
        cs.accumulated_spin_impulse = 0.0;
      }
    }

    cs.prev_normal = tc.normal;
    cs.was_in_contact = true;

    c.kn  = cs.body->invEffectiveMassAlong( c.point, c.normal );
    c.kt1 = cs.body->invEffectiveMassAlong( c.point, c.t1 );
    c.kt2 = cs.body->invEffectiveMassAlong( c.point, c.t2 );
    c.k_spin = cs.body->invAngularEffectiveMassAbout( c.normal );

    contacts.push_back( c );
  }
}

void PhysicsWorld::warmStartContacts()
{
  for ( auto & c : contacts )
  {
    ContactSphere & cs = *c.sphere;

    Vector3 P =
        c.normal * cs.accumulated_normal_impulse
      + c.t1     * cs.accumulated_t1_impulse
      + c.t2     * cs.accumulated_t2_impulse;

    cs.body->applyImpulseAtWorldPoint( P, c.point );

    if ( cs.accumulated_spin_impulse != 0.0 )
      cs.body->applyAngularImpulse( c.normal * cs.accumulated_spin_impulse );
  }
}

bool PhysicsWorld::solveContactsCollisionOnce( double dt )
{
  bool has_error = false;

  for ( auto & c : contacts )
  {
    ContactSphere & cs = *c.sphere;
    RigidBody & b = *cs.body;

    // ------------------------------------------------------------
    // The normal contact impulse (one-sided: the impulse only pushes outward, acc >= 0).
    if ( c.kn > PHYS_EPSILON )
    {
      Vector3 v = b.pointVelocityWorld( c.point );
      double  vn = v * c.normal; // the speed projection on the contact normal

      double target_speed;

      if ( c.separation > 0.0 )
      {
        // ---- speculative contact: the sphere has NOT touched the ground yet ----
        //
        // It only got into the contact list because it is moving fast enough to arrive within this substep.
        // It must NOT be stopped in mid-air, so the constraint is not "stand still" but "do not end the substep below the surface":
        //
        //   the fastest legal approach is exactly the one that closes the gap.
        //
        // Combined with the one-sided clamp below (a contact can only push), a sphere that is approaching slower than that gets no impulse at all - 
        // the row simply does nothing until it is needed.
        target_speed = -c.separation / dt;
      } else
      {
        // Baumgarte bias: pushing out of the penetration.
        double bias = 0.0;
        double penetration = c.penetration - settings.contacts.slop;

        if ( penetration > 0.0 )
        {
          bias = settings.contacts.baumgarte_beta * penetration / dt;
          applyMax( bias, settings.contacts.max_bias_speed );
        }

        // Restitution: the bounce.
        // Driven by the latched arrival speed, not by the current one - see ContactSphere::impact_vn.
        double restitution_speed = 0.0;
        if ( settings.contacts.restitution > 0.0  &&  cs.impact_vn < -settings.contacts.restitution_velocity_threshold ) // impact_vn < 0 when approaching
          restitution_speed = -settings.contacts.restitution * cs.impact_vn;

        // We want either the push-out or the bounce.
        // The target normal speed = max( push-out, bounce ).
        target_speed = max2( bias, restitution_speed );
      }

      // c.kn is J * M^-1 * J^T, i.e. the inverse effective mass.
      double lambda = (target_speed - vn) / c.kn; // == delta_speed * mass == (delta_speed/dt) * mass * dt == F * dt == impulse

      double old_accumulated_normal_impulse = cs.accumulated_normal_impulse;
      double new_accumulated_normal_impulse = old_accumulated_normal_impulse + lambda;

      applyMin( new_accumulated_normal_impulse, 0.0 ); // a contact can only push, never pull

      double impulse_delta = new_accumulated_normal_impulse - old_accumulated_normal_impulse;
      cs.accumulated_normal_impulse = new_accumulated_normal_impulse;

      Real impulse_len = std::abs( impulse_delta );

      if ( !isZero( impulse_len, PHYS_EPSILON ) )
        b.applyImpulseAtWorldPoint( c.normal * impulse_delta, c.point );

      if ( impulse_len > settings.contacts.min_error_for_collision_impulse )
        has_error = true;
    }
  }

  return has_error;
}

bool PhysicsWorld::solveContactsFrictionOnce()
{
  bool has_error = false;

  for ( auto & c : contacts )
  {
    ContactSphere & cs = *c.sphere;
    RigidBody & b = *cs.body;

    // ------------------------------------------------------------
    // The friction impulse with a circular Coulomb cone:
    // two tangents, the clamp is |J_t| <= mu * J_n.
    double max_friction = settings.contacts.friction_mu * cs.accumulated_normal_impulse;
    applyMin( max_friction, 0.0 );

    // A local function: try to set the new accumulated tangent impulses,
    // projecting them into the circle |Jt| <= mu * Jn.
    auto setProjectedFrictionImpulse =
      [&]( double wanted_t1, double wanted_t2 ) -> bool
      {
        double old_t1 = cs.accumulated_t1_impulse;
        double old_t2 = cs.accumulated_t2_impulse;

        double new_t1 = wanted_t1;
        double new_t2 = wanted_t2;

        double len = sqrt( sqr(new_t1) + sqr(new_t2) );

        if ( len > max_friction && len > PHYS_EPSILON )
        {
          double scale = max_friction / len;
          new_t1 *= scale;
          new_t2 *= scale;
        }

        double delta_t1 = new_t1 - old_t1;
        double delta_t2 = new_t2 - old_t2;

        cs.accumulated_t1_impulse = new_t1;
        cs.accumulated_t2_impulse = new_t2;

        Vector3 impulse = c.t1 * delta_t1 + c.t2 * delta_t2;

        if ( !isZero( impulse.lengthSqr(), PHYS_EPSILON_SQR ) )
          b.applyImpulseAtWorldPoint( impulse, c.point );

        return std::abs( delta_t1 ) > settings.contacts.min_error_for_friction_impulse
            || std::abs( delta_t2 ) > settings.contacts.min_error_for_friction_impulse;
      };

    // Important: if the normal impulse has decreased, the old friction may exceed
    // the new limit. Squeeze the old accumulated friction first.
    if ( setProjectedFrictionImpulse( cs.accumulated_t1_impulse, cs.accumulated_t2_impulse ) )
      has_error = true;

    // Solve t1.
    if ( c.kt1 > PHYS_EPSILON && max_friction > 0.0 )
    {
      Vector3 v = b.pointVelocityWorld( c.point );
      double vt1 = v * c.t1;

      double lambda_t1 = -vt1 / c.kt1;

      if ( setProjectedFrictionImpulse( cs.accumulated_t1_impulse + lambda_t1, cs.accumulated_t2_impulse ) )
        has_error = true;
    }

    // Solve t2.
    if ( c.kt2 > PHYS_EPSILON && max_friction > 0.0 )
    {
      Vector3 v = b.pointVelocityWorld( c.point );
      double vt2 = v * c.t2;

      double lambda_t2 = -vt2 / c.kt2;

      if ( setProjectedFrictionImpulse( cs.accumulated_t1_impulse, cs.accumulated_t2_impulse + lambda_t2 ) )
        has_error = true;
    }

    // ------------------------------------------------------------
    // Torsional (spin) friction: one angular row about the contact normal.
    //
    // The two tangential rows above resist SLIDING, but a sphere touches a plane at a single point and nothing there resists TWISTING about the normal.
    // A planted foot is therefore free to spin, and a walking quadruped slowly yaws away with no external torque acting on it at all.
    //
    // A real foot has a finite contact patch, and integrating Coulomb friction over a patch of radius r gives a torque limit proportional to r * (normal force).
    // That is exactly the clamp used here, with the sphere radius standing in for the patch:
    //
    //   |spin impulse| <= spin_friction_mu * radius * normal impulse
    if ( settings.contacts.spin_friction_mu > 0.0 && c.k_spin > PHYS_EPSILON )
    {
      double max_spin = settings.contacts.spin_friction_mu * cs.radius * cs.accumulated_normal_impulse;
      applyMin( max_spin, 0.0 );

      double wn = b.angularSpeed() * c.normal;

      double old_spin = cs.accumulated_spin_impulse;
      double new_spin = old_spin - wn / c.k_spin;
      toRange( new_spin, -max_spin, max_spin );

      cs.accumulated_spin_impulse = new_spin;

      double spin_delta = new_spin - old_spin;

      if ( !isZero( std::abs( spin_delta ), PHYS_EPSILON ) )
        b.applyAngularImpulse( c.normal * spin_delta );

      if ( std::abs(spin_delta) > settings.contacts.min_error_for_friction_impulse )
        has_error = true;
    }
  }

  return has_error;
}

} // namespace phys
} // namespace zygo
