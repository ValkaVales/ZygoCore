#pragma once

// Hinge joint (one rotational DOF) between two rigid bodies,
// with optional angle limits and a velocity/position/torque motor.

#include <zygo/math/vector/vec3.h>
#include <zygo/math/matrix/small_fast_matrix/mat3.h>
#include <zygo/physics/joint/joint_limits.h>
#include <zygo/physics/solver_settings.h>
#include <zygo/physics/i_physics_drawer.h>
#include <zygo/physics/world/world_state.h>


namespace zygo {
namespace phys {

class RigidBody;

#define USE_VELOCITY_SOLVER
#define USE_POSITION_SOLVER

// Angular-momentum conservation checks inside the anchor/axis solves.
// OFF by default. Every guarded impulse costs two calcTotalL() calls - per joint, per Gauss-Seidel iteration - so with 30 iterations and 12 joints that is ~720 extra passes over the bodies per substep,
// which in an MSVC Debug build dominates the whole step.
//
// It is a solver-development tool: turn it on while changing the impulse code, off again afterwards.
// In Release it is inert anyway (ZgAssert compiles out), so leaving it on only slows Debug down for nothing.

#define DEBUG_CONSERVATION_CHECKS


class HingeJoint
{
private:
  enum MotorMode
  {
    MOTOR_OFF,
    MOTOR_VELOCITY,
    MOTOR_POSITION,
    MOTOR_TORQUE
  };

private:
  RigidBody * objA = nullptr;
  RigidBody * objB = nullptr;

  // Shared solver tuning of the world this joint lives in (not owned).
  SolverSettings const * settings = nullptr;

  // The joint point in the local coordinates of each body.
  Vector3 local_anchor_A;
  Vector3 local_anchor_B;

  // The joint axis in the local coordinates of each body.
  Vector3 local_axis_A;
  Vector3 local_axis_B;

  // hinge angle limits
  bool limit_enabled = false;
  JointLimits limits;

  double accumulated_lower_limit_impulse = 0.0;
  double accumulated_upper_limit_impulse = 0.0;

  // Reference vectors for the angle measurement, in local coordinates.
  Vector3 local_ref_A;
  Vector3 local_ref_B;

  // motor
  MotorMode motor_mode = MOTOR_OFF;

  double motor_target_velocity = 0.0; // rad/s
  double motor_target_angle    = 0.0; // rad
  double motor_target_torque   = 0.0; // N*m

  double motor_max_torque      = 0.0; // N*m
  double motor_max_velocity    = 0.0; // rad/s

  double accumulated_motor_impulse = 0.0;

  bool wake_requested = false;

  // ------------------------------------------------------------------ per-substep cache
  // Nothing MOVES during the velocity loop - only velocities change; positions and orientations are updated afterwards, in integrateVelocities().
  // So the anchor points, the Jacobian, the effective mass and the Baumgarte bias are all constants of the substep, and prepareVelocitySolve() computes them once.
  //
  // That is not only cheaper (the effective mass is inverted once instead of once per iteration):
  // sequential impulses converges to the solution of ONE linear system, and re-deriving the system on every iteration is not quite that system.
  bool    cache_valid = false;

  Vector3 cached_axis_A;       // world hinge axis of A, normalized
  double  cached_hinge_angle = 0.0;

  // anchor constraint (3 linear rows)
  bool    anchor_valid = false;
  Vector3 anchor_point;        // shared application point for +J and -J
  Vector3 anchor_rA;
  Vector3 anchor_rB;
  Mat3    anchor_K_inv;        // inverse effective mass
  Vector3 anchor_bias;

  // axis constraint (2 angular rows)
  bool    axis_valid = false;
  Vector3 axis_g1;
  Vector3 axis_g2;
  double  axis_inv_K11 = 0.0;  // inverse of the 2x2 effective mass
  double  axis_inv_K12 = 0.0;
  double  axis_inv_K21 = 0.0;
  double  axis_inv_K22 = 0.0;
  double  axis_bias1   = 0.0;
  double  axis_bias2   = 0.0;

  // ------------------------------------------------------------------ warm starting
  // Total impulse the constraint applied over the last substep, kept in WORLD space so that it does not depend on the tangent basis, which is rebuilt every substep.
  Vector3 accumulated_anchor_impulse;
  Vector3 accumulated_axis_impulse;   // angular

  // The substep dt the accumulators were built with.
  // An impulse is force*dt, so a stored one only means the same thing at the same dt - it is rescaled when the step changes and dropped entirely on the very first substep.
  double accumulated_impulse_dt = 0.0;

  // The length of the substep the accumulators below were built over.
  // Separate from accumulated_impulse_dt, which is a warm-start bookkeeping field and is deliberately zeroed when warm starting is off - the reaction getters need the substep length regardless.
  double last_substep_dt = 0.0;

public:
  HingeJoint() = delete;

  // anchor_mm - the world joint point, in millimeters (construction units, see physics.h);
  // axis_world - the world joint axis (any nonzero length).
  HingeJoint(
    RigidBody * a,
    RigidBody * b,
    Vector3 anchor_mm,
    Vector3 axis_world,
    SolverSettings const * settings
  );

  // ------------------------------------------------------------------ sleeping
  // Set by every command that changes what this joint is trying to do. The owning
  // ArticulatedBody consumes it once per step and wakes the whole assembly - otherwise a
  // sleeping robot would silently ignore the first command it is given.
  inline bool consumeWakeRequest() { bool r = wake_requested; wake_requested = false; return r; }
  inline void requestWake()        { wake_requested = true; }

  void enableAngleLimit( JointLimits limits );

  double currentHingeAngle() const;
  double curAngleVelocity() const;

  // ------------------------------------------------------------------ reaction loads
  // What the joint had to do, last substep, to keep the two bodies together.
  //
  // The solver already computes these: they are the accumulated constraint impulses divided by the substep length.
  // Nothing extra is measured, so reading them is free.
  //
  // SIGN: the load applied to body A. Body B receives exactly the opposite.
  // FRAME: world.
  //
  // These are the quantities a real robot measures - a joint torque sensor, a foot force estimate - so a policy trained on them is trained on an observation that exists on the
  // hardware too, which is not true of a "perfect" joint angle.
  //
  // ACCURACY: these report the impulses the VELOCITY solver applied.
  // When that solve is allowed to stop early (StepSettings::velocity_early_out) part of the load is carried by the position solver instead,\
  // and that part does not appear here - the pose stays correct, but the reported load understates the true one.\
  // At the default JointSettings::convergence_rel_eps the gap is under half a percent; the table there gives the numbers.
  // For exact reactions set velocity_early_out = false.
  //
  // Valid after a step has run; all zero before the first one.
  Vector3 reactionForce()  const; // N,   the force  holding the anchors together
  Vector3 reactionTorque() const; // N*m, the torque holding the axes aligned

  // The torque the motor itself delivered, about the hinge axis, N*m.
  // Signed the same way as setMotorTorque(): positive drives the hinge angle up.
  // |motorTorque()| <= the max_torque given to the servo, so comparing the two tells you whether the actuator was saturated - the single most useful number when a gait misbehaves.
  double motorTorque() const;

  // The torque an angle limit had to absorb, N*m, signed like motorTorque().
  // Zero unless a limit is enabled and actually loaded.
  double limitTorque() const;

  Vector3 worldAnchorA() const;
  Vector3 worldAnchorB() const;

  Vector3 worldAxisA() const;
  Vector3 worldAxisB() const;

  // motor
  void setMotorVelocity( double target_velocity_rad, double max_torque );
  void setMotorPosition( double target_angle_rad, double max_torque, double max_velocity_rad );

  // Open-loop torque source: the motor stops being a constraint and simply injects torque*dt of angular impulse per step, positive torque increasing the hinge angle.
  //
  // Unlike the velocity/position modes this one obeys NEITHER max_torque NOR max_velocity - the caller owns both.
  // That is deliberate: it lets an outer controller model a real actuator
  // (its own torque-speed curve, a gearbox or a belt whose reaction lands on a different hinge)
  // instead of the idealized "unlimited torque up to a hard speed wall" the built-in servo assumes.
  void setMotorTorque( double torque );

  void disableMotor();

  // Rebuilds the per-substep cache above and resets the servo/limit accumulators.
  // Call once per substep, AFTER the world inertia has been refreshed.
  void prepareVelocitySolve( double dt );

  // Applies open-loop MOTOR_TORQUE exactly once per substep.
  // This is an external actuator impulse, not an iterative constraint, so it must live outside the Gauss-Seidel loop or it would depend on max_velocity_iterations.
  void applyExternalActuatorImpulse( double dt );

  // Re-applies the accumulated anchor/axis impulses.
  // Separate from prepareVelocitySolve() so that the world can run it next to warmStartContacts(), i.e. after contact detection has sampled the approach velocities.
  void warmStartVelocitySolve( double dt );

  double hingeAngularMassInv() const;

  //
#ifdef USE_VELOCITY_SOLVER
  bool solveVelocityConstraint( double dt ); // returns true, if still has error
#endif

#ifdef USE_POSITION_SOLVER
  bool solvePositionConstraint(); // returns true, if still has error
#endif

  // Draws the joint axis at both anchors.
  void draw( IPhysicsDrawer const& drawer, double axis_length ) const;

  // ------------------------------------------------------------------ state snapshot
  // The accumulated impulses and the motor command - see world_state.h.
  // The per-substep cache is NOT saved: prepareVelocitySolve() rebuilds it from the bodies at the start of every substep, so restoring it would be storing a derived quantity.
  void saveState   ( HingeJointState & out ) const;
  void restoreState( HingeJointState const & in );

// ------------------------------------------------------------------------------------------------------------------------ private methods
private:
  void prepareAnchorConstraint( double dt );
  void prepareAxisConstraint  ( double dt );

#ifdef USE_VELOCITY_SOLVER
#ifdef DEBUG_CONSERVATION_CHECKS
  Vector3 calcTotalL() const;

  // Relative tolerance of the angular-momentum check.
  // An impulse pair conserves L exactly in exact arithmetic, so this measures pure round-off; 1e-11 leaves ~3 decades of margin over the double round-off of calcTotalL() itself.
  static constexpr double CONSERVATION_REL_EPS = 1e-11;

  void assertAngularMomentumConserved( Vector3 const & L_before, Vector3 const & L_after, double impulse_magnitude ) const;
#endif

  bool solveAxisVelocity  ( double dt ); // returns true, if still has error
  bool solveAnchorVelocity( double dt ); // returns true, if still has error
#endif

#ifdef USE_POSITION_SOLVER
  bool solveAnchorPosition(); // returns true, if still has error
  bool solveAxisPosition  (); // returns true, if still has error
#endif

  bool solveMotorVelocityConstraint( double dt );
  void setMotorMode( MotorMode mode );

  // hinge angle limits
  Vector3 worldRefA() const;
  Vector3 worldRefB() const;

  bool solveAngleLimitVelocity( double dt );

  static double calcSignedAngleAroundAxis(
    Vector3 const & a,
    Vector3 const & b,
    Vector3 const & axis_unit
  );

  static Mat3 computeAnchorEffectiveMass(
    double inv_mass_A,
    double inv_mass_B,
    Vector3 const& rA,
    Vector3 const& rB,
    Mat3 const& inv_IA,
    Mat3 const& inv_IB,
    double softness
  );
};

} // namespace phys
} // namespace zygo
