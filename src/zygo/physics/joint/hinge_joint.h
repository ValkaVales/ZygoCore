#pragma once

// Hinge joint (one rotational DOF) between two rigid bodies,
// with optional angle limits and a velocity/position/torque motor.

#include <zygo/math/vector/vec3.h>
#include <zygo/physics/joint/joint_limits.h>
#include <zygo/physics/solver_settings.h>
#include <zygo/physics/i_physics_drawer.h>


namespace zygo {
namespace phys {

class RigidBody;

#define USE_VELOCITY_SOLVER
#define USE_POSITION_SOLVER
#define DEBUG_CONSERVATION_CHECKS // L conservation checks in the anchor/axis solves (debug only: costs two calcTotalL() per impulse)


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

  void enableAngleLimit( JointLimits limits );

  double currentHingeAngle() const;
  double curAngleVelocity() const;

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

  void prepareVelocitySolve();

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

private:
#ifdef USE_VELOCITY_SOLVER
#ifdef DEBUG_CONSERVATION_CHECKS
  Vector3 calcTotalL() const;
#endif

  bool solveAxisVelocity  ( double dt ); // returns true, if still has error
  bool solveAnchorVelocity( double dt ); // returns true, if still has error
#endif

#ifdef USE_POSITION_SOLVER
  bool solveAnchorPosition(); // returns true, if still has error
  bool solveAxisPosition  (); // returns true, if still has error
#endif

  bool solveMotorVelocityConstraint( double dt );

  // hinge angle limits
  Vector3 worldRefA() const;
  Vector3 worldRefB() const;

  bool solveAngleLimitVelocity( double dt );

  static double calcSignedAngleAroundAxis(
    Vector3 const & a,
    Vector3 const & b,
    Vector3 const & axis_unit
  );
};

} // namespace phys
} // namespace zygo
