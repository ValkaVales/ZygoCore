#pragma once

// Rigid body assembled from simple shapes.
// The construction API takes grams/millimeters; everything else is SI (see physics.h).

#include <zygo/physics/shape/shape.h>
#include <zygo/math/matrix/matrix.h>
#include <zygo/math/quaternion/quaternion.h>
#include <memory>
#include <vector>


namespace zygo {
namespace phys {

class RigidBody
{
  friend class HingeJoint;

private:
  bool initialized;

  Vector3 center_of_mass_pos;
  Quaternion rotation_quaternion;

  Vector3 speed;
  Vector3 angular_speed;    // world-space angular velocity

  double total_mass;
  double inv_mass;

  std::vector<std::unique_ptr<Shape>> shapes;

  uint color;

  // Inertia tensors.
  Matrix inertia_tensor_local;
  Matrix inertia_tensor_local_inv;
  Matrix inertia_tensor_world;
  Matrix inertia_tensor_world_inv;

public:
  explicit RigidBody( uint color );

  RigidBody( RigidBody const & ) = delete;
  RigidBody & operator=( RigidBody const & ) = delete;

  void clearGeometry();
  void rebuildPhysicalParameters_afterAllShapesAdded(); // should be called only once, after all shapes have been added

  void normalizeQuaternion();
  void updateWorldInertia();

  inline bool isStatic() const { return inv_mass == 0.0; }

  // Gravity/external forces: adds an acceleration to the linear speed.
  void applyGravity( Vector3 const & gravity, double dt );

  // Inverse effective mass J * M^-1 * J^T along the (unit) direction dir at world_point.
  // For the contact solver.
  double invEffectiveMassAlong( Vector3 const & world_point, Vector3 const & dir ) const;

  // ------------------------------------------------------------------ construction (grams / millimeters)
  void addBoxBySegment(
    double mass_grams,
    Vector3 p1_mm,
    Vector3 p2_mm,
    double width_mm,
    double height_mm
  );

  void addCapsuleBySegment(
    double mass_grams,
    Vector3 p1_mm,
    Vector3 p2_mm,
    double diameter_mm
  );

  void addBoxByDiagonal(
    double mass_grams,
    Vector3 p1_mm,
    Vector3 p2_mm
  );

  void addVerticalPlate(
    double mass_grams,
    Vector3 p1_mm,
    Vector3 p2_mm,
    double width_mm
  );

  void addCylinderBySegment(
    double mass_grams,
    Vector3 p1_mm,
    Vector3 p2_mm,
    double diameter_mm
  );

  void addSphere(
    double mass_grams,
    Vector3 center_mm,
    double diameter_mm
  );

  //
  uint getColor() const { return color; }

  //
  void addAngularSpeed( Vector3 const & angular_speed_addon );

  Vector3 localPointToWorld ( Vector3 const & point_local ) const;
  Vector3 worldPointToLocal ( Vector3 const & point_world ) const;
  Vector3 localDirToWorld   ( Vector3 const & dir_local ) const;
  Vector3 pointVelocityWorld( Vector3 const & point_world ) const;

  Vector3 worldVectorToLocal( Vector3 const & v ) const;
  Vector3 localVectorToWorld( Vector3 const & v ) const;

  void applyPositionImpulseAtWorldPoint( Vector3 const & impulse, Vector3 const & world_point );
  void applyImpulseAtWorldPoint        ( Vector3 const & impulse, Vector3 const & world_point );
  void applyAngularImpulse( Vector3 const & angular_impulse );
  void integrateVelocities( double dt );

  inline double getMass() const { return total_mass; }
  inline Vector3 const & centerOfMassPos() const { return center_of_mass_pos; }

  // Momentum, angular momentum (about the given system center of mass) and kinetic energy of this body.
  void calcMainPhysicalParameters( Vector3 const & total_center_of_mass_pos, Vector3 & momentum, Vector3 & angular_momentum, double & kinetic_energy ) const;

  void draw( IPhysicsDrawer const& drawer ) const;

private:
  void calcMassAndLocalCenterOfMass();
  Vector3 calcOmegaBodyDerivative( Vector3 const & omega_body, Vector3 const & torque_body ) const;

  void addShape( Shape * shape );

  void syncAngularMomentumFromAngularSpeed();

  void applyOrientationCorrection( Vector3 const & small_angle );
};

} // namespace phys
} // namespace zygo
