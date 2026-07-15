#pragma once

// Base class of collision/inertia shapes a RigidBody is assembled from.

#include <zygo/math/quaternion/quaternion.h>
#include <zygo/math/matrix/matrix.h>
#include <zygo/physics/i_physics_drawer.h>


namespace zygo {
namespace phys {

const uint DEFAULT_SHAPE_COLOR = 0xcccccc;


enum class ShapeType
{
  BOX,
  CYLINDER,
  CAPSULE,
  SPHERE,
};


class Shape
{
protected:
  ShapeType type;

  double mass;
  uint color;

  Vector3 local_pos;
  Vector3 size;
  Quaternion local_rot;

  // size layout per type:
  // BOX:      size_x = length, size_y = width,  size_z = height
  // CYLINDER: size_x = height, size_y = radius, size_z unused (axis along local Z)
  // CAPSULE:  size_x = size_y = radius, size_z = cylinder length (axis along local X)
  // SPHERE:   size_x = radius, the rest unused

public:
  Shape( ShapeType type, double mass, Vector3 const & local_pos, Vector3 const & size, Quaternion const & local_rot, uint color = DEFAULT_SHAPE_COLOR );
  virtual ~Shape() = default;

  inline ShapeType  getType () const { return type; }
  inline double     getMass () const { return mass; }
  inline uint       getColor() const { return color; }

  inline Vector3    const & localPos() const { return local_pos; }
  inline Quaternion const & localRot() const { return local_rot; }

  inline void addToLocalPos( Vector3 const & v ) { local_pos += v; }

  virtual Matrix calcLocalInertiaTensorForPart() const = 0;

  //
  virtual Vector3 calcLocalCenterOfMass() const;
  Vector3 calcWorldCenterOfMass( Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const;

  Quaternion calcWorldRotation( Quaternion const & obj_world_rot ) const;

  virtual void draw( IPhysicsDrawer const& drawer, Vector3 const & obj_world_pos, Quaternion const & obj_world_rot ) const = 0;
};

} // namespace phys
} // namespace zygo
