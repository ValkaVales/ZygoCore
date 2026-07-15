#pragma once

// ZygoPhys: impulse-based rigid body physics for robot simulation.
//
// Units convention:
//   - The CONSTRUCTION API works in robot-CAD units: masses in grams, distances in millimeters
//     (RigidBody::add*, ArticulatedBody::createJoint anchor, PhysicsWorld::addContactSphere).
//     Values are converted to SI once, at the call boundary.
//   - Everything else is strictly SI: kilograms, meters, seconds, radians, N*m.
//     Terrain, gravity, velocities, impulses and all solver internals live in SI.
//
// Precision: the solver uses double throughout (not Real) - iterative impulse solvers
// degrade quickly in single precision, so this does not follow a possible Real = float switch.
//
// Rendering: the engine does not depend on any graphics library. Debug drawing goes through
// the IPhysicsDrawer interface (i_physics_draw.h); a ZygoGL implementation lives outside
// ZygoCore (see zygogl/debug/phys_draw.h).

#include <zygo/physics/phys_consts.h>
#include <zygo/physics/i_physics_drawer.h>
#include <zygo/physics/shape/shape.h>
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/body/articulated_body.h>
#include <zygo/physics/joint/hinge_joint.h>
#include <zygo/physics/terrain/terrain.h>
#include <zygo/physics/world/physics_world.h>
