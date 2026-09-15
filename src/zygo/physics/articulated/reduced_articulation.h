#pragma once

// Reduced-coordinate (joint-space) dynamics of one ArticulatedBody - the alternative to solving its hinges as constraints.
//
// WHY
//
// The sequential-impulse path keeps every body as 6 free DOF and pulls them back together with 5 constraint rows per hinge.
// Gauss-Seidel moves information one joint per iteration, and the robot is the worst case for it: a 5 kg trunk, a 1 kg shoulder block,
// an 80 g thigh and a 120 g shin in series, all of it closed through the ground at the foot.
// Stop the iterations early and what is left over shows up as a leg that stretches, sags and trembles.
//
// Here the state IS the joint angles: 6 DOF of the base plus one per hinge, 18 in total for the dog.
// A hinge cannot come apart because there is nothing that could come apart - the child's pose is computed from the parent's pose and the angle.
// What is left to iterate on are only the rows that really are inequalities or are torque-capped: the servos, the angle limits and the foot contacts,
// and those are solved against the EXACT joint-space inertia of the whole robot, so each of them sees the full mass it is pushing.
//
// HOW (one substep)
//
//   prepare()    read the maximal state back into q, u;  build the joint-space mass matrix M (18x18) from the world-frame body
//                Jacobians, and the generalized force: gravity, open-loop torques, Coriolis/centrifugal/gyroscopic terms;
//                u_free = u + h M^-1 F;  write the free body velocities, so contact detection sees the same thing it sees in the SI path
//   solve()      servos, limits, contacts: projected Gauss-Seidel in generalized velocities, warm started from the SAME accumulators
//                the SI path uses (HingeJoint motor impulse, ContactSphere impulses), so WorldState snapshots work unchanged
//   integrate()  semi-implicit Euler on q, forward kinematics for every body, body velocities from u
//   solvePositions()  the position phase: penetrating feet are pushed out by a joint-space displacement
//                (split impulse - velocities untouched, hinges still exact)
//
// M is built densely and factored by Cholesky. For 18 DOF that is cheaper and much simpler than Featherstone's ABA
// (articulated/aba.h), and it gives M^-1 J^T for the contact rows directly, which ABA does not.
//
// LIMITATIONS
//
//   - Trees only: every body reachable from the root through exactly one chain of hinges. The root may be static (a test rig).
//     build() returns false for anything else and PhysicsWorld then keeps using the SI path.
//   - HingeJoint::reactionForce()/reactionTorque() are not computed (they read zero). motorTorque() and limitTorque() are.
//   - The position phase only handles contacts: there is no joint drift to correct.
//
// The bodies stay the single source of truth between substeps: nothing is cached across calls except the model structure.
// Teleporting a body, restoring a snapshot or switching solvers mid-run just works - the next prepare() reads the new state
// (a slightly inconsistent pose, e.g. one produced by the SI path, is snapped onto the joints by the first integrate()).

#include <zygo/math/vector/vec3.h>
#include <zygo/math/matrix/small_fast_matrix/mat3.h>
#include <zygo/math/quaternion/quaternion.h>
#include <vector>


namespace zygo {
namespace phys {

class ArticulatedBody;
class RigidBody;
class HingeJoint;
struct ContactPoint;
struct ContactSphere;
struct SolverSettings;


class ReducedArticulation
{
private:
  struct Link
  {
    RigidBody  * body   = nullptr;
    HingeJoint * joint  = nullptr; // the hinge to the parent; nullptr for the root
    int          parent = -1;      // link index
    int          dof    = -1;      // index of this hinge's coordinate in u
    double       sign   = 1.0;     // HingeJoint angle = sign * theta (-1 when the joint's objA is the CHILD)

    // Hinge geometry, orthonormalized once in build(). _p: parent's body frame, _c: this body's frame.
    Vector3 anchor_p, axis_p, ref_p, ref2_p;
    Vector3 anchor_c, axis_c, ref_c, ref2_c;
    Quaternion q_rel; // R_child = R_parent * Rot(axis_p, theta) * R(q_rel)

    // Motion dependencies: every DOF whose velocity moves this body (the base DOF, then the hinges from the root down to this one).
    int dofs_begin = 0;
    int dofs_count = 0;

    // -------------------------- per-substep
    Mat3    R;
    Vector3 p;        // center of mass, world
    Vector3 w;        // angular velocity, world (consistent with u)
    Vector3 v;        // center of mass velocity, world (consistent with u)
    Vector3 axis;     // world hinge axis
    Vector3 anchor;   // world hinge anchor
    double  theta     = 0.0;
    double  theta_dot = 0.0;
  };

  // One scalar row of the velocity problem: j . u = target, lo <= lambda <= hi.
  struct Row
  {
    double * acc       = nullptr; // accumulated impulse, lives in the HingeJoint
    double   target    = 0.0;
    double   lo        = 0.0;
    double   hi        = 0.0;
    double   inv_d     = 0.0;
    double   threshold = 0.0;     // convergence threshold of |delta|
  };

  // A foot contact: normal, two tangents, spin - the same world-space accumulators as the SI path.
  struct ContactRows
  {
    ContactSphere * cs = nullptr;
    Vector3 n, t1, t2;
    double  target_n   = 0.0;
    double  inv_dn     = 0.0;
    double  inv_dt1    = 0.0;
    double  inv_dt2    = 0.0;
    double  inv_ds     = 0.0;     // 0 when spin friction is off
  };

  bool valid         = false;
  bool prepared      = false;     // prepare() factored M this substep
  bool fixed_base    = false;
  int  n             = 0;         // DOF count

  std::vector<Link> links;        // links[0] is the root; parents come before children
  std::vector<int>  dof_list;     // Link::dofs_begin/count index into this

  // DOF motion axes, refreshed every substep. Base translation: lin_dir. Everything else: rotation about ang through origin.
  std::vector<Vector3> dof_ang;
  std::vector<Vector3> dof_origin;
  std::vector<Vector3> dof_lin_dir;
  std::vector<char>    dof_is_lin;

  std::vector<double> M;          // n*n, Cholesky factor in the lower triangle after prepare()
  std::vector<double> u;          // generalized velocity
  std::vector<double> F;          // generalized force
  std::vector<double> tmp;

  std::vector<Row>         rows;          // servos and limits
  std::vector<ContactRows> contact_rows;
  std::vector<double>      J;             // Jacobian rows, rows_capacity * n
  std::vector<double>      W;             // M^-1 J^T, same layout
  int                      row_count = 0; // rows used in J/W this substep (servos, limits, then 4 per contact)

  struct PositionRow
  {
    double target = 0.0;
    double inv_k  = 0.0;
    double acc    = 0.0;
  };

  std::vector<PositionRow> pos_rows;
  std::vector<double>      z;     // pseudo-velocity of the position phase

  std::vector<Vector3> scratch_jv, scratch_jw, scratch_Ijw; // one link's Jacobian columns, sized to the deepest link

  std::vector<Vector3> a_bias;    // per link: bias linear acceleration
  std::vector<Vector3> alpha_bias;

public:
  // Builds the tree from the assembly. Must be called after finishInitializing().
  // Returns false for anything that is not a tree of hinges (the assembly then has to stay on the SI path).
  bool build( ArticulatedBody & ab );

  bool isValid() const { return valid; }
  int  dofCount() const { return n; }

  // The three phases of one substep - see the header comment.
  void prepare  ( SolverSettings const & s, double h );
  int  solve    ( SolverSettings const & s, double h, std::vector<ContactPoint> & contacts ); // returns the iterations used
  void integrate( double h );

  // Position phase: pushes penetrating feet out through the generalized coordinates (hinges stay exact).
  // Call after integrate(), with contacts re-detected at zero margin. Returns true while a correction above min_position_correction was applied.
  bool solvePositions( SolverSettings const & s, std::vector<ContactPoint> & contacts );

private:
  int  linkOf( RigidBody const * body ) const;

  void readState();
  void computeVelocitiesFromU( bool write_bodies );
  void updateDofAxes();
  void applyForwardKinematics();
  void buildMassMatrixAndForces( SolverSettings const & s );

  bool choleskyFactor();
  void choleskySolve( double * x ) const;

  // Fills j (size n, zeroed first) with the Jacobian of the velocity of world point x of link L along dir.
  void pointRow  ( int L, Vector3 const & x, Vector3 const & dir, double * j ) const;
  void angularRow( int L, Vector3 const & dir, double * j ) const;

  int    addRowStorage();            // returns row index into J/W
  double finishRow( int r );         // W = M^-1 J, returns d = J.W

  inline double dotRow( int r ) const;
  inline void   applyRow( int r, double lambda );
};

} // namespace phys
} // namespace zygo
