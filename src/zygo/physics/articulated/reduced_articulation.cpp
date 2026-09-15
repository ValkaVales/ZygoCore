#include "reduced_articulation.h"

#include <zygo/physics/body/articulated_body.h>
#include <zygo/physics/body/rigid_body.h>
#include <zygo/physics/joint/hinge_joint.h>
#include <zygo/physics/world/contact_point.h>
#include <zygo/physics/solver_settings.h>
#include <zygo/physics/phys_math.h>
#include <zygo/physics/phys_consts.h>

#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>
#include <algorithm>
#include <cmath>


namespace zygo {
namespace phys {

namespace
{
  // Gram-Schmidt of the stored hinge frame: ref made exactly perpendicular to axis, ref2 = axis x ref.
  void orthonormalFrame( Vector3 const & axis_in, Vector3 const & ref_in, Vector3 & axis, Vector3 & ref, Vector3 & ref2 )
  {
    axis = Vector3::safeNormalized( axis_in );
    ref  = Vector3::safeNormalized( ref_in - axis * (ref_in * axis) );
    ref2 = axis.crossProduct( ref );
  }

  inline double dotN( double const * a, double const * b, int n )
  {
    double s = 0.0;
    for ( int i = 0; i < n; ++i )
      s += a[i] * b[i];
    return s;
  }
}


// ------------------------------------------------------------------------------------------------ Build
bool ReducedArticulation::build( ArticulatedBody & ab )
{
  valid = false;
  links.clear();
  dof_list.clear();

  size_t const nb = ab.objects.size();
  size_t const nj = ab.joints.size();

  if ( nb == 0 || nj + 1 != nb )
    return false; // not a tree (or empty)

  auto bodyIndex = [&]( RigidBody const * b ) -> int
  {
    for ( size_t i = 0; i < nb; ++i )
      if ( ab.objects[i].get() == b )
        return (int)i;
    return -1;
  };

  // -------------------------- The root: the static body if there is one, otherwise the heaviest one
  int root = -1;
  int static_count = 0;

  for ( size_t i = 0; i < nb; ++i )
  {
    if ( ab.objects[i]->isStatic() )
    {
      ++static_count;
      root = (int)i;
    }
  }

  if ( static_count > 1 )
    return false; // two static bodies joined through the tree form a loop with the world

  fixed_base = ( static_count == 1 );

  if ( !fixed_base )
  {
    double best = -1.0;
    for ( size_t i = 0; i < nb; ++i )
    {
      if ( ab.objects[i]->getMass() > best )
      {
        best = ab.objects[i]->getMass();
        root = (int)i;
      }
    }
  }

  // -------------------------- Breadth-first over the hinges
  std::vector<int> link_of_body( nb, -1 );
  std::vector<char> joint_used( nj, 0 );

  Link root_link;
  root_link.body = ab.objects[(size_t)root].get();
  links.push_back( root_link );
  link_of_body[(size_t)root] = 0;

  n = fixed_base ? 0 : 6;

  for ( size_t head = 0; head < links.size(); ++head )
  {
    RigidBody * parent_body = links[head].body;

    for ( size_t j = 0; j < nj; ++j )
    {
      if ( joint_used[j] )
        continue;

      HingeJoint * hj = ab.joints[j].get();

      RigidBody * child_body = nullptr;
      bool parent_is_A = false;

      if ( hj->objA == parent_body )      { child_body = hj->objB; parent_is_A = true;  }
      else if ( hj->objB == parent_body ) { child_body = hj->objA; parent_is_A = false; }
      else continue;

      int const bi = bodyIndex( child_body );
      if ( bi < 0 || link_of_body[(size_t)bi] >= 0 || child_body->isStatic() )
        return false; // outside the assembly, a loop, or a static body that is not the root

      joint_used[j] = 1;

      Link L;
      L.body   = child_body;
      L.joint  = hj;
      L.parent = (int)head;
      L.dof    = n++;
      L.sign   = parent_is_A ? 1.0 : -1.0;

      Vector3 const & anchor_parent = parent_is_A ? hj->local_anchor_A : hj->local_anchor_B;
      Vector3 const & anchor_child  = parent_is_A ? hj->local_anchor_B : hj->local_anchor_A;
      Vector3 const & axis_parent   = parent_is_A ? hj->local_axis_A   : hj->local_axis_B;
      Vector3 const & axis_child    = parent_is_A ? hj->local_axis_B   : hj->local_axis_A;
      Vector3 const & ref_parent    = parent_is_A ? hj->local_ref_A    : hj->local_ref_B;
      Vector3 const & ref_child     = parent_is_A ? hj->local_ref_B    : hj->local_ref_A;

      L.anchor_p = anchor_parent;
      L.anchor_c = anchor_child;

      orthonormalFrame( axis_parent, ref_parent, L.axis_p, L.ref_p, L.ref2_p );
      orthonormalFrame( axis_child,  ref_child,  L.axis_c, L.ref_c, L.ref2_c );

      // R_child * F_c = Rot(axis_world, theta) * R_parent * F_p   =>   R_child = R_parent * Rot(axis_p, theta) * (F_p * F_c^T)
      Mat3 const F_p = Mat3::fromColumns( L.axis_p, L.ref_p, L.ref2_p );
      Mat3 const F_c = Mat3::fromColumns( L.axis_c, L.ref_c, L.ref2_c );

      L.q_rel = buildQuaternionFromMat3( F_p.mulTransposedRight( F_c ) );
      L.q_rel.normalize();

      link_of_body[(size_t)bi] = (int)links.size();
      links.push_back( L );
    }
  }

  if ( links.size() != nb )
    return false; // some body is not connected to the root

  // -------------------------- Motion dependencies: base DOF, then the hinges on the chain from the root
  for ( size_t i = 0; i < links.size(); ++i )
  {
    Link & L = links[i];
    L.dofs_begin = (int)dof_list.size();

    if ( !fixed_base )
      for ( int k = 0; k < 6; ++k )
        dof_list.push_back( k );

    // collect the chain root -> this, in increasing DOF order
    std::vector<int> chain;
    for ( int c = (int)i; c > 0; c = links[(size_t)c].parent )
      chain.push_back( links[(size_t)c].dof );

    for ( size_t k = chain.size(); k-- > 0; )
      dof_list.push_back( chain[k] );

    L.dofs_count = (int)dof_list.size() - L.dofs_begin;
  }

  size_t max_dofs = 0;
  for ( auto const & L : links )
    max_dofs = max2( max_dofs, (size_t)L.dofs_count );

  scratch_jv .assign( max_dofs, Vector3( 0, 0, 0 ) );
  scratch_jw .assign( max_dofs, Vector3( 0, 0, 0 ) );
  scratch_Ijw.assign( max_dofs, Vector3( 0, 0, 0 ) );

  dof_ang    .assign( (size_t)n, Vector3( 0, 0, 0 ) );
  dof_origin .assign( (size_t)n, Vector3( 0, 0, 0 ) );
  dof_lin_dir.assign( (size_t)n, Vector3( 0, 0, 0 ) );
  dof_is_lin .assign( (size_t)n, 0 );

  M  .assign( (size_t)n * (size_t)n, 0.0 );
  u  .assign( (size_t)n, 0.0 );
  z  .assign( (size_t)n, 0.0 );
  F  .assign( (size_t)n, 0.0 );
  tmp.assign( (size_t)n, 0.0 );

  a_bias    .assign( links.size(), Vector3( 0, 0, 0 ) );
  alpha_bias.assign( links.size(), Vector3( 0, 0, 0 ) );

  // Room for every servo, both limits of every hinge and 4 rows for every body touching the ground - no allocation in the hot path.
  size_t const max_rows = nj * 3 + nb * 4;
  rows.reserve( max_rows );
  contact_rows.reserve( nb );
  pos_rows.reserve( nb );
  J.assign( max_rows * (size_t)n, 0.0 );
  W.assign( max_rows * (size_t)n, 0.0 );

  valid = true;
  return true;
}

int ReducedArticulation::linkOf( RigidBody const * body ) const
{
  for ( size_t i = 0; i < links.size(); ++i )
    if ( links[i].body == body )
      return (int)i;
  return -1;
}


// ------------------------------------------------------------------------------------------------ state <-> bodies
void ReducedArticulation::readState()
{
  Link & root = links[0];
  root.R = buildMat3FromQuaternion( root.body->rotation_quaternion );
  root.p = root.body->center_of_mass_pos;

  if ( fixed_base )
  {
    root.w.reset();
    root.v.reset();
  }
  else
  {
    root.v = root.body->speed;
    root.w = root.body->angular_speed;

    u[0] = root.v.x;  u[1] = root.v.y;  u[2] = root.v.z;
    u[3] = root.w.x;  u[4] = root.w.y;  u[5] = root.w.z;
  }

  for ( size_t i = 1; i < links.size(); ++i )
  {
    Link & L = links[i];
    Link const & P = links[(size_t)L.parent];

    L.R = buildMat3FromQuaternion( L.body->rotation_quaternion );
    L.p = L.body->center_of_mass_pos;

    L.axis   = P.R * L.axis_p;
    L.anchor = P.p + P.R * L.anchor_p;

    // atan2 rather than acos: exact near zero, where the gait spends most of its time.
    Vector3 const ref_parent  = P.R * L.ref_p;
    Vector3 const ref2_parent = P.R * L.ref2_p;
    Vector3 const ref_child   = L.R * L.ref_c;

    L.theta     = std::atan2( ref_child * ref2_parent, ref_child * ref_parent );
    L.theta_dot = ( L.body->angular_speed - P.body->angular_speed ) * L.axis;

    u[(size_t)L.dof] = L.theta_dot;
  }
}

void ReducedArticulation::computeVelocitiesFromU( bool write_bodies )
{
  Link & root = links[0];

  if ( fixed_base )
  {
    root.w.reset();
    root.v.reset();
  } else
  {
    root.v = Vector3( u[0], u[1], u[2] );
    root.w = Vector3( u[3], u[4], u[5] );
  }

  if ( write_bodies && !fixed_base )
  {
    root.body->speed         = root.v;
    root.body->angular_speed = root.w;
  }

  for ( size_t i = 1; i < links.size(); ++i )
  {
    Link & L = links[i];
    Link const & P = links[(size_t)L.parent];

    L.theta_dot = u[(size_t)L.dof];

    L.w = P.w + L.axis * L.theta_dot;
    L.v = P.v + P.w.crossProduct( L.anchor - P.p ) + L.w.crossProduct( L.p - L.anchor );

    if ( write_bodies )
    {
      L.body->speed         = L.v;
      L.body->angular_speed = L.w;
    }
  }
}


// ------------------------------------------------------------------------------------------------ dynamics
void ReducedArticulation::updateDofAxes()
{
  if ( !fixed_base )
  {
    Vector3 const e[3] = { Vector3( 1, 0, 0 ), Vector3( 0, 1, 0 ), Vector3( 0, 0, 1 ) };

    for ( int k = 0; k < 3; ++k )
    {
      dof_is_lin [(size_t)k] = 1;
      dof_lin_dir[(size_t)k] = e[k];
      dof_ang    [(size_t)k].reset();

      dof_is_lin [(size_t)(3 + k)] = 0;
      dof_ang    [(size_t)(3 + k)] = e[k];
      dof_origin [(size_t)(3 + k)] = links[0].p;
    }
  }

  for ( size_t i = 1; i < links.size(); ++i )
  {
    Link const & L = links[i];
    dof_is_lin[(size_t)L.dof] = 0;
    dof_ang   [(size_t)L.dof] = L.axis;
    dof_origin[(size_t)L.dof] = L.anchor;
  }
}

void ReducedArticulation::buildMassMatrixAndForces( SolverSettings const & s )
{
  updateDofAxes();

  // ---- velocity-product (bias) accelerations, root to tips
  //   w_c = w_p + a qd                  a is fixed in the parent:  da/dt = w_p x a
  //   v_c = v_p + w_p x r1 + w_c x r2   r1 = anchor - p_p is fixed in the parent, r2 = p_c - anchor in the child
  // Differentiating with zero joint and base accelerations gives the terms below.
  alpha_bias[0].reset();
  a_bias    [0].reset();

  for ( size_t i = 1; i < links.size(); ++i )
  {
    Link const & L = links[i];
    Link const & P = links[(size_t)L.parent];

    Vector3 const r1 = L.anchor - P.p;
    Vector3 const r2 = L.p - L.anchor;

    alpha_bias[i] = alpha_bias[(size_t)L.parent] + P.w.crossProduct( L.axis * L.theta_dot );

    a_bias[i] = a_bias[(size_t)L.parent]
              + alpha_bias[(size_t)L.parent].crossProduct( r1 ) + P.w.crossProduct( P.w.crossProduct( r1 ) )
              + alpha_bias[i].crossProduct( r2 )                + L.w.crossProduct( L.w.crossProduct( r2 ) );
  }

  // ---- M = sum_i ( Jv^T m Jv + Jw^T I Jw ),   F = sum_i ( Jv^T (m g - m a_bias) - Jw^T (I alpha_bias + w x I w) )
  std::fill( M.begin(), M.end(), 0.0 );
  std::fill( F.begin(), F.end(), 0.0 );

  bool const gravity = s.gravity.enabled;
  Vector3 const g = s.gravity.g;

  Vector3 * const jv  = scratch_jv .data();
  Vector3 * const jw  = scratch_jw .data();
  Vector3 * const Ijw = scratch_Ijw.data();

  for ( size_t i = ( fixed_base ? 1 : 0 ); i < links.size(); ++i )
  {
    Link const & L = links[i];
    RigidBody const & b = *L.body;

    double const m = b.total_mass;
    Mat3 const & I = b.inertia_tensor_world;

    int const nd = L.dofs_count;
    int const * dofs = dof_list.data() + L.dofs_begin;

    for ( int a = 0; a < nd; ++a )
    {
      size_t const k = (size_t)dofs[a];

      if ( dof_is_lin[k] )
      {
        jw[a].reset();
        jv[a] = dof_lin_dir[k];
      } else
      {
        jw[a] = dof_ang[k];
        jv[a] = dof_ang[k].crossProduct( L.p - dof_origin[k] );
      }

      Ijw[a] = I * jw[a];
    }

    // dofs are in increasing order, so (a, b <= a) is the lower triangle
    for ( int a = 0; a < nd; ++a )
    {
      size_t const row = (size_t)dofs[a] * (size_t)n;

      for ( int c = 0; c <= a; ++c )
        M[row + (size_t)dofs[c]] += m * ( jv[a] * jv[c] ) + jw[a] * Ijw[c];
    }

    Vector3 f_lin = a_bias[i] * (-m);
    if ( gravity )
      f_lin += g * m;

    Vector3 const f_ang = -( I * alpha_bias[i] + L.w.crossProduct( I * L.w ) );

    for ( int a = 0; a < nd; ++a )
      F[(size_t)dofs[a]] += jv[a] * f_lin + jw[a] * f_ang;
  }

  // ---- open-loop actuator torques (HingeJoint MOTOR_TORQUE): a generalized force on the hinge coordinate
  for ( size_t i = 1; i < links.size(); ++i )
  {
    Link const & L = links[i];
    HingeJoint const & hj = *L.joint;

    if ( hj.motor_mode == HingeJoint::MOTOR_TORQUE )
      F[(size_t)L.dof] += L.sign * hj.motor_target_torque;
  }
}

bool ReducedArticulation::choleskyFactor()
{
  // In place, lower triangle: M = L L^T.
  double * a = M.data();

  for ( int j = 0; j < n; ++j )
  {
    double d = a[j * n + j];

    for ( int k = 0; k < j; ++k )
      d -= a[j * n + k] * a[j * n + k];

    if ( !( d > 0.0 ) )
      return false;

    d = std::sqrt( d );
    a[j * n + j] = d;

    double const inv = 1.0 / d;

    for ( int i = j + 1; i < n; ++i )
    {
      double s = a[i * n + j];

      for ( int k = 0; k < j; ++k )
        s -= a[i * n + k] * a[j * n + k];

      a[i * n + j] = s * inv;
    }
  }

  return true;
}

void ReducedArticulation::choleskySolve( double * x ) const
{
  double const * a = M.data();

  for ( int i = 0; i < n; ++i )
  {
    double s = x[i];
    for ( int k = 0; k < i; ++k )
      s -= a[i * n + k] * x[k];
    x[i] = s / a[i * n + i];
  }

  for ( int i = n - 1; i >= 0; --i )
  {
    double s = x[i];
    for ( int k = i + 1; k < n; ++k )
      s -= a[k * n + i] * x[k];
    x[i] = s / a[i * n + i];
  }
}

void ReducedArticulation::prepare( SolverSettings const & s, double h )
{
  ZgAssert( valid );

  readState();
  computeVelocitiesFromU( false ); // link velocities consistent with u, for the bias terms
  buildMassMatrixAndForces( s );

  // M is SPD by construction for any model build() accepts, so a failure here means the state itself is already non-finite.
  // Freeze this assembly for the substep instead of crashing a long training run; a Debug build still stops right here.
  prepared = choleskyFactor();
  ZgAssert( prepared );

  if ( !prepared )
    return;

  // u_free = u + h M^-1 F
  for ( int i = 0; i < n; ++i )
    tmp[(size_t)i] = F[(size_t)i] * h;

  choleskySolve( tmp.data() );

  for ( int i = 0; i < n; ++i )
    u[(size_t)i] += tmp[(size_t)i];

  // The free velocities go into the bodies now: detectContacts() samples them for the speculative margin and the approach speed,
  // exactly as it samples the post-gravity velocities in the SI path.
  computeVelocitiesFromU( true );
}


// ------------------------------------------------------------------------------------------------ rows
void ReducedArticulation::pointRow( int li, Vector3 const & x, Vector3 const & dir, double * j ) const
{
  std::fill( j, j + n, 0.0 );

  Link const & L = links[(size_t)li];
  int const * dofs = dof_list.data() + L.dofs_begin;

  for ( int a = 0; a < L.dofs_count; ++a )
  {
    size_t const k = (size_t)dofs[a];

    if ( dof_is_lin[k] )
      j[k] = dir * dof_lin_dir[k];
    else
      j[k] = dir * dof_ang[k].crossProduct( x - dof_origin[k] );
  }
}

void ReducedArticulation::angularRow( int li, Vector3 const & dir, double * j ) const
{
  std::fill( j, j + n, 0.0 );

  Link const & L = links[(size_t)li];
  int const * dofs = dof_list.data() + L.dofs_begin;

  for ( int a = 0; a < L.dofs_count; ++a )
  {
    size_t const k = (size_t)dofs[a];

    if ( !dof_is_lin[k] )
      j[k] = dir * dof_ang[k];
  }
}

int ReducedArticulation::addRowStorage()
{
  int const r = row_count++;
  ZgAssert( (size_t)( row_count * n ) <= J.size() );
  return r;
}

double ReducedArticulation::finishRow( int r )
{
  double const * j = J.data() + (size_t)r * (size_t)n;
  double       * w = W.data() + (size_t)r * (size_t)n;

  std::copy( j, j + n, w );
  choleskySolve( w );

  return dotN( j, w, n );
}

inline double ReducedArticulation::dotRow( int r ) const
{
  return dotN( J.data() + (size_t)r * (size_t)n, u.data(), n );
}

inline void ReducedArticulation::applyRow( int r, double lambda )
{
  double const * w = W.data() + (size_t)r * (size_t)n;

  for ( int i = 0; i < n; ++i )
    u[(size_t)i] += w[i] * lambda;
}


// ------------------------------------------------------------------------------------------------ solve
int ReducedArticulation::solve( SolverSettings const & s, double h, std::vector<ContactPoint> & contacts )
{
  ZgAssert( valid );

  if ( !prepared )
    return 0;

  rows.clear();
  contact_rows.clear();
  row_count = 0;

  // ================================================================ Servos and limits (rows 0 .. rows.size()-1 map 1:1 to J/W)
  for ( size_t i = 1; i < links.size(); ++i )
  {
    Link const & L = links[i];
    HingeJoint & hj = *L.joint;

    hj.last_substep_dt = h;
    hj.accumulated_lower_limit_impulse = 0.0;
    hj.accumulated_upper_limit_impulse = 0.0;

    // There are no anchor/axis rows here. Zeroing them keeps reactionForce()/reactionTorque() honest (they read zero, see the header),
    // and a later switch back to the SI path starts cold instead of warm-starting from an impulse that is many substeps old.
    hj.accumulated_anchor_impulse.reset();
    hj.accumulated_axis_impulse  .reset();

    double const hinge_angle = L.sign * L.theta;
    double const hinge_rate  = L.sign * u[(size_t)L.dof]; // free velocity

    // -------------------------- Servo: the same law as HingeJoint::solveMotorVelocityConstraint()
    if ( hj.motor_mode == HingeJoint::MOTOR_VELOCITY || hj.motor_mode == HingeJoint::MOTOR_POSITION )
    {
      Row row;
      row.acc = &hj.accumulated_motor_impulse;
      row.hi  = hj.motor_max_torque * h;
      row.lo  = -row.hi;
      row.threshold = s.motor.min_error_for_impulse;

      if ( hj.motor_mode == HingeJoint::MOTOR_VELOCITY )
      {
        row.target = hj.motor_target_velocity;
      }
      else
      {
        double bias = s.motor.position_erp * ( hinge_angle - hj.motor_target_angle ) / h;
        toRange( bias, -hj.motor_max_velocity, hj.motor_max_velocity );
        row.target = -bias;
      }

      int const r = addRowStorage();
      double * j = J.data() + (size_t)r * (size_t)n;
      std::fill( j, j + n, 0.0 );
      j[(size_t)L.dof] = L.sign;

      row.inv_d = 1.0 / ( finishRow( r ) + s.motor.softness );

      // warm start - same scaling and clamping as HingeJoint::warmStartVelocitySolve()
      if ( s.joints.warm_starting && s.joints.motor_warm_starting )
      {
        double const dt_ratio = ( hj.accumulated_impulse_dt > 0.0 ) ? ( h / hj.accumulated_impulse_dt ) : 0.0;
        hj.accumulated_motor_impulse *= dt_ratio * s.joints.warm_start_factor;
        toRange( hj.accumulated_motor_impulse, row.lo, row.hi );
      }
      else
      {
        hj.accumulated_motor_impulse = 0.0;
      }

      hj.accumulated_impulse_dt = h;
      rows.push_back( row );
    }
    else
    {
      hj.accumulated_motor_impulse = 0.0;
    }

    // -------------------------- Limits: a one-sided row per side that can be reached within this substep
    if ( hj.limit_enabled )
    {
      double const reach = std::abs( hinge_rate ) * h * 2.0 + s.limits.slop;

      for ( int side = 0; side < 2; ++side )
      {
        double const C = ( side == 0 ) ? ( hinge_angle - hj.limits.min_angle_rad ) : ( hj.limits.max_angle_rad - hinge_angle );

        if ( C > reach )
          continue;

        Row row;
        row.acc = ( side == 0 ) ? &hj.accumulated_lower_limit_impulse : &hj.accumulated_upper_limit_impulse;
        row.lo  = 0.0;
        row.hi  = s.limits.max_impulse;
        row.threshold = s.limits.min_error_for_impulse;

        // Inside: approach no faster than closes the gap. Violated: Baumgarte recovery.
        row.target = -C / h;
        if ( C < -s.limits.slop )
          row.target *= s.limits.beta;

        int const r = addRowStorage();
        double * j = J.data() + (size_t)r * (size_t)n;
        std::fill( j, j + n, 0.0 );
        j[(size_t)L.dof] = ( side == 0 ) ? L.sign : -L.sign;

        row.inv_d = 1.0 / ( finishRow( r ) + s.limits.softness );
        rows.push_back( row );
      }
    }
  }

  int const joint_rows = row_count;

  // ================================================================ Contacts (4 rows each: n, t1, t2, spin)
  for ( auto & c : contacts )
  {
    int const li = linkOf( c.sphere->body );
    if ( li < 0 )
      continue; // another assembly's foot

    ContactSphere & cs = *c.sphere;

    ContactRows cr;
    cr.cs = &cs;
    cr.n  = c.normal;
    cr.t1 = c.t1;
    cr.t2 = c.t2;

    int const rn = addRowStorage();
    pointRow( li, c.point, c.normal, J.data() + (size_t)rn * (size_t)n );
    double const dn = finishRow( rn );

    int const rt1 = addRowStorage();
    pointRow( li, c.point, c.t1, J.data() + (size_t)rt1 * (size_t)n );
    double const dt1 = finishRow( rt1 );

    int const rt2 = addRowStorage();
    pointRow( li, c.point, c.t2, J.data() + (size_t)rt2 * (size_t)n );
    double const dt2 = finishRow( rt2 );

    int const rs = addRowStorage();
    angularRow( li, c.normal, J.data() + (size_t)rs * (size_t)n );
    double const ds = finishRow( rs );

    cr.inv_dn  = ( dn  > PHYS_EPSILON ) ? 1.0 / dn  : 0.0;
    cr.inv_dt1 = ( dt1 > PHYS_EPSILON ) ? 1.0 / dt1 : 0.0;
    cr.inv_dt2 = ( dt2 > PHYS_EPSILON ) ? 1.0 / dt2 : 0.0;
    cr.inv_ds  = ( s.contacts.spin_friction_mu > 0.0 && ds > PHYS_EPSILON ) ? 1.0 / ds : 0.0;

    // -------------------------- The normal target, identical to PhysicsWorld::solveContactsCollisionOnce()
    if ( c.separation > 0.0 )
    {
      cr.target_n = -c.separation / h; // speculative: do not end the substep below the surface
    }
    else
    {
      double bias = 0.0;
      double const penetration = c.penetration - s.contacts.slop;

      if ( penetration > 0.0 )
      {
        bias = s.contacts.baumgarte_beta * penetration / h;
        applyMax( bias, s.contacts.max_bias_speed );
      }

      double restitution_speed = 0.0;
      if ( s.contacts.restitution > 0.0 && cs.impact_vn < -s.contacts.restitution_velocity_threshold )
        restitution_speed = -s.contacts.restitution * cs.impact_vn;

      cr.target_n = max2( bias, restitution_speed );
    }

    contact_rows.push_back( cr );
  }

  // ================================================================ Warm start
  for ( size_t r = 0; r < rows.size(); ++r )
    if ( *rows[r].acc != 0.0 )
      applyRow( (int)r, *rows[r].acc );

  for ( size_t k = 0; k < contact_rows.size(); ++k )
  {
    ContactRows const & cr = contact_rows[k];
    ContactSphere const & cs = *cr.cs;
    int const base = joint_rows + (int)k * 4;

    if ( cs.accumulated_normal_impulse != 0.0 )
      applyRow( base, cs.accumulated_normal_impulse );

    if ( !cs.accumulated_tangent_impulse.isZeroVector( PHYS_EPSILON ) )
    {
      applyRow( base + 1, cs.accumulated_tangent_impulse * cr.t1 );
      applyRow( base + 2, cs.accumulated_tangent_impulse * cr.t2 );
    }

    if ( cs.accumulated_spin_impulse != 0.0 && cr.inv_ds > 0.0 )
      applyRow( base + 3, cs.accumulated_spin_impulse );
  }

  // ================================================================ Projected Gauss-Seidel
  int const max_iterations = max2( 1, s.step.max_velocity_iterations );
  int const min_iterations = max2( 1, s.step.min_velocity_iterations );

  int it = 0;

  for ( ; it < max_iterations; ++it )
  {
    bool has_error = false;

    // -------------------------- Servos and limits
    for ( size_t r = 0; r < rows.size(); ++r )
    {
      Row const & row = rows[r];

      double const lambda = ( row.target - dotRow( (int)r ) ) * row.inv_d;

      double const old_acc = *row.acc;
      double new_acc = old_acc + lambda;
      toRange( new_acc, row.lo, row.hi );
      *row.acc = new_acc;

      double const delta = new_acc - old_acc;

      if ( delta != 0.0 )
        applyRow( (int)r, delta );

      if ( std::abs( delta ) > row.threshold )
        has_error = true;
    }

    // -------------------------- Contacts
    for ( size_t k = 0; k < contact_rows.size(); ++k )
    {
      ContactRows const & cr = contact_rows[k];
      ContactSphere & cs = *cr.cs;
      int const base = joint_rows + (int)k * 4;

      // normal (one-sided)
      if ( cr.inv_dn > 0.0 )
      {
        double const lambda = ( cr.target_n - dotRow( base ) ) * cr.inv_dn;

        double const old_acc = cs.accumulated_normal_impulse;
        double new_acc = old_acc + lambda;
        applyMin( new_acc, 0.0 );
        cs.accumulated_normal_impulse = new_acc;

        double const delta = new_acc - old_acc;

        if ( delta != 0.0 )
          applyRow( base, delta );

        if ( std::abs( delta ) > s.contacts.min_error_for_collision_impulse )
          has_error = true;
      }

      // friction: circular cone, accumulated as one world vector (as in PhysicsWorld::solveContactsFrictionOnce())
      double max_friction = s.contacts.friction_mu * cs.accumulated_normal_impulse;
      applyMin( max_friction, 0.0 );

      auto setProjected = [&]( Vector3 wanted ) -> bool
      {
        Vector3 const old_impulse = cs.accumulated_tangent_impulse;

        wanted -= cr.n * ( wanted * cr.n );
        wanted.limitLength( max_friction );

        cs.accumulated_tangent_impulse = wanted;

        Vector3 const delta = wanted - old_impulse;

        double const d1 = delta * cr.t1;
        double const d2 = delta * cr.t2;

        if ( d1 != 0.0 ) applyRow( base + 1, d1 );
        if ( d2 != 0.0 ) applyRow( base + 2, d2 );

        return delta.length() > s.contacts.min_error_for_friction_impulse;
      };

      if ( setProjected( cs.accumulated_tangent_impulse ) )
        has_error = true;

      if ( max_friction > 0.0 )
      {
        if ( cr.inv_dt1 > 0.0 )
        {
          double const lambda = -dotRow( base + 1 ) * cr.inv_dt1;
          if ( setProjected( cs.accumulated_tangent_impulse + cr.t1 * lambda ) )
            has_error = true;
        }

        if ( cr.inv_dt2 > 0.0 )
        {
          double const lambda = -dotRow( base + 2 ) * cr.inv_dt2;
          if ( setProjected( cs.accumulated_tangent_impulse + cr.t2 * lambda ) )
            has_error = true;
        }
      }

      // spin
      if ( cr.inv_ds > 0.0 )
      {
        double max_spin = s.contacts.spin_friction_mu * cs.radius * cs.accumulated_normal_impulse;
        applyMin( max_spin, 0.0 );

        double const old_spin = cs.accumulated_spin_impulse;
        double new_spin = old_spin - dotRow( base + 3 ) * cr.inv_ds;
        toRange( new_spin, -max_spin, max_spin );
        cs.accumulated_spin_impulse = new_spin;

        double const delta = new_spin - old_spin;

        if ( delta != 0.0 )
          applyRow( base + 3, delta );

        if ( std::abs( delta ) > s.contacts.min_error_for_friction_impulse )
          has_error = true;
      }
    }

    if ( s.step.velocity_early_out && !has_error && ( it + 1 ) >= min_iterations )
    {
      ++it;
      break;
    }
  }

  return min2( it, max_iterations );
}


// ------------------------------------------------------------------------------------------------ Integrate
void ReducedArticulation::integrate( double h )
{
  ZgAssert( valid );

  if ( !prepared )
    return;

  if ( !fixed_base )
  {
    Vector3 const v( u[0], u[1], u[2] );
    Vector3 const w( u[3], u[4], u[5] );

    RigidBody & root = *links[0].body;

    root.center_of_mass_pos += v * h;

    Quaternion const dq = Quaternion::calcRotationQuaternion_fromAngularVelocity( w, h );
    root.rotation_quaternion.rotateByRotationQuaternion( dq ); // normalizes
  }

  for ( size_t i = 1; i < links.size(); ++i )
    links[i].theta += u[(size_t)links[i].dof] * h;

  applyForwardKinematics();
}

void ReducedArticulation::applyForwardKinematics()
{
  Link & root = links[0];
  root.p = root.body->center_of_mass_pos;
  root.R = buildMat3FromQuaternion( root.body->rotation_quaternion );

  // Every child is placed exactly on its hinge.
  for ( size_t i = 1; i < links.size(); ++i )
  {
    Link & L = links[i];
    Link const & P = links[(size_t)L.parent];
    RigidBody & b = *L.body;

    Quaternion const q_hinge = Quaternion::calcRotationQuaternion( L.axis_p, L.theta );

    Quaternion q = P.body->rotation_quaternion.crossProduct( q_hinge ).crossProduct( L.q_rel );
    q.normalize();

    b.rotation_quaternion = q;

    L.R      = buildMat3FromQuaternion( q );
    L.axis   = P.R * L.axis_p;
    L.anchor = P.p + P.R * L.anchor_p;
    L.p      = L.anchor - L.R * L.anchor_c;

    b.center_of_mass_pos = L.p;
  }

  // Body velocities from u at the NEW configuration - the next readState() recovers exactly this u.
  computeVelocitiesFromU( true );

  // The second normalization is not about accuracy. RigidBody::restoreState() normalizes the quaternion it is given, and normalizing
  // is not bit-idempotent after a single pass - so a snapshot restored mid-run would start from a pose that differs in the last bits,
  // and an RL episode replayed from it would not be the same episode. Twice (as ArticulatedBody::finalizeStep() effectively does for
  // the SI path) lands on a value the restore leaves alone.
  for ( auto & L : links )
  {
    L.body->normalizeQuaternion();
    L.body->updateWorldInertia();
  }
}


// ------------------------------------------------------------------------------------------------ position phase
bool ReducedArticulation::solvePositions( SolverSettings const & s, std::vector<ContactPoint> & contacts )
{
  ZgAssert( valid );

  if ( !prepared )
    return false;

  // The joint-space twin of PhysicsWorld::solveContactPositionsOnce(): a pseudo-velocity z that moves the penetrating feet out,
  // integrated straight into the pose and never into u. The hinges stay exact because z is a joint-space displacement.
  //
  // Needed for hard landings: a foot on a leg that folds at tens of rad/s does not travel in a straight line during a substep,
  // so the speculative velocity row alone lets it end inside the ground. Dropped from 30 cm, one 5 ms substep: 19 mm without this, 1.5 mm with it.
  //
  // The price, at ONE substep only: every touchdown ends a millimetre or two deep, the correction lifts the robot out in a single jump,
  // and the trunk jitter of a walking robot goes from 35 to ~90 um. At two substeps the phase does nothing measurable while walking.
  // So: two substeps, or position_iterations = 0 when running one substep on flat ground.
  //
  // (Moving only the six base DOF instead - the whole robot lifted as one rigid composite, joint angles untouched - was tried:
  //  the jitter at one substep is the same, and landings end deeper, so the plain joint-space version stays.)
  //
  // M is the factor from prepare(), i.e. one substep stale - more than good enough for a correction of a millimetre or two.
  row_count = 0;
  pos_rows.clear();

  updateDofAxes(); // J at the configuration integrate() has just produced


  for ( auto & c : contacts )
  {
    double const penetration = c.penetration - s.contacts.position_slop;
    if ( penetration <= 0.0 )
      continue;

    int const li = linkOf( c.sphere->body );
    if ( li < 0 )
      continue;

    double correction = s.contacts.position_beta * penetration;
    applyMax( correction, s.contacts.max_position_correction );

    int const r = addRowStorage();
    pointRow( li, c.point, c.normal, J.data() + (size_t)r * (size_t)n );
    double const k = finishRow( r );

    if ( k <= PHYS_EPSILON )
    {
      --row_count;
      continue;
    }

    PositionRow pr;
    pr.target = correction;
    pr.inv_k  = 1.0 / k;
    pr.acc    = 0.0;
    pos_rows.push_back( pr );
  }

  if ( pos_rows.empty() )
    return false;

  // A few Gauss-Seidel sweeps on the linear model, one-sided: a foot can only be pushed out.
  std::fill( z.begin(), z.end(), 0.0 );

  const int sweeps = 4;
  double max_correction = 0.0;

  for ( int sweep = 0; sweep < sweeps; ++sweep )
  {
    for ( size_t r = 0; r < pos_rows.size(); ++r )
    {
      PositionRow & pr = pos_rows[r];

      double const * j = J.data() + r * (size_t)n;
      double const * w = W.data() + r * (size_t)n;

      double const lambda = ( pr.target - dotN( j, z.data(), n ) ) * pr.inv_k;

      double const old_acc = pr.acc;
      double new_acc = old_acc + lambda;
      applyMin( new_acc, 0.0 );
      pr.acc = new_acc;

      double const delta = new_acc - old_acc;

      for ( int i = 0; i < n; ++i )
        z[(size_t)i] += w[i] * delta;
    }
  }

  for ( auto const & pr : pos_rows )
    max_correction = max2( max_correction, pr.target );

  // -------------------------- Apply z as a displacement of the generalized coordinates
  double const max_angle = s.contacts.max_position_angular_correction;

  if ( !fixed_base )
  {
    RigidBody & root = *links[0].body;
    root.center_of_mass_pos += Vector3( z[0], z[1], z[2] );

    Vector3 rot( z[3], z[4], z[5] );
    rot.limitLength( max_angle );

    double const angle = rot.length();
    if ( angle > PHYS_EPSILON )
      root.rotation_quaternion.rotateByRotationQuaternion( Quaternion::calcRotationQuaternion( rot / angle, angle ) );
  }

  for ( size_t i = 1; i < links.size(); ++i )
  {
    double dq = z[(size_t)links[i].dof];
    toRange( dq, -max_angle, max_angle );
    links[i].theta += dq;
  }

  applyForwardKinematics();

  return max_correction > s.contacts.min_position_correction;
}

} // namespace phys
} // namespace zygo
