#pragma once

// Joint angle limits and a leg joint-angles triple.


namespace zygo {
namespace phys {

struct JointLimits
{
  double min_angle_rad;
  double max_angle_rad;

  JointLimits();
  JointLimits( double min_angle_deg, double max_angle_deg );

  bool insideLimits( double angle ) const;
  double angleError( double angle ) const;
};


// Robot-dog leg convention: q0 = abduction, q1 = hip, q2 = knee.
struct JointAngles
{
  double q0; // abduction
  double q1; // hip
  double q2; // knee

  JointAngles();

  static JointAngles fromDeg( double q0_deg, double q1_deg, double q2_deg );
  static JointAngles fromRad( double q0_rad, double q1_rad, double q2_rad );

private:
  JointAngles( double q0_rad, double q1_rad, double q2_rad );
};

} // namespace phys
} // namespace zygo
