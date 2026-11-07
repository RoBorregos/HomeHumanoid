// Leg-inertial odometry for the G1, from rt/lowstate only.
//
// MuJoCo's ground-truth pose (rt/sportmodestate) does not exist on the real G1:
// unitree_hg's SportModeState_ has no pose (docs/plan.md, open question 5). This
// estimates the pelvis pose from what both the sim and the robot publish on
// rt/lowstate: the 12 leg joint angles and the pelvis IMU orientation.
//
// Method (stance-pivot anchoring):
//   - Orientation comes from the IMU quaternion, re-based so the first sample
//     has yaw = initial_yaw. In MuJoCo that quaternion is exact; on the robot
//     its yaw is gyro-integrated and drifts, which this does not correct.
//   - Forward kinematics gives 4 sole contact points per foot (the 8 spheres of
//     the MuJoCo model) relative to the pelvis, rotated into the odom frame.
//   - The lowest point is taken as the pivot touching the ground, and is assumed
//     not to move: pelvis = anchor - R * p_pivot. When another point becomes
//     lower by more than switch_margin, the anchor moves to it, keeping the
//     pelvis position continuous. Using the lowest corner rather than the sole's
//     center follows the heel-to-toe roll instead of counting it as motion.
//   - Velocity is a low-passed finite difference of that position.
// Errors come from foot slip and yaw drift; there is no loop closure, so on the
// real robot this is meant to be corrected by a localizer.
//
// Needs a foot on the ground from the first sample: with the robot lifted (the
// MuJoCo elastic band, or held on the real robot) the lowest point swings in the
// air and is still taken as fixed, which drifts fast. reset() starts over at the
// initial pose, for once the robot stands.
//
// Assumes flat ground: a pivot on a step would be anchored at the wrong height,
// which only affects z, not the planar pose.
//
// Header-only so g1_rl_bridge can run it inside its own DDS participant: in the
// sim, rt/lowstate lives on loopback without multicast, where each extra
// participant takes one of a few auto indices shared with every other DDS
// process on domain 0.
#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include "unitree/idl/hg/LowState_.hpp"

namespace g1_leg_odom {

struct Pose {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();    // Odom frame; z is pelvis height over the pivot.
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();    // Odom frame.
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();  // Body frame, from the gyro.
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  long pivot_switches = 0;
};

class LegOdometry {
public:
  struct Params {
    double initial_x = 0, initial_y = 0, initial_yaw = 0;
    double switch_margin = 0.005;      // m
    double velocity_filter_tau = 0.1;  // s
  };

  explicit LegOdometry(const Params & p) : p_(p) {}

  bool initialized() const { return init_; }
  // Starts over at the initial pose on the next sample.
  void reset() {
    init_ = false;
    pose_ = Pose();
  }
  const Pose & pose() const { return pose_; }

  // Returns false if the sample was unusable (IMU not filled in yet).
  bool update(const unitree_hg::msg::dds_::LowState_ & s, std::chrono::steady_clock::time_point now) {
    using Eigen::Quaterniond;
    using Eigen::Vector3d;
    const auto & qi = s.imu_state().quaternion();
    Quaterniond q_imu(qi[0], qi[1], qi[2], qi[3]);
    if (q_imu.norm() < 0.5) return false;
    q_imu.normalize();

    if (!init_) {
      const Vector3d fwd = q_imu * Vector3d::UnitX();
      const double yaw = std::atan2(fwd.y(), fwd.x());
      yaw_offset_ = Quaterniond(Eigen::AngleAxisd(p_.initial_yaw - yaw, Vector3d::UnitZ()));
    }
    const Quaterniond q = yaw_offset_ * q_imu;
    const Eigen::Matrix3d rot = q.toRotationMatrix();

    // Contact points relative to the pelvis, in the odom frame's orientation.
    std::array<Vector3d, kPoints> pts;
    for (int f = 0; f < 2; ++f) {
      const Eigen::Isometry3d t = foot_fk(f == 0, s);
      for (int c = 0; c < 4; ++c) pts[f * 4 + c] = rot * (t * sole()[c]);
    }
    int lowest = 0;
    for (int i = 1; i < kPoints; ++i)
      if (pts[i].z() < pts[lowest].z()) lowest = i;

    Vector3d & pos = pose_.pos;
    if (!init_) {
      pivot_ = lowest;
      pos = Vector3d(p_.initial_x, p_.initial_y, -pts[pivot_].z());  // Pivot on z=0.
      anchor_ = pos + pts[pivot_];
      last_t_ = now;
      init_ = true;
    } else if (lowest != pivot_ && pts[lowest].z() < pts[pivot_].z() - p_.switch_margin) {
      anchor_ = (anchor_ - pts[pivot_]) + pts[lowest];  // Current estimate, no jump.
      pivot_ = lowest;
      ++pose_.pivot_switches;
    }
    const Vector3d prev = pos;
    pos = anchor_ - pts[pivot_];
    const double dt = std::chrono::duration<double>(now - last_t_).count();
    last_t_ = now;
    if (dt > 1e-4 && dt < 0.1) {
      const double a = dt / (p_.velocity_filter_tau + dt);
      pose_.vel = (1 - a) * pose_.vel + a * (pos - prev) / dt;
    }
    pose_.q = q;
    const auto & g = s.imu_state().gyroscope();
    pose_.omega = Vector3d(g[0], g[1], g[2]);
    return true;
  }

private:
  static constexpr int kLegJoints = 6, kPoints = 8;

  struct Link {
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;  // w, x, y, z
    Eigen::Vector3d axis;
  };

  // Pelvis -> ankle_roll_link, from unitree_mujoco's g1_29dof.xml (same as
  // unitree_ros' g1_29dof.urdf). Order matches LowState motor_state 0-5 (left)
  // and 6-11 (right): hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll.
  static const std::array<Link, kLegJoints> & leg(bool left) {
    static const std::array<Link, kLegJoints> l = {{
      {{0, 0.064452, -0.1027}, {1, 0, 0, 0}, {0, 1, 0}},
      {{0, 0.052, -0.030465}, {0.996179, 0, -0.0873386, 0}, {1, 0, 0}},
      {{0.025001, 0, -0.12412}, {1, 0, 0, 0}, {0, 0, 1}},
      {{-0.078273, 0.0021489, -0.17734}, {0.996179, 0, 0.0873386, 0}, {0, 1, 0}},
      {{0, -9.4445e-05, -0.30001}, {1, 0, 0, 0}, {0, 1, 0}},
      {{0, 0, -0.017558}, {1, 0, 0, 0}, {1, 0, 0}},
    }};
    static const std::array<Link, kLegJoints> r = [] {
      auto m = l;  // The right leg differs only in the sign of the y offsets.
      for (auto & k : m) k.pos.y() = -k.pos.y();
      return m;
    }();
    return left ? l : r;
  }

  // Sole spheres in ankle_roll_link: centers at z=-0.03, radius 0.005.
  static const std::array<Eigen::Vector3d, 4> & sole() {
    static const std::array<Eigen::Vector3d, 4> s = {{
      {-0.05, 0.025, -0.035}, {-0.05, -0.025, -0.035}, {0.12, 0.03, -0.035}, {0.12, -0.03, -0.035},
    }};
    return s;
  }

  static Eigen::Isometry3d foot_fk(bool left, const unitree_hg::msg::dds_::LowState_ & s) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    const int base = left ? 0 : kLegJoints;
    const auto & chain = leg(left);
    for (int i = 0; i < kLegJoints; ++i) {
      const double q = s.motor_state()[base + i].q();
      t = t * Eigen::Translation3d(chain[i].pos) * chain[i].quat.normalized() *
          Eigen::AngleAxisd(q, chain[i].axis);
    }
    return t;
  }

  Params p_;
  bool init_ = false;
  int pivot_ = 0;
  Eigen::Vector3d anchor_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond yaw_offset_ = Eigen::Quaterniond::Identity();
  std::chrono::steady_clock::time_point last_t_;
  Pose pose_;
};

// Planar odometry for Nav2: yaw only in the pose and TF, since the walking sway
// in roll and pitch would tilt the costmaps' view of the robot.
inline void to_msgs(const Pose & p, const rclcpp::Time & stamp, const std::string & odom_frame,
                    const std::string & base_frame, nav_msgs::msg::Odometry & o,
                    geometry_msgs::msg::TransformStamped & t) {
  const Eigen::Vector3d fwd = p.q * Eigen::Vector3d::UnitX();
  const double yaw = std::atan2(fwd.y(), fwd.x());
  o = nav_msgs::msg::Odometry();
  o.header.stamp = stamp;
  o.header.frame_id = odom_frame;
  o.child_frame_id = base_frame;
  o.pose.pose.position.x = p.pos.x();
  o.pose.pose.position.y = p.pos.y();
  o.pose.pose.position.z = p.pos.z();  // Height over the pivot. Not in the TF.
  o.pose.pose.orientation.z = std::sin(yaw / 2);
  o.pose.pose.orientation.w = std::cos(yaw / 2);
  const double c = std::cos(yaw), s = std::sin(yaw);
  o.twist.twist.linear.x = c * p.vel.x() + s * p.vel.y();
  o.twist.twist.linear.y = -s * p.vel.x() + c * p.vel.y();
  o.twist.twist.angular.z = p.omega.z();
  // Rough and uncalibrated: the real error grows with distance walked.
  o.pose.covariance[0] = o.pose.covariance[7] = o.pose.covariance[35] = 0.01;
  o.pose.covariance[14] = o.pose.covariance[21] = o.pose.covariance[28] = 1e3;
  o.twist.covariance[0] = o.twist.covariance[7] = o.twist.covariance[35] = 0.01;

  t = geometry_msgs::msg::TransformStamped();
  t.header = o.header;
  t.child_frame_id = base_frame;
  t.transform.translation.x = p.pos.x();
  t.transform.translation.y = p.pos.y();
  t.transform.rotation.z = o.pose.pose.orientation.z;
  t.transform.rotation.w = o.pose.pose.orientation.w;
}

}  // namespace g1_leg_odom
