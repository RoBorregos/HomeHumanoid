// Track B bridge: drives unitree_rl_lab's g1_ctrl from Nav2 in MuJoCo.
//
// g1_ctrl reads its velocity command from the wireless_remote bytes embedded in
// rt/lowstate (see docs/plan.md, "M1 findings"), so this node relays every
// rt/lowstate sample with those bytes overridden. Sim only: on the real robot
// use g1::LocoClient::SetVelocity instead of faking the remote.
//
// Axis mapping comes from unitree_rl_lab's velocity_commands observation:
//   vx = ly, vy = -lx, wz = -rx, each clamped to the policy's trained range.
// The values are m/s and rad/s directly, there is no joystick scale.
//
// The policy does not track those values linearly: it has a dead zone (0.2 m/s
// forward does nothing) and tracks well below the command (0.3 rad/s yields
// 0.05 rad/s). Nav2 expects to get what it asks for, so each axis goes through a
// table measured in MuJoCo, from desired velocity to the command that yields it.
//
// Safety: cmd_vel that stopped arriving, or whose stamp is older than
// cmd_vel_timeout, zeroes the command here. The stamp check matters because
// collision_monitor was seen republishing a stale command forever. If this
// process dies, g1_ctrl falls back to the simulator's own samples, whose remote
// bytes are all zero.
//
// Odometry: unitree_mujoco publishes the IMU site's world position and velocity
// on rt/sportmodestate using the go2 message type, even for the G1. Yaw comes
// from the IMU in rt/lowstate. That is ground truth the real robot doesn't have.
// odom_source selects what drives /odom and odom -> pelvis:
//   leg (default): g1_leg_odom's estimator, from joint angles and the IMU as on
//     the robot, fed from this node's own rt/lowstate subscription. Ground truth
//     goes to /odom_ground_truth, without TF, to measure the estimator's drift.
//   ground_truth: MuJoCo's pose on /odom, the original Track B setup.
// The estimator runs here rather than as the standalone g1_leg_odom node because
// on loopback without multicast a second DDS participant can fail to get one of
// the few auto participant indices that every DDS process on domain 0 shares.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "g1_leg_odom/leg_odometry.hpp"

#include "unitree/idl/go2/SportModeState_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

using namespace std::chrono_literals;
using LowState = unitree_hg::msg::dds_::LowState_;
using SportState = unitree_go::msg::dds_::SportModeState_;

namespace {
// wireless_remote layout, from unitree_sdk2's unitree_joystick.hpp.
constexpr size_t kOffsetKeys = 2, kOffsetLx = 4, kOffsetRx = 8, kOffsetLy = 20;
constexpr uint16_t kR1 = 1 << 0, kL2 = 1 << 5, kX = 1 << 10, kUp = 1 << 12;
// Byte 0 is an unused header. Marks samples this node relayed: the relay also
// receives its own writes on rt/lowstate, and would otherwise republish forever.
constexpr uint8_t kRelayMark = 0xA5;

double clampf(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }

// Piecewise-linear lookup on |v|, keeping the sign. Clamps past the table's end.
double compensate(double v, const std::vector<double> & want, const std::vector<double> & cmd) {
  const double a = std::fabs(v);
  double out = cmd.back();
  for (size_t i = 1; i < want.size(); ++i) {
    if (a <= want[i]) {
      out = cmd[i - 1] + (a - want[i - 1]) / (want[i] - want[i - 1]) * (cmd[i] - cmd[i - 1]);
      break;
    }
  }
  return std::copysign(out, v);
}
}  // namespace

class G1RlBridge : public rclcpp::Node {
public:
  G1RlBridge() : Node("g1_rl_bridge") {
    cmd_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
    settle_s_ = declare_parameter("fixstand_settle_s", 15.0);
    vx_min_ = declare_parameter("vx_min", -0.5);
    vx_max_ = declare_parameter("vx_max", 1.0);
    vy_max_ = declare_parameter("vy_max", 0.5);
    wz_max_ = declare_parameter("wz_max", 1.0);
    // Measured steady tracking; the 0.01 entry jumps straight to the dead zone's edge.
    vx_want_ = declare_parameter("vx_want", std::vector<double>{0.0, 0.01, 0.109, 0.161, 0.305, 0.397});
    vx_cmd_ = declare_parameter("vx_cmd", std::vector<double>{0.0, 0.2, 0.25, 0.3, 0.4, 0.5});
    vy_want_ = declare_parameter("vy_want", std::vector<double>{0.0, 0.01, 0.197, 0.345});
    vy_cmd_ = declare_parameter("vy_cmd", std::vector<double>{0.0, 0.2, 0.3, 0.5});
    wz_want_ = declare_parameter("wz_want", std::vector<double>{0.0, 0.01, 0.048, 0.156, 0.414});
    wz_cmd_ = declare_parameter("wz_cmd", std::vector<double>{0.0, 0.2, 0.3, 0.6, 1.0});
    spawn_x_ = declare_parameter("spawn_x", 0.0);
    spawn_y_ = declare_parameter("spawn_y", 0.0);
    spawn_yaw_ = declare_parameter("spawn_yaw", 0.0);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    const auto odom_source = declare_parameter("odom_source", std::string("leg"));
    if (odom_source != "leg" && odom_source != "ground_truth")
      throw std::invalid_argument("odom_source must be 'leg' or 'ground_truth', got " + odom_source);
    use_leg_ = odom_source == "leg";
    if (use_leg_) {
      g1_leg_odom::LegOdometry::Params lp;
      lp.initial_x = spawn_x_;
      lp.initial_y = spawn_y_;
      lp.initial_yaw = spawn_yaw_;
      lp.switch_margin = declare_parameter("switch_margin", lp.switch_margin);
      lp.use_load = declare_parameter("use_load", lp.use_load);
      lp.load_filter_tau = declare_parameter("load_filter_tau", lp.load_filter_tau);
      lp.load_hysteresis = declare_parameter("load_hysteresis", lp.load_hysteresis);
      lp.velocity_filter_tau = declare_parameter("velocity_filter_tau", lp.velocity_filter_tau);
      leg_odom_ = std::make_unique<g1_leg_odom::LegOdometry>(lp);
    }
    const auto iface = declare_parameter("dds_network_interface", std::string("lo"));
    const auto domain = declare_parameter("dds_domain_id", 0);

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        const double age = (now() - rclcpp::Time(m->header.stamp, get_clock()->get_clock_type())).seconds();
        if (age > cmd_timeout_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping stale cmd_vel, %.2fs old", age);
          return;
        }
        std::lock_guard<std::mutex> l(mutex_);
        vx_ = m->twist.linear.x;
        vy_ = m->twist.linear.y;
        wz_ = m->twist.angular.z;
        last_cmd_ = std::chrono::steady_clock::now();
      });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(use_leg_ ? "/odom_ground_truth" : "/odom", 10);
    if (use_leg_) {
      leg_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      // The estimate drifts while the elastic band holds the robot up, so
      // release_band.py calls this once the feet carry the weight.
      reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_odom", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                               std_srvs::srv::Trigger::Response::SharedPtr res) {
          std::lock_guard<std::mutex> l(mutex_);
          leg_odom_->reset();
          res->success = true;
          res->message = "leg odometry reset to the spawn pose";
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        });
    }
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_timer_ = create_wall_timer(20ms, [this] { publish_odom(); });

    unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
    pub_ = std::make_shared<unitree::robot::ChannelPublisher<LowState>>("rt/lowstate");
    pub_->InitChannel();
    sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    sub_->InitChannel([this](const void * m) { on_lowstate(*static_cast<const LowState *>(m)); }, 1);
    sport_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<SportState>>("rt/sportmodestate");
    sport_sub_->InitChannel([this](const void * m) { on_sport(*static_cast<const SportState *>(m)); }, 1);

    startup_ = std::thread([this] { run_startup(); });
  }

  ~G1RlBridge() override {
    if (startup_.joinable()) startup_.join();
  }

private:
  void on_lowstate(const LowState & in) {
    if (in.wireless_remote()[0] == kRelayMark) return;  // Our own write.
    seen_lowstate_ = true;
    LowState out = in;
    const auto & rpy = in.imu_state().rpy();
    double vx = 0, vy = 0, wz = 0;
    {
      std::lock_guard<std::mutex> l(mutex_);
      yaw_ = rpy[2];
      wz_meas_ = in.imu_state().gyroscope()[2];
      have_yaw_ = true;
      const std::chrono::duration<double> age = std::chrono::steady_clock::now() - last_cmd_;
      if (ready_ && age.count() < cmd_timeout_) {
        vx = clampf(compensate(vx_, vx_want_, vx_cmd_), vx_min_, vx_max_);
        vy = clampf(compensate(vy_, vy_want_, vy_cmd_), -vy_max_, vy_max_);
        wz = clampf(compensate(wz_, wz_want_, wz_cmd_), -wz_max_, wz_max_);
      }
    }
    const uint16_t keys = keys_.load();
    const float lx = static_cast<float>(-vy), ly = static_cast<float>(vx), rx = static_cast<float>(-wz);
    auto remote = out.wireless_remote();
    remote.fill(0);
    remote[0] = kRelayMark;
    std::memcpy(remote.data() + kOffsetKeys, &keys, sizeof(keys));
    std::memcpy(remote.data() + kOffsetLx, &lx, sizeof(lx));
    std::memcpy(remote.data() + kOffsetRx, &rx, sizeof(rx));
    std::memcpy(remote.data() + kOffsetLy, &ly, sizeof(ly));
    out.wireless_remote(remote);
    pub_->Write(out);
    // After the relay, not before: g1_ctrl takes the newest sample on the topic,
    // and until this copy lands that is MuJoCo's own, with no joystick. Running
    // the estimator first widened that gap enough that the FSM missed the
    // button presses and the policy read zero velocity most of the time.
    if (leg_odom_) {
      std::lock_guard<std::mutex> l(mutex_);
      if (leg_odom_->update(in, std::chrono::steady_clock::now())) leg_fresh_ = true;
    }
  }

  void on_sport(const SportState & s) {
    std::lock_guard<std::mutex> l(mutex_);
    pos_ = {s.position()[0], s.position()[1]};
    vel_ = {s.velocity()[0], s.velocity()[1]};
    z_ = s.position()[2];
    have_pos_ = true;
  }

  void publish_odom() {
    if (use_leg_) publish_leg_odom();
    std::array<float, 2> p, v;
    double yaw, z, wz;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (!have_pos_ || !have_yaw_) return;
      p = pos_;
      v = vel_;
      yaw = yaw_;
      z = z_;
      wz = wz_meas_;
    }
    if (!origin_set_) {
      origin_ = p;
      yaw0_ = yaw;
      origin_set_ = true;
    }
    // Express motion since start in the odom frame, seeded at the spawn pose.
    const double dyaw = spawn_yaw_ - yaw0_;
    const double dx = p[0] - origin_[0], dy = p[1] - origin_[1];
    const double x = spawn_x_ + std::cos(dyaw) * dx - std::sin(dyaw) * dy;
    const double y = spawn_y_ + std::sin(dyaw) * dx + std::cos(dyaw) * dy;
    const double th = yaw + dyaw;
    const auto now = get_clock()->now();

    nav_msgs::msg::Odometry o;
    o.header.stamp = now;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = x;
    o.pose.pose.position.y = y;
    o.pose.pose.position.z = z;  // Height of the IMU, shows a fall. Not used in the TF.
    o.pose.pose.orientation.z = std::sin(th / 2);
    o.pose.pose.orientation.w = std::cos(th / 2);
    // World velocity to body frame.
    o.twist.twist.linear.x = std::cos(yaw) * v[0] + std::sin(yaw) * v[1];
    o.twist.twist.linear.y = -std::sin(yaw) * v[0] + std::cos(yaw) * v[1];
    o.twist.twist.angular.z = wz;
    odom_pub_->publish(o);
    if (use_leg_) return;  // Ground truth only for comparison: the TF is the estimate's.

    geometry_msgs::msg::TransformStamped t;
    t.header = o.header;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.z = o.pose.pose.orientation.z;
    t.transform.rotation.w = o.pose.pose.orientation.w;
    tf_->sendTransform(t);
  }

  void publish_leg_odom() {
    g1_leg_odom::Pose p;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (!leg_odom_->initialized() || !leg_fresh_) return;
      leg_fresh_ = false;
      p = leg_odom_->pose();
    }
    nav_msgs::msg::Odometry o;
    geometry_msgs::msg::TransformStamped t;
    g1_leg_odom::to_msgs(p, get_clock()->now(), odom_frame_, base_frame_, o, t);
    leg_pub_->publish(o);
    tf_->sendTransform(t);
  }

  // Passive -> FixStand -> Velocity, the only way into the policy: there is no
  // entry point that bypasses this FSM. The buttons are edge-triggered.
  void run_startup() {
    while (rclcpp::ok() && !seen_lowstate_) std::this_thread::sleep_for(100ms);
    const auto hold = [this](uint16_t k, std::chrono::milliseconds d) {
      keys_ = k;
      std::this_thread::sleep_for(d);
    };
    hold(0, 500ms);
    hold(kL2, 500ms);
    hold(kL2 | kUp, 300ms);
    RCLCPP_INFO(get_logger(), "FixStand requested, settling %.0fs", settle_s_);
    hold(0, std::chrono::milliseconds(static_cast<int>(settle_s_ * 1000)));
    hold(kR1, 500ms);
    hold(kR1 | kX, 300ms);
    keys_ = 0;
    {
      std::lock_guard<std::mutex> l(mutex_);
      ready_ = true;
    }
    RCLCPP_INFO(get_logger(), "Velocity requested, accepting cmd_vel");
  }

  double cmd_timeout_, settle_s_, vx_min_, vx_max_, vy_max_, wz_max_, spawn_x_, spawn_y_, spawn_yaw_;
  std::vector<double> vx_want_, vx_cmd_, vy_want_, vy_cmd_, wz_want_, wz_cmd_;
  std::string odom_frame_, base_frame_;
  bool use_leg_ = false, leg_fresh_ = false;
  std::unique_ptr<g1_leg_odom::LegOdometry> leg_odom_;

  std::mutex mutex_;
  double vx_ = 0, vy_ = 0, wz_ = 0, yaw_ = 0, yaw0_ = 0, wz_meas_ = 0;
  std::array<float, 2> pos_{}, vel_{}, origin_{};
  float z_ = 0;
  bool have_pos_ = false, have_yaw_ = false, origin_set_ = false, ready_ = false;
  std::chrono::steady_clock::time_point last_cmd_{};
  std::atomic<uint16_t> keys_{0};
  std::atomic<bool> seen_lowstate_{false};
  std::thread startup_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, leg_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  unitree::robot::ChannelPublisherPtr<LowState> pub_;
  unitree::robot::ChannelSubscriberPtr<LowState> sub_;
  unitree::robot::ChannelSubscriberPtr<SportState> sport_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1RlBridge>());
  rclcpp::shutdown();
  return 0;
}
