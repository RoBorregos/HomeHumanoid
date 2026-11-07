// Real-robot bridge: Nav2 cmd_vel -> g1::LocoClient::SetVelocity, plus leg-inertial
// odometry from rt/lowstate. One process, one DDS participant.
//
// UNTESTED ON HARDWARE. Written against unitree_sdk2's g1_loco_client.hpp and
// its g1_loco_client_example; validate with the robot on its gantry first, with
// the wireless remote's emergency stop in hand.
//
// Why not the sim's joystick relay (g1_rl_bridge): on the robot, Unitree's own
// locomotion controller exposes a velocity RPC, and SetVelocity's duration
// argument is a built-in dead-man: if this process stops sending, the robot
// stops after `command_duration` seconds on its own.
//
// What this node does NOT do: change the robot's FSM. The operator brings the
// robot to its walking mode with the remote (or Unitree's app); this node only
// sends velocities, and logs the FSM id it sees at startup. Set auto_start to
// call LocoClient::Start() instead, only once that has been checked by hand.
//
// Safety:
//   - cmd_vel older than cmd_vel_timeout (by stamp or by arrival) sends zero.
//   - Each axis is clamped to max_vx / max_vy / max_wz, conservative defaults.
//   - StopMove() on shutdown.
//   - The g1_rl_bridge compensation tables are NOT used: they were measured for
//     the sim policy only (docs/plan.md, M2b) and must be re-measured here.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "g1_leg_odom/leg_odometry.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/g1/loco/g1_loco_client.hpp"

using namespace std::chrono_literals;
using LowState = unitree_hg::msg::dds_::LowState_;

class G1LocoBridge : public rclcpp::Node {
public:
  G1LocoBridge() : Node("g1_loco_bridge") {
    cmd_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
    max_vx_ = declare_parameter("max_vx", 0.3);
    max_vy_ = declare_parameter("max_vy", 0.2);
    max_wz_ = declare_parameter("max_wz", 0.3);
    const double rate = declare_parameter("command_rate", 10.0);
    // Must outlast a couple of command periods, and is the stop latency if this dies.
    duration_ = declare_parameter("command_duration", 0.3);
    const bool auto_start = declare_parameter("auto_start", false);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("pelvis"));
    const auto iface = declare_parameter("dds_network_interface", std::string("eth0"));
    const auto domain = declare_parameter("dds_domain_id", 0);
    g1_leg_odom::LegOdometry::Params lp;
    lp.initial_x = declare_parameter("initial_x", 0.0);
    lp.initial_y = declare_parameter("initial_y", 0.0);
    lp.initial_yaw = declare_parameter("initial_yaw", 0.0);
    lp.switch_margin = declare_parameter("switch_margin", lp.switch_margin);
    lp.velocity_filter_tau = declare_parameter("velocity_filter_tau", lp.velocity_filter_tau);
    odom_ = std::make_unique<g1_leg_odom::LegOdometry>(lp);

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        const double age = (now() - rclcpp::Time(m->header.stamp, get_clock()->get_clock_type())).seconds();
        if (age > cmd_timeout_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping stale cmd_vel, %.2fs old", age);
          return;
        }
        std::lock_guard<std::mutex> l(cmd_mutex_);
        vx_ = std::clamp(m->twist.linear.x, -max_vx_, max_vx_);
        vy_ = std::clamp(m->twist.linear.y, -max_vy_, max_vy_);
        wz_ = std::clamp(m->twist.angular.z, -max_wz_, max_wz_);
        last_cmd_ = std::chrono::steady_clock::now();
      });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_timer_ = create_wall_timer(20ms, [this] { publish_odom(); });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_odom", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                             std_srvs::srv::Trigger::Response::SharedPtr res) {
        std::lock_guard<std::mutex> l(odom_mutex_);
        odom_->reset();
        res->success = true;
        res->message = "leg odometry reset to the initial pose";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
    sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    sub_->InitChannel([this](const void * m) {
      std::lock_guard<std::mutex> l(odom_mutex_);
      if (odom_->update(*static_cast<const LowState *>(m), std::chrono::steady_clock::now())) odom_fresh_ = true;
    }, 1);

    // Constructed after ChannelFactory::Init, as in unitree_sdk2's g1_loco_client_example.
    loco_ = std::make_unique<unitree::robot::g1::LocoClient>();
    loco_->Init();
    loco_->SetTimeout(1.0f);
    int fsm_id = -1;
    const int32_t rc = loco_->GetFsmId(fsm_id);
    RCLCPP_INFO(get_logger(), "LocoClient on %s: GetFsmId rc=%d fsm_id=%d", iface.c_str(), rc, fsm_id);
    if (auto_start) {
      RCLCPP_WARN(get_logger(), "auto_start: calling LocoClient::Start()");
      loco_->Start();
    }

    // RPCs block for up to the client timeout, so they run off the executor.
    period_ = std::chrono::duration<double>(1.0 / rate);
    sender_ = std::thread([this] { send_loop(); });
  }

  ~G1LocoBridge() override {
    running_ = false;
    if (sender_.joinable()) sender_.join();
    if (loco_) loco_->StopMove();
  }

private:
  void send_loop() {
    bool was_moving = false;
    while (running_ && rclcpp::ok()) {
      float vx = 0, vy = 0, wz = 0;
      {
        std::lock_guard<std::mutex> l(cmd_mutex_);
        const std::chrono::duration<double> age = std::chrono::steady_clock::now() - last_cmd_;
        if (age.count() < cmd_timeout_) {
          vx = vx_;
          vy = vy_;
          wz = wz_;
        }
      }
      const bool moving = vx != 0 || vy != 0 || wz != 0;
      // Zeros are sent once on the transition, not forever, so the remote can
      // still drive the robot while Nav2 is idle.
      if (moving || was_moving) {
        const int32_t rc = loco_->SetVelocity(vx, vy, wz, static_cast<float>(duration_));
        if (rc != 0) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "SetVelocity rc=%d", rc);
      }
      was_moving = moving;
      std::this_thread::sleep_for(period_);
    }
  }

  void publish_odom() {
    g1_leg_odom::Pose p;
    {
      std::lock_guard<std::mutex> l(odom_mutex_);
      if (!odom_->initialized() || !odom_fresh_) return;
      odom_fresh_ = false;
      p = odom_->pose();
    }
    nav_msgs::msg::Odometry o;
    geometry_msgs::msg::TransformStamped t;
    g1_leg_odom::to_msgs(p, get_clock()->now(), odom_frame_, base_frame_, o, t);
    odom_pub_->publish(o);
    tf_->sendTransform(t);
  }

  double cmd_timeout_, max_vx_, max_vy_, max_wz_, duration_;
  std::string odom_frame_, base_frame_;
  std::chrono::duration<double> period_{0.1};

  std::mutex cmd_mutex_;
  double vx_ = 0, vy_ = 0, wz_ = 0;
  std::chrono::steady_clock::time_point last_cmd_{};

  std::mutex odom_mutex_;
  std::unique_ptr<g1_leg_odom::LegOdometry> odom_;
  bool odom_fresh_ = false;

  std::atomic<bool> running_{true};
  std::thread sender_;
  std::unique_ptr<unitree::robot::g1::LocoClient> loco_;
  unitree::robot::ChannelSubscriberPtr<LowState> sub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1LocoBridge>());
  rclcpp::shutdown();
  return 0;
}
