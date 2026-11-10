// Standalone leg-inertial odometry node: rt/lowstate -> /odom + odom -> pelvis.
// The estimator and its limits are described in leg_odometry.hpp.
//
// For the real robot, where nothing else relays rt/lowstate. In the MuJoCo sim,
// g1_rl_bridge runs the same estimator itself (odom_source:=leg), since a second
// DDS participant on loopback can fail to get a participant index.
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "g1_leg_odom/leg_odometry.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

using LowState = unitree_hg::msg::dds_::LowState_;

namespace {
// g1_rl_bridge marks its relayed copies in wireless_remote[0]. They carry the
// same joint and IMU data, so skip them rather than process each sample twice.
constexpr uint8_t kRelayMark = 0xA5;
}  // namespace

class G1LegOdomNode : public rclcpp::Node {
public:
  G1LegOdomNode() : Node("g1_leg_odom"), odom_(params()) {
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    publish_tf_ = declare_parameter("publish_tf", true);
    const double rate = declare_parameter("publish_rate", 50.0);
    const auto iface = declare_parameter("dds_network_interface", std::string("lo"));
    const auto domain = declare_parameter("dds_domain_id", 0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_odom", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                             std_srvs::srv::Trigger::Response::SharedPtr res) {
        std::lock_guard<std::mutex> l(mutex_);
        odom_.reset();
        res->success = true;
        res->message = "leg odometry reset to the initial pose";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { publish(); });

    unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
    sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    sub_->InitChannel([this](const void * m) { on_lowstate(*static_cast<const LowState *>(m)); }, 1);
    RCLCPP_INFO(get_logger(), "Waiting for rt/lowstate on %s, domain %ld", iface.c_str(), domain);
  }

private:
  g1_leg_odom::LegOdometry::Params params() {
    g1_leg_odom::LegOdometry::Params p;
    p.initial_x = declare_parameter("initial_x", 0.0);
    p.initial_y = declare_parameter("initial_y", 0.0);
    p.initial_yaw = declare_parameter("initial_yaw", 0.0);
    p.switch_margin = declare_parameter("switch_margin", p.switch_margin);
    p.velocity_filter_tau = declare_parameter("velocity_filter_tau", p.velocity_filter_tau);
    return p;
  }

  void on_lowstate(const LowState & s) {
    if (s.wireless_remote()[0] == kRelayMark) return;
    std::lock_guard<std::mutex> l(mutex_);
    if (odom_.update(s, std::chrono::steady_clock::now())) fresh_ = true;
  }

  void publish() {
    g1_leg_odom::Pose p;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (!odom_.initialized()) return;
      if (!fresh_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No new rt/lowstate, not publishing /odom");
        return;
      }
      fresh_ = false;
      p = odom_.pose();
    }
    nav_msgs::msg::Odometry o;
    geometry_msgs::msg::TransformStamped t;
    g1_leg_odom::to_msgs(p, get_clock()->now(), odom_frame_, base_frame_, o, t);
    odom_pub_->publish(o);
    if (publish_tf_) tf_->sendTransform(t);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "pose (%.3f, %.3f), %ld pivot switches",
                         p.pos.x(), p.pos.y(), p.pivot_switches);
  }

  std::string odom_frame_, base_frame_;
  bool publish_tf_;
  std::mutex mutex_;
  g1_leg_odom::LegOdometry odom_;
  bool fresh_ = false;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  unitree::robot::ChannelSubscriberPtr<LowState> sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1LegOdomNode>());
  rclcpp::shutdown();
  return 0;
}
