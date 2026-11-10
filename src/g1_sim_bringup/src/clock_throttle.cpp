// /clock_gz (every Gazebo step, ~1 kHz) -> /clock at most `rate` Hz.
//
// Every node with use_sim_time subscribes to /clock. At Gazebo's step rate the
// Python ones (nav_central, semantic_nav, person_goal_smoother, the bridges,
// even initial_pose) spent 40-90% of a core each just deserializing it on the
// Orin (2026-11-10), which also delayed slam_toolbox's map -> odom. 100 Hz is
// plenty for timers, TF lookups and Nav2's 20 Hz loops.
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ClockThrottle : public rclcpp::Node {
public:
  ClockThrottle() : Node("clock_throttle") {
    const double rate = declare_parameter("rate", 100.0);
    period_ns_ = static_cast<int64_t>(1e9 / rate);
    pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock_gz", rclcpp::ClockQoS(), [this](rosgraph_msgs::msg::Clock::ConstSharedPtr m) {
        const int64_t t = rclcpp::Time(m->clock).nanoseconds();
        if (t < last_ns_ || t - last_ns_ >= period_ns_) {  // t < last: sim reset
          last_ns_ = t;
          pub_->publish(*m);
        }
      });
  }

private:
  int64_t period_ns_ = 10'000'000;
  int64_t last_ns_ = 0;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockThrottle>());
  rclcpp::shutdown();
  return 0;
}
