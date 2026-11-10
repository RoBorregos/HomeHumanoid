// livox_ros_driver2 CustomMsg -> sensor_msgs/PointCloud2 (x, y, z, intensity, time).
//
// On the robot the driver publishes CustomMsg (xfer_format 1) because FAST-LIO's
// Livox handler needs the per-point offset_time to deskew. Everything else
// (crop, scan projection, voxel layer) takes PointCloud2, so this republishes
// the same scan. frame_id is overridden to the URDF frame (mid360_link) since
// the driver stamps its own "livox_frame".
#include <cstring>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class LivoxCustomToPc2 : public rclcpp::Node {
public:
  LivoxCustomToPc2() : Node("livox_custom_to_pc2") {
    frame_id_ = declare_parameter("frame_id", std::string("mid360_link"));
    const auto in = declare_parameter("input_topic", std::string("/livox/lidar"));
    const auto out = declare_parameter("output_topic", std::string("/livox/points"));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::SensorDataQoS());
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      in, rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr m) { convert(*m); });
  }

private:
  void convert(const livox_ros_driver2::msg::CustomMsg & m) {
    sensor_msgs::msg::PointCloud2 pc;
    pc.header.stamp = m.header.stamp;
    pc.header.frame_id = frame_id_;
    pc.height = 1;
    pc.width = static_cast<uint32_t>(m.points.size());
    const char * names[] = {"x", "y", "z", "intensity", "time"};
    for (uint32_t i = 0; i < 5; ++i) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      pc.fields.push_back(f);
    }
    pc.point_step = 20;
    pc.row_step = pc.point_step * pc.width;
    pc.is_bigendian = false;
    pc.is_dense = true;
    pc.data.resize(pc.row_step);
    uint8_t * d = pc.data.data();
    for (const auto & p : m.points) {
      const float v[5] = {p.x, p.y, p.z, static_cast<float>(p.reflectivity),
                          static_cast<float>(p.offset_time * 1e-9)};
      std::memcpy(d, v, sizeof(v));
      d += pc.point_step;
    }
    pub_->publish(std::move(pc));
  }

  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomToPc2>());
  rclcpp::shutdown();
  return 0;
}
