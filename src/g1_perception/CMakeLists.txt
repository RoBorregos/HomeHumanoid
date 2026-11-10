cmake_minimum_required(VERSION 3.8)
project(g1_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Only on the robot with FAST-LIO: livox_ros_driver2 publishes CustomMsg (with
# per-point time, which FAST-LIO needs), and this converts it to PointCloud2 for
# the crop / scan projection. Built only when third_party/lio_ws is sourced.
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
  add_executable(livox_custom_to_pc2 src/livox_custom_to_pc2.cpp)
  target_compile_features(livox_custom_to_pc2 PRIVATE cxx_std_17)
  target_link_libraries(livox_custom_to_pc2 rclcpp::rclcpp ${sensor_msgs_TARGETS})
  target_include_directories(livox_custom_to_pc2 PRIVATE ${livox_ros_driver2_INCLUDE_DIRS})
  ament_target_dependencies(livox_custom_to_pc2 livox_ros_driver2)
  install(TARGETS livox_custom_to_pc2 DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "livox_ros_driver2 not found: skipping livox_custom_to_pc2 "
                 "(run third_party/fetch_lio.sh and source lio_ws for the robot)")
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lio_odom_adapter.py DESTINATION lib/${PROJECT_NAME})

ament_package()
