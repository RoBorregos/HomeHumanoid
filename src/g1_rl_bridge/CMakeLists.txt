cmake_minimum_required(VERSION 3.8)
project(g1_rl_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# Installed separately at the default prefix, see third_party/README.md.
find_package(unitree_sdk2 REQUIRED)

add_executable(g1_rl_bridge src/g1_rl_bridge.cpp)
target_compile_features(g1_rl_bridge PUBLIC cxx_std_17)
target_link_libraries(g1_rl_bridge unitree_sdk2)
ament_target_dependencies(g1_rl_bridge rclcpp geometry_msgs nav_msgs tf2_ros)

install(TARGETS g1_rl_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()
