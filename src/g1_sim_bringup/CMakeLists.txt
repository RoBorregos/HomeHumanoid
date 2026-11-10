cmake_minimum_required(VERSION 3.8)
project(g1_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(clock_throttle src/clock_throttle.cpp)
target_compile_features(clock_throttle PRIVATE cxx_std_17)
target_link_libraries(clock_throttle rclcpp::rclcpp ${rosgraph_msgs_TARGETS})
install(TARGETS clock_throttle DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_mujoco.sh
  scripts/release_band.py
  scripts/odom_error.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
