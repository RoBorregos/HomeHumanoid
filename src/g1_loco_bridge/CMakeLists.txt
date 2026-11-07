cmake_minimum_required(VERSION 3.8)
project(g1_loco_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(g1_leg_odom REQUIRED)
find_package(Eigen3 REQUIRED)
# Installed separately at the default prefix, see third_party/README.md.
find_package(unitree_sdk2 REQUIRED)

add_executable(g1_loco_bridge src/g1_loco_bridge.cpp)
target_compile_features(g1_loco_bridge PUBLIC cxx_std_17)
target_include_directories(g1_loco_bridge PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(g1_loco_bridge unitree_sdk2)
ament_target_dependencies(g1_loco_bridge rclcpp geometry_msgs nav_msgs tf2_ros std_srvs g1_leg_odom)

install(TARGETS g1_loco_bridge DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
