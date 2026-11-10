cmake_minimum_required(VERSION 3.8)
project(g1_leg_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
# Installed separately at the default prefix, see third_party/README.md.
find_package(unitree_sdk2 REQUIRED)

add_executable(g1_leg_odom src/g1_leg_odom.cpp)
target_compile_features(g1_leg_odom PUBLIC cxx_std_17)
target_include_directories(g1_leg_odom PRIVATE include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(g1_leg_odom unitree_sdk2)
ament_target_dependencies(g1_leg_odom rclcpp geometry_msgs nav_msgs tf2_ros std_srvs)

install(TARGETS g1_leg_odom DESTINATION lib/${PROJECT_NAME})
# Header-only estimator, also built into g1_rl_bridge.
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp geometry_msgs nav_msgs)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
