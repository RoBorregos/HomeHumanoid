cmake_minimum_required(VERSION 3.8)
project(g1_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/nav_central.py
  scripts/semantic_nav_node.py
  scripts/person_goal_smoother.py
  scripts/table_docker.py
  scripts/nav_ui.py
  scripts/map_area_tagger.py
  scripts/nav_client_check.py
  scripts/run_nav.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config bt maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_utils test/test_launch_utils.py)
endif()

ament_package()
