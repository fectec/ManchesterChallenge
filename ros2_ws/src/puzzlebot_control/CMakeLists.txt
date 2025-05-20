cmake_minimum_required(VERSION 3.8)
project(puzzlebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy                REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(nav_msgs             REQUIRED)
find_package(tf2_ros              REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simulation_velocity_controller.py
  ${PROJECT_NAME}/real_velocity_controller.py
  ${PROJECT_NAME}/open_loop_path_generator.py
  ${PROJECT_NAME}/open_loop_point_controller.py
  ${PROJECT_NAME}/odometry_localization.py
  ${PROJECT_NAME}/pid_path_generator.py
  ${PROJECT_NAME}/pid_point_controller.py
  ${PROJECT_NAME}/line_follow_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclpy
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

ament_package()