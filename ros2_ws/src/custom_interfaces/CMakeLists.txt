cmake_minimum_required(VERSION 3.8)
project(custom_interfaces VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake               REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime    REQUIRED)
find_package(std_msgs                  REQUIRED)
find_package(geometry_msgs             REQUIRED)
find_package(builtin_interfaces        REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetProcessBool.srv"
  "srv/NextPIDWaypoint.srv"
  "msg/OpenLoopPose.msg"
  "msg/PIDGoalPose.msg"
  "msg/ColorBlobDetection.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()