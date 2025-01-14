cmake_minimum_required(VERSION 3.8)
project(vehicle_following_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -w)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(pid_package REQUIRED)
find_package(psdk_interfaces REQUIRED)
find_package(dji_mission_interfaces REQUIRED)

set(ROS_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  geodesy
)

include_directories(
  include
  ${pid_package_INCLUDE_DIRS}
)

add_executable(vehicle_following src/gps_to_ned.cpp src/vehicle_following.cpp)
ament_target_dependencies(vehicle_following 
  ${ROS_DEPENDENCIES}
  pid_package
  psdk_interfaces
  dji_mission_interfaces
)

install(TARGETS vehicle_following
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
