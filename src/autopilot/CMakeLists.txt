cmake_minimum_required(VERSION 3.8)
project(autopilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(autopilot src/autopilot_node.cpp)
ament_target_dependencies(autopilot rclcpp rcl_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs)

install(TARGETS
	autopilot
	DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})

ament_package()
