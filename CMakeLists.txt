cmake_minimum_required(VERSION 3.8)
project(turtle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(keyboard_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/RobotCmd.msg"
  DEPENDENCIES  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Include Cpp "include" directory
include_directories(include)

add_executable(controller_server src/controller_server.cpp)
ament_target_dependencies(controller_server rclcpp std_msgs keyboard_msgs geometry_msgs nav_msgs turtlesim tf2 sensor_msgs)
target_link_libraries(controller_server "${cpp_typesupport_target}")

install(TARGETS
controller_server
  DESTINATION lib/${PROJECT_NAME}
 )

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/controller_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)


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
