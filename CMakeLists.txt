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
  "msg/DirectionalDistance.msg"
  DEPENDENCIES  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Include Cpp "include" directory
include_directories(include/turtle_controller)
set(HEADER_FILES 
  include/turtle_controller/controller_server.hpp
  include/turtle_controller/keyboard_parser.hpp
)

# Cria uma biblioteca a partir da implementação
add_library(controller_server_lib src/controller_server.cpp ${HEADER_FILES})
ament_target_dependencies(controller_server_lib PUBLIC rclcpp std_msgs keyboard_msgs geometry_msgs nav_msgs turtlesim tf2 sensor_msgs)
target_link_libraries(controller_server_lib PUBLIC "${cpp_typesupport_target}")

add_library(keyboard_parser_lib src/keyboard_parser.cpp ${HEADER_FILES})
ament_target_dependencies(keyboard_parser_lib PUBLIC rclcpp std_msgs keyboard_msgs )
target_link_libraries(keyboard_parser_lib PUBLIC "${cpp_typesupport_target}")

# Cria o executável
add_executable(controller_server src/server_node.cpp)
ament_target_dependencies(controller_server PUBLIC rclcpp std_msgs keyboard_msgs geometry_msgs nav_msgs turtlesim tf2 sensor_msgs)
target_link_libraries(controller_server PUBLIC controller_server_lib)

add_executable(keyboard_parser src/keyboard_parser_node.cpp)
ament_target_dependencies(keyboard_parser PUBLIC rclcpp std_msgs keyboard_msgs)
target_link_libraries(keyboard_parser PUBLIC keyboard_parser_lib)

# Instala os diretórios de cabeçalhos
install(
  DIRECTORY include/turtle_controller
  DESTINATION include
)

# Instala o executável
install(TARGETS
  controller_server
  keyboard_parser
  DESTINATION lib/${PROJECT_NAME}
)

# Instala a biblioteca
install(
  TARGETS controller_server_lib
  keyboard_parser_lib
  EXPORT export_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# ament_export_targets(controller_server_lib HAS_LIBRARY_TARGET)
# ament_export_targets(keyboard_parser_lib HAS_LIBRARY_TARGET)
ament_export_targets(export_targets HAS_LIBRARY_TARGET)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/controller_base.py
  scripts/controller_scape.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/main.cpp
      test/test_controller.cpp
      )
  # Add gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test controller_server_lib)

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
