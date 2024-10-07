cmake_minimum_required(VERSION 3.8)
project(choreonoid_ros2_sample_drone_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(choreonoid REQUIRED)

set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

add_subdirectory(src)

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

add_executable(choreonoid_joy_node src/choreonoid_joy_node.cpp)
set_target_properties(choreonoid_joy_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(choreonoid_joy_node Choreonoid::CnoidUtil)
ament_target_dependencies(choreonoid_joy_node rclcpp sensor_msgs choreonoid)

install(TARGETS
  choreonoid_joy_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

install(DIRECTORY project launch model DESTINATION share/${PROJECT_NAME})