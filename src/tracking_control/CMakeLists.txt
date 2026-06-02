cmake_minimum_required(VERSION 3.8)
project(tracking_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(dynamic_node_test src/dynamic_node_test.cpp)
ament_target_dependencies(dynamic_node_test rclcpp nav_msgs visualization_msgs tf2)
add_executable(sf_local_tracking src/sf_local_tracking.cpp)
ament_target_dependencies(sf_local_tracking rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

add_executable(thruster_allocator src/thruster_allocator.cpp)
ament_target_dependencies(thruster_allocator rclcpp geometry_msgs std_msgs)

add_executable(simulator src/simulator.cpp)
ament_target_dependencies(simulator rclcpp geometry_msgs nav_msgs tf2 tf2_ros)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS dynamic_node_test sf_local_tracking simulator thruster_allocator
      DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
      DESTINATION share/${PROJECT_NAME})



ament_package()
