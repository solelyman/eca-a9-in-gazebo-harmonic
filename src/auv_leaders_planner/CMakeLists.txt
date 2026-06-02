cmake_minimum_required(VERSION 3.8)
project(auv_leaders_planner)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


set(msg_files
  "msg/AuxVar.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)


add_executable(leader_planner_node src/leaders_planner.cpp)


target_include_directories(leader_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(leader_planner_node PUBLIC c_std_99 cxx_std_17)


ament_target_dependencies(leader_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(leader_planner_node
  "${cpp_typesupport_target}"
  ${EIGEN3_LIBRARIES}
)


install(TARGETS leader_planner_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})


install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS 
  scripts/current_predictor.py
  scripts/record_metrics.py
  scripts/record_dim2_metrics.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
