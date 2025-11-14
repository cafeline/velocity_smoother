cmake_minimum_required(VERSION 3.8)
project(velocity_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(velocity_smoother_core
  src/velocity_smoother_node.cpp)
target_include_directories(velocity_smoother_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(velocity_smoother_core
  rclcpp
  geometry_msgs
  visualization_msgs)
target_compile_features(velocity_smoother_core PUBLIC c_std_99 cxx_std_17)

add_executable(velocity_smoother src/velocity_smoother_main.cpp)
target_link_libraries(velocity_smoother PRIVATE velocity_smoother_core)

install(TARGETS velocity_smoother velocity_smoother_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_command_filter test/test_command_filter.cpp)
  target_link_libraries(test_command_filter
    velocity_smoother_core)
  ament_target_dependencies(test_command_filter
    geometry_msgs)

  ament_add_gtest(test_velocity_smoother_node test/test_velocity_smoother_node.cpp)
  target_link_libraries(test_velocity_smoother_node
    velocity_smoother_core)
  ament_target_dependencies(test_velocity_smoother_node
    rclcpp
    geometry_msgs
    visualization_msgs)
endif()

ament_package()
