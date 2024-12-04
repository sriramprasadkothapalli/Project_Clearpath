# Locate necessary packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add test_level1 executable
add_executable(
  cpp_test
  test_level1.cpp)

# Include directories needed for test_level1
target_include_directories(
  cpp_test PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS}
                  ${rclcpp_INCLUDE_DIRS})

# Link dependencies for test_level1
ament_target_dependencies(
  cpp_test
  tf2
  tf2_geometry_msgs
  nav_msgs
  OpenCV
  rclcpp
  std_msgs
  gazebo_msgs)

# Link GTest explicitly
target_link_libraries(cpp_test gtest gtest_main myDebris)

# Enable code coverage flags for cpp_test
target_compile_options(cpp_test PUBLIC --coverage)
target_link_options(cpp_test PUBLIC --coverage)

# Discover tests for test_level1
gtest_discover_tests(cpp_test)