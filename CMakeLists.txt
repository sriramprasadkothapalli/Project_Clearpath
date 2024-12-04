cmake_minimum_required(VERSION 3.8)
project(project_clearpath)

#
# GoogleTest Setup ref:
# https://google.github.io/googletest/quickstart-cmake.html
#

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Declare a dependency on GoogleTest which is downloaded from GitHub
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enables testing for this directory and below
enable_testing()
include(GoogleTest)

# Package dependencies
set(PACKAGE_DEPENDENCIES
    ament_cmake
    std_msgs
    sensor_msgs
    geometry_msgs
    gazebo_ros
    gazebo_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    image_transport
    cv_bridge
    rclcpp
    rclpy
    tf2
    OpenCV
    rosidl_default_generators
    rosbag2_cpp)

# Find dependencies
foreach(dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

# Doxygen Support: Check if Doxygen is installed ref:
# https://cmake.org/cmake/help/latest/module/FindDoxygen.html
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_CLASS_DIAGRAMS YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_CLASS_GRAPH YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_PACKAGE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
  set(DOXYGEN_TEMPLATE_RELATIONS YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
  set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
  set(DOXYGEN_DOT_TRANSPARENT YES)
else()
  message(
    FATAL_ERROR
      "Doxygen needs to be installed to generate the doxygen documentation")
endif()

#
# Create the compilation database for clangd and move it out of the build dir.
#
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink "${PROJECT_BINARY_DIR}/compile_commands.json"
          "${PROJECT_SOURCE_DIR}/compile_commands.json")

#
# Set up code coverage generation. Must compile with debug, e.g, cmake -S ./ -B
# build/ -D CMAKE_BUILD_TYPE=Debug
#
option(WANT_COVERAGE "This option enables code coverage" ON)
if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  set(WANT_COVERAGE OFF)
endif()

if(WANT_COVERAGE)
  message("Enabling coverage")

  # Add the directory containing CodeCoverage.cmake to the CMake module path
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")

  # Include the CodeCoverage module
  include(CodeCoverage)

  # This will add "--coverage" flags to the compiler
  append_coverage_compiler_flags()

  # Exclude build, docs, and unnecessary files from coverage analysis
  set(COVERAGE_EXCLUDES
    ${PROJECT_BINARY_DIR}         # Exclude build directories
    ${PROJECT_SOURCE_DIR}/docs    # Exclude docs directory
    "/usr/include/*"              # Exclude system headers
    "*gtest*"                     # Exclude GoogleTest code
  )

  # Include only .cpp and .hpp files from the 'debris' directory for coverage
  file(GLOB DEBRIS_SOURCES "${PROJECT_SOURCE_DIR}/libs/debris/debris_remover.cpp" "${PROJECT_SOURCE_DIR}/libs/debris/debris_remover.hpp"
                           "${PROJECT_SOURCE_DIR}/libs/debris/debris_detection.cpp" "${PROJECT_SOURCE_DIR}/libs/debris/debris_detection.hpp")

  # Unit test coverage setup
  setup_target_for_coverage_lcov(
    NAME test_coverage
    EXECUTABLE ctest || true
    EXCLUDE
    "app/main.cpp"
    "*gtest*"
    "/usr/include/*"
  )

  # Code coverage for app (shell-app)
  setup_target_for_coverage_gcovr_html(
    NAME app_coverage
    EXECUTABLE collector_node
    EXCLUDE
    "test/test_level1.cpp"
    "test/test_level2.cpp"
    INCLUDE ${DEBRIS_SOURCES}
  )
endif()

#
# c++ Boilerplate Modification Starts Here ref:
# https://iamsorush.com/posts/cpp-cmake-essential/ ref:
# https://cmake.org/cmake/help/latest/guide/tutorial/index.html
#
add_subdirectory(libs)
add_subdirectory(app)
add_subdirectory(test)

# Create a target to build documentation
doxygen_add_docs(docs # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/libs
  # Exclude docs or any directories/files that are not needed for coverage
  EXCLUDE ${PROJECT_SOURCE_DIR}/docs
)

# Install the necessary directories
install(DIRECTORY
  launch
  models
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# Sanity check: You can also do "cmake -S ./ -B build/ -LAH" to print all
# variables
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message(STATUS "WANT_COVERAGE    = ${WANT_COVERAGE}")
