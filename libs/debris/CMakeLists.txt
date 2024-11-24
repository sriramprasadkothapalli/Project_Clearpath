# Create a library called "myDebris"
add_library(myDebris
  # list of cpp source files:
  debris_detection.cpp
)

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(myDebris PUBLIC
  # list of directories:
  .
)

# Link dependencies to the library
ament_target_dependencies(myDebris
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  image_transport
  cv_bridge
  std_msgs
  OpenCV
  tf2_ros
  tf2
)
