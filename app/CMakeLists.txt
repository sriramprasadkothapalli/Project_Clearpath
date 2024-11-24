# Any C++ source files needed to build this target (shell-app).
add_executable(collector_node
  # list of source cpp files:
  main.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(collector_node PUBLIC
  # list inclue directories:
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/libs/debris
  ${OpenCV_INCLUDE_DIRS}  # Add OpenCV include directories
)

# Any dependent libraires needed to build this target.
target_link_libraries(collector_node PUBLIC 
  # list of libraries
  myDebris
  ${OpenCV_LIBS}
  )

# Install the collector_node executable
install(TARGETS collector_node
  DESTINATION lib/${PROJECT_NAME}  # Installs in lib/project_clearpath
)