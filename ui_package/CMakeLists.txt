cmake_minimum_required(VERSION 3.5)
project(ui_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArrayPoseStampedWithCovariance.msg"
  DEPENDENCIES geometry_msgs
)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
 # std_msgs
#  geometry_msgs
#)



# Export message runtime dependency
ament_export_dependencies(message_runtime)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# # Declare a C++ executable
# add_executable(folders_handler
#   src/folders_handler.py
# )

# # Link the executable to ROS2 libraries
# target_link_libraries(folders_handler
#   ${rclcpp_LIBRARIES}
#   ${std_msgs_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
# )

# Add dependencies for message generation
# add_dependencies(folders_handler
#   ${PROJECT_NAME}_generate_messages_cpp
# )

# # Install the executable
# install(TARGETS
#   folders_handler
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python scripts (if any)
# install(PROGRAMS
#   scripts/folders_handler.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY
  launch
  maps
  param
  paths
  msg
  src
  DESTINATION share/${PROJECT_NAME}
)

# Install message and service files
install(FILES
  msg/ArrayPoseStampedWithCovariance.msg
  DESTINATION share/${PROJECT_NAME}/msg
)
install(PROGRAMS
  scripts/folders_handler.py
  scripts/way_points_handler.py
  scripts/gui_map_downsampler.py
  src/flask_app.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
)

# Finalize the package
ament_package()
