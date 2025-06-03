cmake_minimum_required(VERSION 3.5)
project(my_robot_bringup)

# Set default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)

# Install launch files (*.py)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files (*.yaml)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install RViz config files (*.rviz)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.rviz"
)

# (Optional) Install package resource files if using Python modules
install(
  DIRECTORY my_robot_bringup/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "__init__.py"
)

ament_package()
