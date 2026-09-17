cmake_minimum_required(VERSION 3.5)
# find_package(Python3 COMPONENTS Interpreter REQUIRED)
project(rover_simulation LANGUAGES C CXX)
# add_executable(dummy src/dummy.cpp)  # Removed dummy target
# install(TARGETS dummy DESTINATION lib/${PROJECT_NAME})  # Removed dummy install

# Find ament packages
find_package(ament_cmake REQUIRED)
# Only ament_cmake is strictly required to build and install data files.
# The following are runtime dependencies usually declared in package.xml
# find_package(rclpy REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(ros_gz_sim REQUIRED)
# find_package(ros_gz_bridge REQUIRED)
# find_package(xacro REQUIRED)
# find_package(rviz2 REQUIRED)

# Install Python package
# ament_python_install_package(${PROJECT_NAME})  # Removed to avoid Python compiler requirement

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

# Rover control nodes
install(PROGRAMS
  rover_simulation/rover_teleop.py
  rover_simulation/rover_patrol.py
  rover_simulation/rover_keyboard.py
  rover_simulation/rover_creep.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
