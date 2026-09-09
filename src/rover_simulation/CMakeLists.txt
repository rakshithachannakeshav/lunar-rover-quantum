cmake_minimum_required(VERSION 3.5)
project(rover_simulation LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)

# Data files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Rover control nodes (flat scripts)
install(PROGRAMS
  rover_simulation/rover_patrol.py
  rover_simulation/rover_keyboard.py
  rover_simulation/rover_creep.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
