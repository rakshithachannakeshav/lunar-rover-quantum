cmake_minimum_required(VERSION 3.5)
project(sensor_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  sensor_pkg/lidar_processor.py
  sensor_pkg/imu_processor.py
  sensor_pkg/encoder_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
