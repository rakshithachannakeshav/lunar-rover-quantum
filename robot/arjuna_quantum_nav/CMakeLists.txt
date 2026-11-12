cmake_minimum_required(VERSION 2.8.3)
project(arjuna_quantum_nav)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/quantum_nav_node.py
  scripts/drive_test.py
  scripts/nav_core.py
  scripts/crater_detector_node.py
  scripts/crater_core.py
  scripts/terrain_core.py
  scripts/terrain_objects.py
  scripts/make_arena_map.py
  scripts/compare_runs.py
  scripts/compare_plans.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
