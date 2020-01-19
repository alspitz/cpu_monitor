cmake_minimum_required(VERSION 2.8.3)
project(cpu_monitor)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
install(PROGRAMS
   monitor.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN *~ EXCLUDE
)


