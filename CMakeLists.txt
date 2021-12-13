cmake_minimum_required(VERSION 2.8.3)
project(roas6000_final)

find_package(catkin REQUIRED
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(OpenCV 3 REQUIRED)

catkin_package()


include_directories(
 
   ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   scripts/cv_detect_cus_test.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/teleop_twist_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/dlib_detection.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
	
catkin_install_python(PROGRAMS
   scripts/area_check.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/visual_servoing.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
