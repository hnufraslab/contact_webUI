cmake_minimum_required(VERSION 3.0.2)
project(contact_webui)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs std_msgs
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/pointcloud_processor.py
  src/pose_to_web_frame.py
  test/test_publisher.py
  test/test_pcd_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install web files
install(DIRECTORY js/ css/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES index.html
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
