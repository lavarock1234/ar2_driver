cmake_minimum_required(VERSION 2.8.3)
project(ar2_driver)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
  std_msgs
  sensor_msgs
  pluginlib
  message_generation
  urdf
  xacro    
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArmJointStep.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS scripts/driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

