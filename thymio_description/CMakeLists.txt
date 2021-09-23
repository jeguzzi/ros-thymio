cmake_minimum_required(VERSION 3.0.2)
project(thymio_description)

find_package(catkin REQUIRED COMPONENTS
	urdf
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS urdf
)

###########
## Build ##
###########


#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

catkin_install_python(PROGRAMS script/generate_rviz_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
