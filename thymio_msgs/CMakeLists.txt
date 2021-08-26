cmake_minimum_required(VERSION 3.13.4)
project(thymio_msgs)


find_package(catkin REQUIRED COMPONENTS
	message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Comm.msg
  Led.msg
  LedGesture.msg
  Sound.msg
  SystemSound.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########


#############
## Install ##
#############
