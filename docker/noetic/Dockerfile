FROM jeguzzi/ros-aseba:noetic
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-noetic-xacro \
    ros-noetic-tf \
    ros-noetic-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros_ws/src && \
  cd /ros_ws/src && \
  git clone https://github.com/jeguzzi/ros-thymio.git --branch noetic && \
  cd /ros_ws && \
  source ./install/setup.bash && \
  catkin build && \
  rm -r build src
