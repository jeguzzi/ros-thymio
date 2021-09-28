FROM jeguzzi/ros-aseba:galactic
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-galactic-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros_ws/src && \
  cd /ros_ws/src && \
  git clone https://github.com/jeguzzi/ros-thymio.git --branch master && \
  cd /ros_ws && \
  source ./install/setup.bash && \
  colcon build --merge-install && \
  rm -r build log src
