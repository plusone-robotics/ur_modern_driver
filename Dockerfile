FROM ros:kinetic-ros-core


RUN apt-get update

RUN apt-get update && apt-get install -y --no-install-recommends \
  python-wstool \
  python-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws/src/



