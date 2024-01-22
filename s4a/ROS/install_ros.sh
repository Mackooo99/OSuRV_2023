#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt -y install curl
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt -y install \
	ros-noetic-desktop-full \
	ros-noetic-joy \
	ros-noetic-effort-controllers \
	ros-noetic-velocity-controllers \
	ros-noetic-joint-trajectory-controller \
	ros-noetic-rqt \
	ros-noetic-rqt-common-plugins \
	ros-noetic-plotjuggler-ros \
	ros-noetic-moveit \
	ros-noetic-moveit-resources-prbt-moveit-config \
	ros-noetic-pilz-industrial-motion-planner \
	ros-noetic-moveit-servo \
	ros-noetic-ackermann-msgs \
	ros-noetic-ackermann-steering-controller \
	tmux xsel

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
