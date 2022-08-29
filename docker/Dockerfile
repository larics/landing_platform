ARG DISTRO=bionic
FROM lmark1/uav_ros_stack:$DISTRO

ARG ROS_DISTRO_ARG=melodic
ARG CATKIN_WORKSPACE=uav_ws
ARG DEBIAN_FRONTEND=nointeractive
ARG HOME=/root

RUN apt-get update && \
    apt-get install -y \
        git \
        openssh-server \
        libmysqlclient-dev \
        ros-$ROS_DISTRO-realsense2-camera \
        ros-$ROS_DISTRO-rgbd-launch \
        ros-$ROS_DISTRO-openzen-sensor \
        libglew-dev \
        libglfw3 \
        libglfw3-dev  \
        vim  \
        nano \
        dnsmasq  \
        libgstreamer1.0-0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-doc \
        gstreamer1.0-tools \
        gstreamer1.0-x \
        gstreamer1.0-alsa \
        gstreamer1.0-gl \
        gstreamer1.0-gtk3 \
        gstreamer1.0-qt5 \
        gstreamer1.0-pulseaudio \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev \
        libgstreamer-plugins-bad1.0-dev \
        libgstrtspserver-1.0-dev \
        linuxptp && \
    pip install pypozyx

RUN curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules >> /etc/udev/rules.d/99-realsense-libusb.rules

ARG ROS_HOSTNAME=localhost.local
ARG ROS_MASTER_URI=http://localhost.local:11311
ARG ROS_IP=localhost.local

WORKDIR $HOME/$CATKIN_WORKSPACE/src/landing_platform
COPY . .
RUN catkin build --limit-status-rate 0.2 --this
RUN mkdir $HOME/bags
RUN mkdir $HOME/rosbag_default
