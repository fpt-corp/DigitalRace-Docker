FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

LABEL maintainer "Aperture Prometheus <aperture147@gmail.com>"
LABEL maintainer "Dzung Phanba <badungphan99@gmail.com>"

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO melodic

RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

# setup keys and sources.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        cmake \
        dirmngr \
        gnupg2 \
        tzdata \
        python-rosdep \
        python-rosinstall \
        python-vcstools \
        ros-$ROS_DISTRO-ros-base \
        ros-$ROS_DISTRO-rosbridge-server \
        ros-$ROS_DISTRO-cv-bridge \
        ros-$ROS_DISTRO-vision-opencv \
        ros-$ROS_DISTRO-image-common \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && rosdep update

COPY ./video_stream /catkin_ws/src/video_stream
COPY ./entrypoint.sh /

# Setup video stream
WORKDIR /catkin_ws
RUN mkdir -p src \
    && /bin/sh -c '. /opt/ros/$ROS_DISTRO/setup.bash; catkin_init_workspace /catkin_ws/src' \
    && /bin/sh -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /catkin_ws; catkin_make' \
    && chmod +x entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Prevent users from running this image without adding cmd
# CMD ["/bin/bash"]