# Ubuntu bionic 18.04
FROM ubuntu:bionic

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# Setup your sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installation
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

RUN apt install ros-melodic-ros-base

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN rosdep init \
    && rosdep update \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

RUN apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

RUN source /opt/ros/melodic/setup.bash

RUN mkdir -p /catkin_ws/src \
    && cd /catkin_ws/ \
    && catkin_make

RUN apt-get install ros-lunar-rosbridge-server

COPY ./video_stream /catkin_ws/src

RUN cd /catkin_ws \
    && catkin_make

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
