# Ubuntu bionic 18.04 cuda 10.0 cudnn7
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && /bin/bash -c "source ~/.bashrc"

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"

RUN mkdir -p catkin_ws/src

WORKDIR /catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace /catkin_ws/src'

WORKDIR /catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make'
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN apt-get update \
    && apt-get install -y ros-melodic-rosbridge-server \
    ros-melodic-cv-bridge \
    ros-melodic-vision-opencv \
    ros-melodic-image-common \
    && rm -rf /var/lib/apt/lists/*

COPY ./video_stream /catkin_ws/src/video_stream

WORKDIR /catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make'
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# setup entrypoint
COPY ./entrypoint.sh /

WORKDIR /

RUN chmod +x entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]
