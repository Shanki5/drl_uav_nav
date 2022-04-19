FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Set non-interactive installation
ARG DEBIAN_FRONTEND=noninteractive

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

### Install essentials, toolchain, python, cudnn...
ARG PYTHON_VERSION=3
RUN apt update && \
    apt install -yq --no-install-recommends \
        apt-utils \
        locales \
        locales-all \
        tzdata \
        software-properties-common \
        git \
        wget \
        curl \
        gnupg \
        lsb-release \
        build-essential \
        make \
        cmake \
        g++ \
        autoconf \
        automake \
        clang \
        ninja-build \
        python${PYTHON_VERSION} \
        python${PYTHON_VERSION}-dev \
        python${PYTHON_VERSION}-pip \
        python${PYTHON_VERSION}-wheel \
        python${PYTHON_VERSION}-numpy \
        python${PYTHON_VERSION}-tk \
        python${PYTHON_VERSION}-pybind11 \
        libpython${PYTHON_VERSION}-dev \
        libopenmpi-dev \
        zlib1g-dev \
        libcudnn8-dev \
        nano && \
    rm -rf /var/lib/apt/lists/*

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

### Set working directory
ARG DRL_DRONE_NAVDIR=/root/drl_drone_ws
ENV DRL_DRONE_NAVDIR=${DRL_DRONE_NAVDIR}
WORKDIR ${DRL_DRONE_NAVDIR}

# Get Base Packages
RUN source /opt/ros/noetic/setup.bash \ 
 && mkdir /root/drl_drone_ws/src \
 && cd /root/drl_drone_ws/src \
 && catkin_init_workspace \
 && git clone -b version2 https://bitbucket.org/theconstructcore/openai_ros.git \
 && git clone -b version2 https://bitbucket.org/theconstructcore/openai_examples_projects.git \
 && git clone -b kinetic-gazebo9 https://bitbucket.org/theconstructcore/parrot_ardrone.git

RUN source /opt/ros/noetic/setup.bash \
 && cd /root/drl_drone_ws \
 && rosdep install --from-paths src --ignore-src -r -y \
 && catkin_make \
 && source devel/setup.bash

RUN echo "source /root/drl_drone_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR ${DRL_DRONE_NAVDIR}

### Add entrypoint sourcing the environment
COPY ./entrypoint.bash ./entrypoint.bash

### Set entrypoint and default command
ENTRYPOINT ["/bin/bash", "-c", "source ${DRL_DRONE_NAVDIR}/entrypoint.bash && \"$@\"", "-s"]
CMD ["/bin/bash"]
