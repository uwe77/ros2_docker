FROM nvidia/cuda:12.0.0-cudnn8-devel-ubuntu22.04


ARG DEBIAN_FRONTEND=noninteractive
ARG SKIP_AP_EXT_ENV=0
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1
ARG DO_AP_STM_ENV=1

ENV ROS_DISTRO=humble \
    GZ_VERSION=harmonic \
    SHELL=/bin/bash \
    NB_USER=uwe \
    NB_UID=1000

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime


###################################### user #####################################
RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER} 

RUN echo "root:root" | chpasswd \
    && echo "${NB_USER}:111111" | chpasswd

ENV HOME=/home/${NB_USER}

###################################### basic #####################################
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends \
    tzdata \
    pip \
    dirmngr \
    gnupg2 \
    git \
    gitk git-gui \
    vim \
    less \
    curl \
    wget \
    sudo \
    cmake \
    net-tools \
    lsb-release \
    gnupg \
    build-essential \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

###################################### ros2 ####################################
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

###################################### gazebo ##################################
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
    gz-${GZ_VERSION} \
    ros-${ROS_DISTRO}-ros-gz${GZ_VERSION} && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep init && \
    rosdep update

###################################### python pkg###############################
RUN apt-get update && apt-get upgrade pip -y && \
    pip install --upgrade pip && \
    pip install \
    numpy \
    scipy \
    matplotlib \
    scikit-learn \
    scikit-image \
    pandas \
    opencv-python \
    opencv-python-headless \
    pyyaml \
    pyquaternion \
    pyserial

###################################### ardupilot gz ################################
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends \
    libgz-sim8-dev \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl

RUN bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list' && \
    rosdep update && \
    rosdep resolve gz-${GZ_VERSION} && \
    mkdir -p ${HOME}/ros2_ws/src && \
    rosdep install --from-paths ${HOME}/ros2_ws/src --ignore-src -y && \
    rm -r ${HOME}/ros2_ws

RUN cd /opt && \
    git clone https://github.com/ArduPilot/ardupilot_gazebo && \
    cd ardupilot_gazebo && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j4

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH \
    GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

RUN echo "root ALL=(ALL)  ALL" >> /etc/sudoers \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc 

ENV XLA_PYTHON_CLIENT_PREALLOCATE=false
ENV HF_HUB_ENABLE_HF_TRANSFER=1