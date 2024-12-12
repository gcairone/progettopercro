# from https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/noetic/ubuntu/focal/ros-base/Dockerfile
# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install coreresearch drivers and opencv
RUN apt-get update && \
    apt-get install -y curl && \
    curl -Ls http://deb.7sr.ch/pubkey.gpg | gpg --dearmor -o /usr/share/keyrings/deb-7sr-ch-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/deb-7sr-ch-keyring.gpg] http://deb.7sr.ch/alphasense/stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/sevensense.list && \
    apt-get update && \
    apt-get install -y alphasense-driver-core alphasense-viewer alphasense-firmware ros-noetic-alphasense-driver-ros ros-noetic-alphasense-driver && \
    apt-get install -y libopencv-dev python3-opencv && \
    rm -rf /var/lib/apt/lists/*

# install packages for openvins
RUN apt-get update && \
apt-get install -y python3-catkin-tools \
                   ros-noetic-cv-bridge \
                   libcereal-dev \
                   libceres-dev \
                   ros-noetic-rviz \
                   ros-noetic-image-transport-plugins && \
rm -rf /var/lib/apt/lists/*

# build openvins
WORKDIR /root/catkin_ws_ov/
RUN catkin clean -y 
RUN catkin build -j1 -p1
