# ARG is overridable by --build-arg
ARG UBUNTU_CODENAME=focal
ARG ROS_DISTRO=foxy

FROM arm32v7/ubuntu:${UBUNTU_CODENAME}

# NOTE: An ARG declared before a FROM is outside of a build stage, so it can’t be used in any instruction after a FROM.
# To use the default value of an ARG declared before the first FROM use an ARG instruction without a value inside of a build stage
ARG UBUNTU_CODENAME
ARG ROS_DISTRO

# Set locale
RUN apt-get update && apt-get install -y locales \
	&& locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

ENV LANG en_US.utf8

# Add the ROS 2 apt repository
RUN apt-get update && apt-get install -y software-properties-common \
  && add-apt-repository universe \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && rm -rf /var/lib/apt/lists/*

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools
RUN apt-get update && apt-get install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools \
  && rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

## install Fast-RTPS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  libasio-dev \
  libtinyxml2-dev \
  && rm -rf /var/lib/apt/lists/*

## install Cyclone DDS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  libcunit1-dev \
  && rm -rf /var/lib/apt/lists/*

# Get ROS 2 code
RUN update-ca-certificates --fresh \
  && mkdir -p /root/ros2_ws/src \
  && vcs import --input https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos /root/ros2_ws/src

# Install dependencies using rosdep
RUN apt-get update && apt-get upgrade \
  && update-ca-certificates --fresh \
  && rosdep init \
  && rosdep update \
  && rosdep install --from-paths /root/ros2_ws/src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers" \
  && rm -rf /var/lib/apt/lists/*

# Install latest cmake
RUN apt-get update && apt-get install gpg wget \
  && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
  && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ ${UBUNTU_CODENAME} main" | tee /etc/apt/sources.list.d/kitware.list >/dev/null \
  && apt-get update \
  && apt-get install -y cmake \
  && rm -rf /var/lib/apt/lists/*

RUN cd /root/ros2_ws && colcon build --symlink-install
