FROM arm32v7/ubuntu:focal

# Set locale
RUN apt update && apt install -y locales && rm -rf /var/lib/apt/lists/* \
	&& locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.utf8

# Add the ROS 2 apt repository
RUN apt install software-properties-common && add-apt-repository universe

RUN apt update && apt install curl gnupg2 lsb-release && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools
RUN sudo apt update && sudo apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

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
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
## install Cyclone DDS dependencies
RUN apt install --no-install-recommends -y \
  libcunit1-dev

# Get ROS 2 code
RUN mkdir -p /root/ros2_foxy/src && \
  cd /root/ros2_foxy && \
  vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src

# Install dependencies using rosdep
RUN apt upgrade
RUN rosdep init && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

RUN cd /root/ros2_foxy && \
  colcon build --symlink-install
