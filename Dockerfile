FROM ubuntu:focal
MAINTAINER jc.manzanares.serrano@gmail.com
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt update
RUN apt-get install -y software-properties-common

# Install ROS Noetic 
RUN apt install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN add-apt-repository 'deb http://packages.ros.org/ros/ubuntu focal main'
RUN apt update
RUN apt install -y ros-noetic-ros-base ros-noetic-std-msgs ros-noetic-tf2-msgs ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs ros-noetic-moveit

# Install ROS2 Deps 
RUN apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN /bin/bash -c "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main' | tee /etc/apt/sources.list.d/ros2.list > /dev/null"
RUN apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  nano
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
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
RUN apt install --no-install-recommends -y \
  libcunit1-dev

RUN useradd -ms /bin/bash bridge
WORKDIR /home/bridge
USER bridge

RUN mkdir -p ros2_foxy_bridge_ws/src
WORKDIR /home/bridge/ros2_foxy_bridge_ws
RUN wget https://raw.githubusercontent.com/juandpenan/ros1_bridge/robocup/ros2_foxy.repos
RUN vcs import src < ros2_foxy.repos

USER root
RUN rosdep init
RUN rosdep update && rosdep install --from-paths src --ignore-src -y --skip-keys 'fastcdr rti-connext-dds-5.3.1 urdfdom_headers'


# Build ROS2 and Bridge
USER bridge
RUN colcon build --symlink-install --packages-skip rviz rviz2 rviz_assimp_vendor rviz_common rviz_default_plugins rviz_ogre_vendor rviz_rendering rviz_rendering_tests rviz_visual_testing_framework ros1_bridge
RUN source install/setup.bash
RUN source /opt/ros/noetic/setup.bash
RUN source install/setup.bash && source /opt/ros/noetic/setup.bash && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

COPY entrypoint.sh /home/bridge
# RUN chmod 666 /home/bridge/entrypoint.sh
ENTRYPOINT ["/home/bridge/entrypoint.sh"]
CMD ["bash"]

