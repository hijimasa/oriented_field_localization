# oriented_field_localization をビルド・テストするための最小 ROS 2 環境。
#
# 開発機に ROS を入れずに colcon build / colcon test を通すためのもの。
# package.xml / CMakeLists.txt が要求する依存だけを入れる (PCL は不要)。
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes

RUN set -x && \
    apt-get update -y -qq && \
    apt-get install -y -qq --no-install-recommends \
      build-essential \
      cmake \
      libopencv-dev \
      python3-colcon-common-extensions \
      python3-numpy \
      ros-humble-ament-cmake \
      ros-humble-ament-lint-auto \
      ros-humble-ament-lint-common \
      ros-humble-geometry-msgs \
      ros-humble-nav-msgs \
      ros-humble-rclcpp \
      ros-humble-sensor-msgs \
      ros-humble-std-srvs \
      ros-humble-tf2-ros && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

WORKDIR /ws
ENTRYPOINT ["/bin/bash"]
