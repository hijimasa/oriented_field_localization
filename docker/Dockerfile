# oriented_field_localization をビルド・テストするための最小 ROS 2 環境。
#
# 開発機に ROS を入れずに colcon build / colcon test を通すためのもの。
# package.xml / CMakeLists.txt が要求する依存だけを入れる (PCL は不要)。
# 2026-09-03 に検証したmulti-architecture manifestへ固定する。更新時はCI結果と
# docker/run_ci.sh が出力するpackage versionを確認してdigestを差し替える。
ARG ROS_BASE_IMAGE=ros:humble-ros-base@sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987
FROM ${ROS_BASE_IMAGE}
ARG ROS_BASE_IMAGE
LABEL org.opencontainers.image.base.name="${ROS_BASE_IMAGE}"
ENV OFL_ROS_BASE_IMAGE="${ROS_BASE_IMAGE}"

ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes

RUN set -x && \
    apt-get update -y -qq && \
    apt-get install -y -qq --no-install-recommends \
      build-essential \
      cmake \
      libopencv-dev \
      libyaml-cpp-dev \
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
