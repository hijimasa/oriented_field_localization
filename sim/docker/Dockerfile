# Gazebo Classic での連続走行検証 (sim/run_sim.sh, sim/run_nav2.sh) を回すための環境。
#
# 中身は Gazebo Classic 11 + gazebo_ros + Nav2 + このパッケージのビルドに要るもの
# だけ。パッケージ自体はイメージに焼かず、実行時に read-only で mount して
# コンテナの中で colcon build する (run_nav2.sh を参照)。
#
#   ./sim/docker/build_image.sh
# core CIと同じ、2026-09-03検証済みのmulti-architecture manifestへ固定する。
ARG ROS_BASE_IMAGE=ros:humble-ros-base@sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987
FROM ${ROS_BASE_IMAGE}
ARG ROS_BASE_IMAGE
LABEL org.opencontainers.image.base.name="${ROS_BASE_IMAGE}"
ENV OFL_ROS_BASE_IMAGE="${ROS_BASE_IMAGE}"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      libopencv-dev \
      libyaml-cpp-dev \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-plugins \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-robot-state-publisher \
      ros-humble-tf2-ros \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# gzserver は起動時にモデルデータベースを取りに行こうとする。sim は
# --network none で回すので、待たされないように無効化しておく。
ENV GAZEBO_MODEL_DATABASE_URI=""

SHELL ["/bin/bash", "-lc"]
