# Gazebo Classic での連続走行検証 (sim/run_sim.sh, sim/run_nav2.sh) を回すための環境。
#
# 中身は Gazebo Classic 11 + gazebo_ros + Nav2 + このパッケージのビルドに要るもの
# だけ。パッケージ自体はイメージに焼かず、実行時に read-only で mount して
# コンテナの中で colcon build する (run_nav2.sh を参照)。
#
#   ./sim/docker/build_image.sh
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      libopencv-dev \
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
