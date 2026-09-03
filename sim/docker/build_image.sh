#!/usr/bin/env bash
# sim/run_sim.sh と sim/run_nav2.sh が使うイメージを作る。
#
#   ./sim/docker/build_image.sh            # bac_gazebo_runtime:humble を作る
#   IMAGE=my:tag ./sim/docker/build_image.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${IMAGE:-bac_gazebo_runtime:humble}"
echo "[build_image] building ${IMAGE} (Gazebo Classic + Nav2, 数 GB 落ちてくる)"
docker build -t "${IMAGE}" "${HERE}"
echo "[build_image] done: ${IMAGE}"
