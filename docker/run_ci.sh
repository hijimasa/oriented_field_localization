#!/usr/bin/env bash
# oriented_field_localization を Docker 上の ROS 2 環境で build / test する。
#
#   ./docker/run_ci.sh            # イメージを用意して build + test
#   ./docker/run_ci.sh --shell    # 同じ環境の対話シェルへ入る
#
# パッケージは read-only で mount するので、ホストの作業ツリーは汚れない
# (build / install / log はコンテナ内の /ws に作られる)。
# 既定でDockerfileからイメージを作り直す。ローカルの既存イメージを意図的に
# 使う場合だけ REUSE_IMAGE=1 を付ける。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${HERE}/.." && pwd)"
PKG_NAME="$(basename "${PKG_DIR}")"
IMAGE="${IMAGE:-oriented-field-localization-ci:humble}"

if [ "${REUSE_IMAGE:-0}" != "1" ] || [ -z "$(docker images -q "${IMAGE}" 2>/dev/null)" ]; then
    echo "[run_ci] building ${IMAGE}"
    docker build --pull -t "${IMAGE}" "${HERE}"
else
    echo "[run_ci] explicitly reusing ${IMAGE}"
fi

if [ "${1:-}" = "--shell" ]; then
    exec docker run --rm -it \
        -v "${PKG_DIR}:/ws/src/${PKG_NAME}:ro" \
        -w /ws "${IMAGE}"
fi

exec docker run --rm \
    -v "${PKG_DIR}:/ws/src/${PKG_NAME}:ro" \
    -w /ws "${IMAGE}" -lc '
        # ROS の setup.bash は未定義変数を参照するので set -u は使わない
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        echo "[environment] container_id=$(cat /etc/hostname)"
        echo "[environment] base_image=${OFL_ROS_BASE_IMAGE}"
        echo "[environment] $(. /etc/os-release && echo "${PRETTY_NAME}") / ROS ${ROS_DISTRO}"
        LC_ALL=C dpkg-query -W \
            build-essential cmake libopencv-dev libyaml-cpp-dev python3-colcon-common-extensions \
            ros-humble-ament-cmake ros-humble-rclcpp ros-humble-tf2-ros \
            | sed "s/^/[environment] /"
        colcon build --packages-select '"${PKG_NAME}"' \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
        colcon test --packages-select '"${PKG_NAME}"'
        colcon test-result --verbose
        source /ws/install/setup.bash
        cmake -S /ws/src/'"${PKG_NAME}"'/tests/downstream -B /tmp/ofl-downstream
        cmake --build /tmp/ofl-downstream --parallel 2
        /tmp/ofl-downstream/downstream_smoke
    '
