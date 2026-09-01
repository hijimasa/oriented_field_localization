#!/usr/bin/env bash
# oriented_field_localization を Docker 上の ROS 2 環境で build / test する。
#
#   ./docker/run_ci.sh            # イメージを用意して build + test
#   ./docker/run_ci.sh --shell    # 同じ環境の対話シェルへ入る
#
# パッケージは read-only で mount するので、ホストの作業ツリーは汚れない
# (build / install / log はコンテナ内の /ws に作られる)。
# イメージを作り直すときは REBUILD=1 を付ける。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${HERE}/.." && pwd)"
PKG_NAME="$(basename "${PKG_DIR}")"
IMAGE="${IMAGE:-ofl-ci:humble}"

if [ -z "$(docker images -q "${IMAGE}" 2>/dev/null)" ] || [ "${REBUILD:-0}" = "1" ]; then
    echo "[run_ci] building ${IMAGE}"
    docker build -t "${IMAGE}" "${HERE}"
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
        colcon build --packages-select '"${PKG_NAME}"' \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
        colcon test --packages-select '"${PKG_NAME}"'
        colcon test-result --verbose
    '
