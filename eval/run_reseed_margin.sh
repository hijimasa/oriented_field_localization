#!/usr/bin/env bash
# AMCL 監視の幾何判定 (supervise_max_wfrac / supervise_wfrac_excess) を
# 合成地図の 15 外乱条件で統計的に較正する。
#
#   ./run_reseed_margin.sh [out_dir] [poses]
#
# run_compare.sh と同じ地図・同じスキャンダンプを使うので、既に out/ があれば
# そのまま再利用する (無ければ作る)。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-${HERE}/out}"
POSES="${2:-40}"
MAP_DIR="${MAP_DIR:-${OUT}/maps}"
MAP_PGM="${OFL_MAP_PGM:-${MAP_DIR}/synthetic.pgm}"

mkdir -p "${OUT}"

CXXFLAGS="-O3 -fopenmp -std=c++17"
OPENCV="$(pkg-config --cflags --libs opencv4)"

for t in make_scans; do
    if [ ! -x "${OUT}/${t}" ] || [ "${HERE}/${t}.cpp" -nt "${OUT}/${t}" ]; then
        echo "[build] ${t}"
        # shellcheck disable=SC2086
        g++ ${CXXFLAGS} "${HERE}/${t}.cpp" ${OPENCV} -o "${OUT}/${t}"
    fi
done
if [ ! -x "${OUT}/reseed_margin_eval" ] \
   || [ "${HERE}/reseed_margin_eval.cpp" -nt "${OUT}/reseed_margin_eval" ] \
   || [ "${HERE}/../src/oriented_field_matcher.cpp" -nt "${OUT}/reseed_margin_eval" ]; then
    echo "[build] reseed_margin_eval"
    # shellcheck disable=SC2086
    g++ ${CXXFLAGS} "${HERE}/reseed_margin_eval.cpp" \
        "${HERE}/../src/oriented_field_matcher.cpp" \
        -I"${HERE}/../include" ${OPENCV} -o "${OUT}/reseed_margin_eval"
fi

if [ ! -f "${MAP_PGM}" ]; then
    echo "[map] 合成地図を生成 (${MAP_DIR})"
    python3 "${HERE}/make_synthetic_map.py" "${MAP_DIR}" > /dev/null
fi

export OFL_MAP_PGM="${MAP_PGM}"
export OFL_MAP_RES="${OFL_MAP_RES:-0.02}"
export OFL_MAX_RANGE="${OFL_MAX_RANGE:-10}"
export OFL_MATCH_RES="${OFL_MATCH_RES:-0.05}"
export OFL_MARGIN_M="${OFL_MARGIN_M:-10}"
export OFL_NEAR_WALL_M="${OFL_NEAR_WALL_M:-8}"

if [ ! -s "${OUT}/scans.csv" ]; then
    echo "[run] make_scans (${POSES} 姿勢 x 15 条件)"
    "${OUT}/make_scans" "${POSES}" > "${OUT}/scans.csv" 2> "${OUT}/scans.log"
fi

echo "[run] reseed_margin_eval"
"${OUT}/reseed_margin_eval" "${OUT}/scans.csv" "${MAP_PGM}" \
    > "${OUT}/reseed_margin.csv" 2> "${OUT}/reseed_margin.log"

python3 "${HERE}/summarize_reseed.py" "${OUT}/reseed_margin.csv" | tee "${OUT}/reseed_margin.txt"
