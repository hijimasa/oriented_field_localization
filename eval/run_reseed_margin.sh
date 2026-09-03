#!/usr/bin/env bash
# AMCL 監視の幾何判定 (supervise_max_wfrac / supervise_wfrac_excess) を
# 合成地図の 15 外乱条件で統計的に較正する。
#
#   ./run_reseed_margin.sh [out_dir] [poses]
#
# 既定では地図とスキャンを再生成する。run_compare.sh が同じ条件で生成した
# scans.csv を意図的に使う場合に限り REUSE_RESULTS=1 を指定する。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-${HERE}/out}"
POSES="${2:-40}"
MAP_DIR="${MAP_DIR:-${OUT}/maps}"
MAP_PGM="${OFL_MAP_PGM:-${MAP_DIR}/synthetic.pgm}"
REUSE_RESULTS="${REUSE_RESULTS:-0}"

case "${REUSE_RESULTS}" in
  0 | 1) ;;
  *) echo "REUSE_RESULTS は 0 または 1 を指定すること" >&2; exit 2 ;;
esac

mkdir -p "${OUT}"

CXXFLAGS="-O3 -fopenmp -std=c++17"
OPENCV="$(pkg-config --cflags --libs opencv4)"

echo "[build] make_scans"
# shellcheck disable=SC2086
g++ ${CXXFLAGS} "${HERE}/make_scans.cpp" ${OPENCV} -o "${OUT}/make_scans"
echo "[build] reseed_margin_eval"
# shellcheck disable=SC2086
g++ ${CXXFLAGS} "${HERE}/reseed_margin_eval.cpp" \
    "${HERE}/../src/oriented_field_matcher.cpp" \
    -I"${HERE}/../include" ${OPENCV} -o "${OUT}/reseed_margin_eval"

if [ -n "${OFL_MAP_PGM:-}" ]; then
    if [ ! -f "${MAP_PGM}" ]; then
        echo "指定された OFL_MAP_PGM が存在しない: ${MAP_PGM}" >&2
        exit 2
    fi
elif [ "${REUSE_RESULTS}" != "1" ] || [ ! -f "${MAP_PGM}" ]; then
    echo "[map] 合成地図を生成 (${MAP_DIR})"
    python3 "${HERE}/make_synthetic_map.py" "${MAP_DIR}" > /dev/null
fi

export OFL_MAP_PGM="${MAP_PGM}"
export OFL_MAP_RES="${OFL_MAP_RES:-0.02}"
export OFL_MAX_RANGE="${OFL_MAX_RANGE:-10}"
export OFL_MATCH_RES="${OFL_MATCH_RES:-0.05}"
export OFL_MARGIN_M="${OFL_MARGIN_M:-10}"
export OFL_NEAR_WALL_M="${OFL_NEAR_WALL_M:-8}"

if [ "${REUSE_RESULTS}" = "1" ]; then
    if [ ! -s "${OUT}/scans.csv" ]; then
        echo "REUSE_RESULTS=1 だが ${OUT}/scans.csv が無いか空である" >&2
        exit 2
    fi
    if [ ! -s "${OUT}/run_compare.manifest" ] &&
      [ ! -s "${OUT}/run_reseed_margin.manifest" ]
    then
        echo "REUSE_RESULTS=1 だがscan生成時のmanifestが無い" >&2
        exit 2
    fi
    echo "[reuse] 既存の scans.csv を明示的に再利用"
    SCAN_SOURCE="existing_explicit_reuse"
else
    echo "[run] make_scans (${POSES} 姿勢 x 15 条件)"
    "${OUT}/make_scans" "${POSES}" > "${OUT}/scans.csv" 2> "${OUT}/scans.log"
    SCAN_SOURCE="fresh"
fi

echo "[run] reseed_margin_eval"
"${OUT}/reseed_margin_eval" "${OUT}/scans.csv" "${MAP_PGM}" \
    > "${OUT}/reseed_margin.csv" 2> "${OUT}/reseed_margin.log"

{
    echo "generated_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "reuse_results=${REUSE_RESULTS}"
    echo "scan_source=${SCAN_SOURCE}"
    echo "requested_poses_argument=${POSES}"
    echo "map_pgm=${MAP_PGM}"
    echo "map_sha256=$(sha256sum "${MAP_PGM}" | awk '{print $1}')"
    echo "scans_sha256=$(sha256sum "${OUT}/scans.csv" | awk '{print $1}')"
    echo "result_sha256=$(sha256sum "${OUT}/reseed_margin.csv" | awk '{print $1}')"
    echo "cxx=$(g++ --version | head -n 1)"
    echo "cxxflags=${CXXFLAGS}"
    env | LC_ALL=C sort | sed -n '/^OFL_/p'
    sha256sum "${HERE}/make_synthetic_map.py" "${HERE}/make_scans.cpp" \
        "${HERE}/reseed_margin_eval.cpp" "${HERE}/summarize_reseed.py" \
        "${HERE}/../src/oriented_field_matcher.cpp"
} > "${OUT}/run_reseed_margin.manifest"

python3 "${HERE}/summarize_reseed.py" "${OUT}/reseed_margin.csv" | tee "${OUT}/reseed_margin.txt"
