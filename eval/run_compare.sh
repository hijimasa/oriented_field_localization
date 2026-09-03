#!/usr/bin/env bash
# 合成地図の上で本手法 (画像空間の向き付き場相関) と BBS を同一スキャンで比較する。
#
#   ./run_compare.sh [out_dir] [poses]
#
# 手順:
#   1. make_synthetic_map.py  … 第三者データを含まない合成地図を生成する
#   2. make_scans             … 姿勢を撒き、外乱付きスキャンを CSV へ書き出す
#   3. bbs_eval               … そのダンプを BBS (Olson/Hess 型 分枝限定) で解く
#   4. ofl_eval               … 同じダンプを本手法で解く
# 両者がバイト同一のスキャンを見るので対応比較になる。
#
# 既定では、条件変更後に古い CSV を誤って集計しないよう全結果を再生成する。
# 既存結果を意図的に使う場合に限り REUSE_RESULTS=1 を指定する。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-${HERE}/out}"
POSES="${2:-20}"
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

echo "[build] ofl_eval"
# shellcheck disable=SC2086
g++ ${CXXFLAGS} "${HERE}/ofl_eval.cpp" "${HERE}/../src/oriented_field_matcher.cpp" \
    -I"${HERE}/../include" ${OPENCV} -o "${OUT}/ofl_eval"
for t in make_scans bbs_eval; do
    echo "[build] ${t}"
    # shellcheck disable=SC2086
    g++ ${CXXFLAGS} "${HERE}/${t}.cpp" ${OPENCV} -o "${OUT}/${t}"
done

if [ -n "${OFL_MAP_PGM:-}" ]; then
    if [ ! -f "${MAP_PGM}" ]; then
        echo "指定された OFL_MAP_PGM が存在しない: ${MAP_PGM}" >&2
        exit 2
    fi
elif [ "${REUSE_RESULTS}" != "1" ] || [ ! -f "${MAP_PGM}" ]; then
    echo "[map] 合成地図を生成 (${MAP_DIR})"
    python3 "${HERE}/make_synthetic_map.py" "${MAP_DIR}" > /dev/null
fi

# 評価器へ渡す共通設定 (config/params.yaml の既定と一致させてある)
export OFL_MAP_PGM="${MAP_PGM}"
export OFL_MAP_RES="${OFL_MAP_RES:-0.02}"
export OFL_MAX_RANGE="${OFL_MAX_RANGE:-10}"
export OFL_MATCH_RES="${OFL_MATCH_RES:-0.05}"
export OFL_MARGIN_M="${OFL_MARGIN_M:-10}"
export OFL_NEAR_WALL_M="${OFL_NEAR_WALL_M:-8}"
export OFL_LEVELS="${OFL_LEVELS:-3}"
export OFL_ANGLE_STEP="${OFL_ANGLE_STEP:-6}"
export OFL_WFRAC_MARGIN="${OFL_WFRAC_MARGIN:-1.05}"
export BBS_SIGMA="${BBS_SIGMA:-0.1}"
export BBS_TRUNC="${BBS_TRUNC:-0.5}"

if [ "${REUSE_RESULTS}" = "1" ]; then
    for artifact in scans.csv bbs.csv ofl.csv run_compare.manifest; do
        if [ ! -s "${OUT}/${artifact}" ]; then
            echo "REUSE_RESULTS=1 だが ${OUT}/${artifact} が無いか空である" >&2
            exit 2
        fi
    done
    echo "[reuse] 既存の scans/ofl/bbs CSV を明示的に再利用"
else
    echo "[run] make_scans (${POSES} 姿勢 x 15 条件)"
    "${OUT}/make_scans" "${POSES}" > "${OUT}/scans.csv" 2> "${OUT}/scans.log"
    echo "[run] bbs_eval (角度 1.0 度, 格子 0.05 m)"
    "${OUT}/bbs_eval" "${OUT}/scans.csv" "${MAP_PGM}" 1.0 0.05 0.0 \
        > "${OUT}/bbs.csv" 2> "${OUT}/bbs.log"
    echo "[run] ofl_eval (levels ${OFL_LEVELS}, 角度 ${OFL_ANGLE_STEP} 度)"
    OFL_STAGEDUMP=1 "${OUT}/ofl_eval" "${OUT}/scans.csv" "${MAP_PGM}" \
        > "${OUT}/ofl.csv" 2> "${OUT}/ofl.log"
    {
        echo "generated_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "requested_poses=${POSES}"
        echo "map_pgm=${MAP_PGM}"
        echo "map_sha256=$(sha256sum "${MAP_PGM}" | awk '{print $1}')"
        echo "bbs_angle_step_deg=1.0"
        echo "bbs_grid_m=0.05"
        echo "bbs_min_range_m=0.0"
        echo "cxx=$(g++ --version | head -n 1)"
        echo "cxxflags=${CXXFLAGS}"
        env | LC_ALL=C sort | sed -n '/^OFL_/p; /^BBS_/p'
        sha256sum "${HERE}/make_synthetic_map.py" "${HERE}/make_scans.cpp" \
            "${HERE}/bbs_eval.cpp" "${HERE}/ofl_eval.cpp" \
            "${HERE}/summarize.py" "${HERE}/../src/oriented_field_matcher.cpp" \
            "${OUT}/scans.csv" "${OUT}/bbs.csv" "${OUT}/ofl.csv"
    } > "${OUT}/run_compare.manifest"
fi

echo
python3 "${HERE}/summarize.py" "ofl=${OUT}/ofl.csv" "bbs=${OUT}/bbs.csv"
echo
sed -n '/\[stage\]/,$p' "${OUT}/ofl.log" || true
echo "生成物: ${OUT}/{scans,ofl,bbs}.csv"
echo "provenance: ${OUT}/run_compare.manifest"
