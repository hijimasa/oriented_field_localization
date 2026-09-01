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
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-${HERE}/out}"
POSES="${2:-20}"
MAP_DIR="${MAP_DIR:-${OUT}/maps}"
MAP_PGM="${OFL_MAP_PGM:-${MAP_DIR}/synthetic.pgm}"

mkdir -p "${OUT}"

CXXFLAGS="-O3 -fopenmp -std=c++17"
OPENCV="$(pkg-config --cflags --libs opencv4)"

if [ ! -x "${OUT}/ofl_eval" ] || [ "${HERE}/ofl_eval.cpp" -nt "${OUT}/ofl_eval" ] \
   || [ "${HERE}/../src/oriented_field_matcher.cpp" -nt "${OUT}/ofl_eval" ]; then
    echo "[build] ofl_eval"
    # shellcheck disable=SC2086
    g++ ${CXXFLAGS} "${HERE}/ofl_eval.cpp" "${HERE}/../src/oriented_field_matcher.cpp" \
        -I"${HERE}/../include" ${OPENCV} -o "${OUT}/ofl_eval"
fi
for t in make_scans bbs_eval; do
    if [ ! -x "${OUT}/${t}" ] || [ "${HERE}/${t}.cpp" -nt "${OUT}/${t}" ]; then
        echo "[build] ${t}"
        # shellcheck disable=SC2086
        g++ ${CXXFLAGS} "${HERE}/${t}.cpp" ${OPENCV} -o "${OUT}/${t}"
    fi
done

if [ ! -f "${MAP_PGM}" ]; then
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

if [ ! -s "${OUT}/scans.csv" ]; then
    echo "[run] make_scans (${POSES} 姿勢 x 15 条件)"
    "${OUT}/make_scans" "${POSES}" > "${OUT}/scans.csv" 2> "${OUT}/scans.log"
fi

if [ ! -s "${OUT}/bbs.csv" ]; then
    echo "[run] bbs_eval (角度 1.0 度, 格子 0.05 m)"
    BBS_SIGMA="${BBS_SIGMA:-0.1}" BBS_TRUNC="${BBS_TRUNC:-0.5}" \
        "${OUT}/bbs_eval" "${OUT}/scans.csv" "${MAP_PGM}" 1.0 0.05 0.0 \
        > "${OUT}/bbs.csv" 2> "${OUT}/bbs.log"
fi

if [ ! -s "${OUT}/ofl.csv" ]; then
    echo "[run] ofl_eval (levels ${OFL_LEVELS}, 角度 ${OFL_ANGLE_STEP} 度)"
    OFL_STAGEDUMP=1 "${OUT}/ofl_eval" "${OUT}/scans.csv" "${MAP_PGM}" \
        > "${OUT}/ofl.csv" 2> "${OUT}/ofl.log"
fi

echo
python3 "${HERE}/summarize.py" "ofl=${OUT}/ofl.csv" "bbs=${OUT}/bbs.csv"
echo
sed -n '/\[stage\]/,$p' "${OUT}/ofl.log" || true
echo "生成物: ${OUT}/{scans,ofl,bbs}.csv"
