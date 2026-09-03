#!/usr/bin/env bash
# Gazebo での連続走行検証。
#
#   ./run_sim.sh [out_dir] [duration_s]
#
# Gazebo Classic + gazebo_ros を持つイメージの中で、
#   world + 地図 (make_env.py が同じ壁定義から生成) -> Gazebo -> ロボット spawn
#   -> ofl_node -> 経路追従 + 誤差記録
# を回す。制御は真値で行い、位置推定は受け身の観測者として評価する。
#
# 環境変数:
#   IMAGE       使用する docker イメージ (Gazebo Classic + gazebo_ros が要る)
#   KIDNAP_AT   この秒数で瞬間移動させる (kidnap からの復帰を見る)
#   OFL_ARGS    ofl_node へ渡す追加パラメータ (例: "-p enable_track:=false")
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "${HERE}/.." && pwd)"
OUT="${1:-${HERE}/out}"
DURATION="${2:-180}"
IMAGE="${IMAGE:-oriented-field-localization-sim:humble}"

mkdir -p "${OUT}"
# docker run -v はスラッシュを含まない相対パスを bind mount ではなく
# **名前付きボリューム**として解釈する (コンテナの /out が空になり、
# route.json が無いという不可解な失敗になる)。絶対パスに正規化しておく。
OUT="$(cd "${OUT}" && pwd)"
python3 "${HERE}/make_env.py" "${OUT}/env"
rm -f "${OUT}/run.csv" "${OUT}/run_events.json"

docker run --rm --network none \
    -v "${PKG}:/ws/src/oriented_field_localization:ro" \
    -v "${OUT}:/out" \
    -e "DURATION=${DURATION}" -e "KIDNAP_AT=${KIDNAP_AT:-}" \
    -e "OFL_ARGS=${OFL_ARGS:-}" -e "OMP_NUM_THREADS=${OMP_NUM_THREADS:-8}" \
    --entrypoint /bin/bash "${IMAGE}" -lc '
set -eo pipefail
cleanup() {
  status=$?
  trap - EXIT INT TERM
  mapfile -t pids < <(jobs -pr)
  if [ "${#pids[@]}" -gt 0 ]; then
    kill "${pids[@]}" 2>/dev/null || true
    sleep 1
    kill -KILL "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
  exit "${status}"
}
trap cleanup EXIT
trap "exit 130" INT
trap "exit 143" TERM

source /opt/ros/humble/setup.bash
cd /ws
colcon build --packages-select oriented_field_localization \
    --cmake-args -DCMAKE_BUILD_TYPE=Release > /out/build.log 2>&1
source install/setup.bash
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}

# --- Gazebo (ヘッドレス) ---
gzserver --verbose \
    -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    /out/env/office.world > /out/gzserver.log 2>&1 &
sleep 8

# --- ロボットの spawn と TF ---
URDF=/ws/src/oriented_field_localization/sim/models/robot.urdf
ros2 run robot_state_publisher robot_state_publisher "${URDF}" \
    --ros-args -p use_sim_time:=true > /out/rsp.log 2>&1 &
sleep 2
START=$(python3 -c "import json;s=json.load(open(\"/out/env/route.json\"))[\"start\"];print(f\"{s[0]} {s[1]} {s[2]}\")")
set -- $START
# 車輪半径 0.08 + 取り付け -0.02 なので base_link の高さは 0.10 で接地する
ros2 run gazebo_ros spawn_entity.py -file "${URDF}" -entity ofl_bot \
    -x "$1" -y "$2" -z 0.10 -Y "$3" > /out/spawn.log 2>&1
sleep 3

# --- 位置推定 ---
# shellcheck disable=SC2086
ros2 run oriented_field_localization ofl_node --ros-args \
    -p use_sim_time:=true \
    -p map_yaml_path:=/out/env/office.yaml \
    -p auto_localize:=true \
    -p tf_mode:=map_to_odom \
    -p max_range:=10.0 \
    -p margin_pixels:=284 \
    ${OFL_ARGS} > /out/ofl.log 2>&1 &
sleep 5

# --- 走行と記録 ---
KID=""
[ -n "${KIDNAP_AT}" ] && KID="--kidnap-at ${KIDNAP_AT}"
# 実時間の倍率が 1 を下回るので、sim 時間 DURATION には実時間でその数倍かかる。
# 想定の 5 倍 + 余裕で頭打ちにして、何かが固まっても必ず後片付けへ進む。
HARD=$(python3 -c "print(int(${DURATION} * 5 + 120))")
# shellcheck disable=SC2086
timeout -k 20 "${HARD}" python3 /ws/src/oriented_field_localization/sim/drive_node.py \
    --route /out/env/route.json --out /out/run.csv \
    --duration "${DURATION}" ${KID} \
    --ros-args -p use_sim_time:=true > /out/drive.log 2>&1
python3 -c "import json,os;p=\"/out/run_events.json\";d={\"events\":[{\"t\":float(os.environ[\"DURATION\"]),\"ev\":\"run_complete\"}]};open(p,\"w\").write(json.dumps(d,indent=2))"
'

python3 - "${OUT}/run.csv" "${DURATION}" <<'PY'
import csv
import json
import math
import os
import sys

path, duration_text = sys.argv[1:]
duration = float(duration_text)
if duration <= 0:
    raise SystemExit('duration_s must be positive')
with open(path, newline='') as stream:
    rows = list(csv.DictReader(stream))
required = {'t', 'gt_x', 'gt_y', 'est_x', 'est_y', 'pos_err', 'yaw_err'}
if not rows or not required.issubset(rows[0]):
    raise SystemExit(f'run_sim: invalid or empty CSV: {path}')
minimum_rows = max(2, int(duration * 5))
if len(rows) < minimum_rows:
    raise SystemExit(f'run_sim: only {len(rows)} rows; expected at least {minimum_rows}')
try:
    final_t = float(rows[-1]['t'])
except (TypeError, ValueError):
    raise SystemExit('run_sim: final CSV timestamp is invalid')
if not math.isfinite(final_t) or final_t < duration - 0.5:
    raise SystemExit(f'run_sim: stopped at {final_t:.3f}s before {duration:.3f}s')
if not any(math.isfinite(float(row['pos_err'])) for row in rows
           if row['pos_err'] not in ('', 'nan')):
    raise SystemExit('run_sim: CSV contains no finite localization estimate')
events_path = os.path.splitext(path)[0] + '_events.json'
with open(events_path) as stream:
    events = json.load(stream).get('events', [])
if not any(event.get('ev') == 'run_complete' for event in events):
    raise SystemExit('run_sim: completion report has no run_complete event')
print(f'[validate] {len(rows)} rows through {final_t:.3f}s; run_complete event present')
PY

echo
python3 "${HERE}/summarize_run.py" "${OUT}/run.csv"
