#!/usr/bin/env bash
# Nav2 との閉ループ検証。
#
#   ./run_nav2.sh [out_dir] [duration_s]
#
# 位置推定だけを差し替えて (LOC)、同じ world・同じ地図・同じ目標列で走らせる。
# 航法スタックの設定は 3 条件で完全に同一なので、差は位置推定に帰属する。
#
# 環境変数:
#   LOC        ofl (既定) / amcl / gt / amcl_ofl / amcl_sup … map->odom を出すのは誰か
#              amcl_ofl は「OFL で初期姿勢を与え、以降は AMCL が追う」構成
#              (ofl_node は TF を出さず /initialpose を初回だけ出す)
#              amcl_sup はそれに**監視**を足した構成。OFL が AMCL の姿勢を毎スキャン
#              WFRAC で検証し、壊れていると判断したら撒き直させる (supervise_amcl)
#   DYNAMIC    1 で地図に無い動く障害物を置く
#   KIDNAP_AT  この秒数で瞬間移動させる
#   IMAGE      Gazebo Classic + gazebo_ros + nav2 を持つ docker イメージ
#   OFL_ARGS   ofl_node への追加パラメータ
#   OMP_NUM_THREADS  位置推定の相関に使うスレッド数 (既定 6)
#   RETRIES    Nav2 の起動に失敗したときの再試行回数 (既定 2)
#   CPU_LOG    1 で位置推定ノードの CPU 時間を 2 秒ごとに /out/cpu.log へ記録する
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "${HERE}/.." && pwd)"
OUT="${1:-${HERE}/out_nav2}"
DURATION="${2:-300}"
IMAGE="${IMAGE:-bac_gazebo_runtime:humble}"
LOC="${LOC:-ofl}"

mkdir -p "${OUT}"
DYNFLAG=""
[ "${DYNAMIC:-0}" = "1" ] && DYNFLAG="--dynamic"
# shellcheck disable=SC2086
python3 "${HERE}/make_env.py" "${OUT}/env" ${DYNFLAG}

run_once() {
docker run --rm --network none \
    -v "${PKG}:/ws/src/oriented_field_localization:ro" \
    -v "${OUT}:/out" \
    -e "DURATION=${DURATION}" -e "KIDNAP_AT=${KIDNAP_AT:-}" \
    -e "LOC=${LOC}" -e "DYNAMIC=${DYNAMIC:-0}" \
    -e "OFL_ARGS=${OFL_ARGS:-}" -e "OMP_NUM_THREADS=${OMP_NUM_THREADS:-6}" \
    -e "DUMP_COSTMAP=${DUMP_COSTMAP:-0}" -e "CPU_LOG=${CPU_LOG:-0}" \
    -e "OMP_WAIT_POLICY=${OMP_WAIT_POLICY:-}" \
    --entrypoint /bin/bash "${IMAGE}" -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /ws
colcon build --packages-select oriented_field_localization \
    --cmake-args -DCMAKE_BUILD_TYPE=Release > /out/build.log 2>&1
source install/setup.bash
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}
SIM=/ws/src/oriented_field_localization/sim
PARAMS=${SIM}/nav2_params.yaml

gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    /out/env/office.world > /out/gzserver.log 2>&1 &
sleep 8

URDF=${SIM}/models/robot.urdf
ros2 run robot_state_publisher robot_state_publisher "${URDF}" \
    --ros-args -p use_sim_time:=true > /out/rsp.log 2>&1 &
sleep 2
read -r SX SY SYAW < <(python3 -c "import json;s=json.load(open(\"/out/env/route.json\"))[\"start\"];print(s[0],s[1],s[2])")
ros2 run gazebo_ros spawn_entity.py -file "${URDF}" -entity ofl_bot \
    -x "${SX}" -y "${SY}" -z 0.10 -Y "${SYAW}" > /out/spawn.log 2>&1
sleep 3

# ---- 位置推定 (ここだけが条件で変わる) ----
LOC_NODES="map_server"
case "${LOC}" in
  ofl)
    # shellcheck disable=SC2086
    ros2 run oriented_field_localization ofl_node --ros-args \
        -p use_sim_time:=true -p map_yaml_path:=/out/env/office.yaml \
        -p auto_localize:=true -p tf_mode:=map_to_odom \
        -p max_range:=10.0 -p margin_pixels:=284 \
        ${OFL_ARGS} > /out/loc.log 2>&1 &
    ;;
  amcl)
    LOC_NODES="map_server, amcl"
    ros2 run nav2_amcl amcl --ros-args --params-file "${PARAMS}" \
        -r __node:=amcl -p use_sim_time:=true \
        -p initial_pose.x:="${SX}" -p initial_pose.y:="${SY}" \
        -p initial_pose.z:=0.0 -p initial_pose.yaw:="${SYAW}" > /out/loc.log 2>&1 &
    ;;
  gt)
    python3 ${SIM}/gt_tf_node.py --ros-args -p use_sim_time:=true > /out/loc.log 2>&1 &
    ;;
  amcl_ofl | amcl_sup)
    # AMCL には初期姿勢を与えない。OFL が最初に採択した姿勢を /initialpose へ
    # 流し、そこから AMCL が追う (publish_initialpose の実運用経路)。
    # amcl_sup ではさらに OFL が AMCL の姿勢を毎スキャン検証し、壊れていると
    # 判断したら撒き直させる。**2 つの違いは supervise_amcl だけ**である。
    LOC_NODES="map_server, amcl"
    SUPERVISE=false
    [ "${LOC}" = "amcl_sup" ] && SUPERVISE=true
    ros2 run nav2_amcl amcl --ros-args --params-file "${PARAMS}" \
        -r __node:=amcl -p use_sim_time:=true \
        -p set_initial_pose:=false > /out/loc.log 2>&1 &
    # shellcheck disable=SC2086
    ros2 run oriented_field_localization ofl_node --ros-args \
        -p use_sim_time:=true -p map_yaml_path:=/out/env/office.yaml \
        -p auto_localize:=true -p tf_mode:=none -p publish_initialpose:=true \
        -p supervise_amcl:=${SUPERVISE} \
        -r amcl_pose:=/amcl_pose \
        -p max_range:=10.0 -p margin_pixels:=284 \
        ${OFL_ARGS} > /out/ofl_init.log 2>&1 &
    ;;
  *) echo "unknown LOC=${LOC}" >&2; exit 2;;
esac

# ---- 位置推定の CPU 時間 (任意) ----
# 遅延 (1 スキャン何 ms) はログに出るが、**並列に動かせるか**を決めるのは
# CPU 時間のほうなので別に測る。/proc/PID/stat の utime+stime (10 ms 刻み)。
if [ "${CPU_LOG:-0}" = "1" ]; then
  ( while : ; do
      for pat in ofl_node amcl; do
        for pid in $(pgrep -x "${pat}" 2>/dev/null); do
          if read -r _ _ _ _ _ _ _ _ _ _ _ _ _ ut st _ < "/proc/${pid}/stat" 2>/dev/null; then
            echo "$(date +%s) ${pat} ${pid} ${ut} ${st}"
          fi
        done
      done
      sleep 2
    done > /out/cpu.log 2>/dev/null ) &
fi

# ---- Nav2 ----
ros2 run nav2_map_server map_server --ros-args --params-file "${PARAMS}" \
    -r __node:=map_server -p use_sim_time:=true \
    -p yaml_filename:=/out/env/office.yaml > /out/map_server.log 2>&1 &
ros2 run nav2_planner planner_server --ros-args --params-file "${PARAMS}" \
    -r __node:=planner_server -p use_sim_time:=true > /out/planner.log 2>&1 &
ros2 run nav2_controller controller_server --ros-args --params-file "${PARAMS}" \
    -r __node:=controller_server -p use_sim_time:=true > /out/controller.log 2>&1 &
ros2 run nav2_behaviors behavior_server --ros-args --params-file "${PARAMS}" \
    -r __node:=behavior_server -p use_sim_time:=true > /out/behavior.log 2>&1 &
ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file "${PARAMS}" \
    -r __node:=bt_navigator -p use_sim_time:=true > /out/bt.log 2>&1 &
sleep 5
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -r __node:=lifecycle_manager_localization -p use_sim_time:=true \
    -p autostart:=true -p node_names:="[${LOC_NODES}]" > /out/lm_loc.log 2>&1 &
sleep 3
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -r __node:=lifecycle_manager_navigation -p use_sim_time:=true \
    -p autostart:=true \
    -p node_names:="[controller_server, planner_server, behavior_server, bt_navigator]" \
    > /out/lm_nav.log 2>&1 &
sleep 8

# ---- 動的障害物 ----
DYNARG=""
if [ "${DYNAMIC}" = "1" ]; then
  python3 ${SIM}/obstacle_node.py --dynamic /out/env/dynamic.json \
      --ros-args -p use_sim_time:=true > /out/obstacles.log 2>&1 &
  DYNARG="--dynamic /out/env/dynamic.json"
  sleep 2
fi

# ---- 走行と記録 ----
KID=""
[ -n "${KIDNAP_AT}" ] && KID="--kidnap-at ${KIDNAP_AT}"
CM=""
[ "${DUMP_COSTMAP:-0}" = "1" ] && CM="--dump-costmap"
HARD=$(python3 -c "print(int(${DURATION} * 3 + 300))")
# shellcheck disable=SC2086
timeout -k 20 "${HARD}" python3 ${SIM}/nav2_drive_node.py \
    --route /out/env/route.json --map /out/env/office.yaml \
    --out /out/run.csv --duration "${DURATION}" ${DYNARG} ${KID} ${CM} \
    --ros-args -p use_sim_time:=true > /out/drive.log 2>&1 || true

for p in gzserver ofl_node amcl map_server planner_server controller_server \
         behavior_server bt_navigator lifecycle_manager robot_state_publisher \
         obstacle_node.py gt_tf_node.py; do
  pkill -f "${p}" 2>/dev/null || true
done
sleep 2
exit 0
'
}

# Nav2 の lifecycle 起動は稀に失敗する。configure 自体は通っているのに
# サービスの応答が Fast DDS で落ちることがあり
#   [WARN] [planner_server.rclcpp]: failed to send response to
#          /planner_server/change_state (timeout)
# lifecycle_manager がそこで待ち続けて bt_navigator まで到達しない。
# 25 走行に 1 回程度なので、空の run.csv を検出して回し直す。
RETRIES="${RETRIES:-2}"
attempt=0
while : ; do
  attempt=$((attempt + 1))
  rm -f "${OUT}/run.csv"      # 前回の残りを成功と誤判定しない
  run_once
  # ヘッダのみ = 1 行なら失敗
  if [ "$(wc -l < "${OUT}/run.csv" 2>/dev/null || echo 0)" -gt 1 ]; then
    break
  fi
  if [ "${attempt}" -gt "${RETRIES}" ]; then
    echo "run_nav2: ${attempt} 回試して起動できなかった (${OUT}/lm_nav.log を見ること)" >&2
    break
  fi
  echo "run_nav2: Nav2 が起動しなかった (${attempt} 回目)。やり直す" >&2
  sleep 5
done

echo
python3 "${HERE}/summarize_nav2.py" "${OUT}/run.csv" || true
exit 0
