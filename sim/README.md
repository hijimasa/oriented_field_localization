# Gazebo での連続走行検証

[English](README.en.md) | 日本語

オフラインのハーネス (`../eval/`) は 1 スキャンずつの単発評価しかしない。ここでは
Gazebo Classic の中でロボットを走らせ、オフラインでは測れないものを見る。

**開ループ** (`run_sim.sh`, 制御は真値) — 位置推定を受け身の観測者として測る

- オドメトリのドリフトが溜まっていく状況での追従
- kidnap (瞬間移動) からの復帰
- TRACK が実際に何を稼いでいるか (毎スキャン GLOBAL との対照)

**閉ループ** (`run_nav2.sh`, Nav2 が位置推定の出力で走る) — ずれが航法に何をするかを測る

- 推定の跳びが制御・costmap・経路計画に何を起こすか
- 地図に無い動く障害物が入ったときの劣化 (AMCL との対照)
- kidnap から航法スタックごと復帰できるか

測定結果は [../docs/simulation.md](../docs/simulation.md) (開ループ) と
[../docs/nav2_closed_loop.md](../docs/nav2_closed_loop.md) (閉ループ)。

## 構成

```text
sim/
├── make_env.py         壁の定義から world SDF・占有格子・経路を生成する
│                       (--dynamic で地図に無い動く障害物を world だけに置く)
├── models/robot.urdf   差動二輪 + 2D LiDAR + 真値プラグイン
├── obstacle_node.py    動的障害物を決められた軌跡で動かす
├── gt_tf_node.py       真値から map -> odom を出す「完全な位置推定」
│
├── drive_node.py       [開ループ] 経路追従 (真値ベース) と誤差記録
├── summarize_run.py    [開ループ] run.csv の要約
├── run_sim.sh          [開ループ] 上を Docker の中で順に回す
│
├── nav2_params.yaml    [閉ループ] Nav2 の設定 (全条件で同一)
├── nav2_drive_node.py  [閉ループ] 目標の送信と、航法込みの記録
├── summarize_nav2.py   [閉ループ] 1 走行の要約
├── compare_nav2.py     [閉ループ] 複数の走行を 1 枚の表に
├── costmap_phantoms.py [閉ループ] costmap に焼き付いた幻の障害物を数える
└── run_nav2.sh         [閉ループ] 上を Docker の中で順に回す
```

### 開ループと閉ループ

`run_sim.sh` は**位置推定を受け身の観測者として**測る (制御は真値で回す)。
`run_nav2.sh` は **Nav2 が位置推定の出力だけで走る**ので、推定がずれれば経路も
costmap も追従もずれる。両方が要る理由は、前者でないと位置推定そのものを切り離して
測れず、後者でないと「ずれが航法に何をするか」が測れないからである。

### 壁の定義を単一の出所にする

`make_env.py` は壁の線分集合 1 つから **world SDF と占有格子の両方**を生成する。
world と地図を別々に用意すると、両者のずれが位置推定の誤差に化けて評価が意味を
失うためである。経路も同じ地図の上でクリアランス 0.9 m 以上のセルだけを通るように
BFS で計画するので、「地図では通れるが Gazebo では壁」という食い違いが起きない。

環境は 24 x 18 m のオフィス。外周・3 つの部屋・斜めの間仕切り・柱を持ち、
**点対称にならないように非対称なクラッタを置いてある** (対称な環境では 180 度
回した姿勢が同じ観測を与えるので、位置推定の評価にならない)。

### 制御に真値を使う理由

`drive_node.py` の経路追従は **`/ground_truth` を使う**。制御器の出来は評価対象では
ないし、位置推定の出力で走らせると一度外した瞬間に壁へ突っ込んで以降の評価が
成立しなくなる。位置推定は `/scan` と `/odom` だけを見て、走行には一切影響しない
受け身の観測者として評価する。

### オドメトリ

`libgazebo_ros_diff_drive` を `odometry_source=0` (車輪エンコーダ) で使う。
スリップで実際にドリフトするので、TRACK の事前姿勢の質が現実的になる。
記録される `dr_*` 列は「最初の真値を原点に置いてオドメトリの相対変位だけを積分した
姿勢」で、**位置推定が何を直しているかの基準**になる。

### LiDAR の取り付け位置

LiDAR は `base_link` と xy が一致する位置に置いてある。位置推定が返すのは
スキャン原点の姿勢なので、センサをずらすと真値 (base_link) との間に系統的な
オフセットが乗って評価が濁る。

## 実行

### 開ループ (位置推定だけを測る)

```bash
./sim/run_sim.sh out 240          # 240 秒の連続走行
KIDNAP_AT=120 ./sim/run_sim.sh out_kidnap 240      # 120 秒で瞬間移動
OFL_ARGS="-p enable_track:=false" ./sim/run_sim.sh out_notrack 240   # TRACK 無し
```

### 閉ループ (Nav2 が位置推定の出力で走る)

```bash
./sim/run_nav2.sh out_nav2 300                     # OFL, 静的
LOC=amcl ./sim/run_nav2.sh out_amcl 300            # AMCL に差し替え
LOC=gt   ./sim/run_nav2.sh out_gt 300              # 真値 (航法スタックの上限)
DYNAMIC=1 ./sim/run_nav2.sh out_dyn 300            # 地図に無い動く障害物
KIDNAP_AT=150 ./sim/run_nav2.sh out_kid 300        # 閉ループでの kidnap
DUMP_COSTMAP=1 ./sim/run_nav2.sh out_cm 300        # 最後の大域 costmap も保存
LOC=amcl_ofl ./sim/run_nav2.sh out_seed 300        # OFL で初期姿勢を与えて AMCL が追う
```

複数の走行を並べるには

```bash
python3 sim/compare_nav2.py OFL=out_nav2 AMCL=out_amcl 真値=out_gt
```

**位置推定 (`LOC`) 以外の設定は全条件で同一である。**変えると「航法の差」と
「位置推定の差」が混ざって比較にならない。

### 動的障害物

`make_env.py --dynamic` は 40 cm 角の歩行者 3 体と 70 x 50 cm の台車 1 台を
**world にだけ**置く (占有格子には焼かない)。`obstacle_node.py` が
`/gazebo/set_entity_state` で往復させる。位置は **sim 時刻の関数として陽に決める**
ので、位置推定の条件を変えても軌跡は同一である (速度指令で押すと物理の揺らぎで
実行ごとに軌跡が変わり、障害物の違いが比較に混ざる)。リンクは `kinematic` なので、
teleport しても物理が破綻せず、衝突はする。

Gazebo Classic と `gazebo_ros` を持つイメージが要る (`IMAGE` で指定、既定は
`bac_gazebo_runtime:humble`)。ネットワークは不要 (`--network none` で回る)。

出力は `out/run.csv` (20 Hz の時系列) と各プロセスのログ。閉ループでは
`run_events.json` (目標の送受と成否) も出る。要約は `summarize_run.py` /
`summarize_nav2.py`。

## 注意

- **実時間倍率を確認すること。**ODE 500 iter/s と 360 本のレイセンサで、1 本ずつ
  逐次に回すぶんには 1.00x で走る (閉ループの全条件で 1.00x だった)。複数を同時に
  回すと下がる。`run_nav2.sh` は CSV に実時間を記録するので `summarize_nav2.py` が
  倍率を出す。**「1 スキャンあたり何 ms」を実機の速度として読むときは倍率を見ること。**
- `real_time_update_rate=0` (できるだけ速く) にすると実時間の数百倍で走り、
  位置推定も制御も追従できない。`make_env.py` は 500 iter/s に固定してある。
- **動的障害物は押しても動かない** (`kinematic`)。人や台車は本来押されるので、
  詰まったときの挙動は現実より厳しい側に出る。
- **DWB に後退を許していない** (`min_vel_x: 0.0`)。角に鼻先を突っ込むと回頭でしか
  抜けられない。走行距離や「前進していた時間」は航法側の事情を多く含むので、
  位置推定の比較として読まないこと。
