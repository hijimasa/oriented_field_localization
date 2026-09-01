# Oriented Field Localization

[English](README.md) | 日本語

2D LiDAR スキャンと占有格子地図を、**画像空間で向き付き場 (壁質量 + 観測側を向く
単位法線) の FFT 相関**として照合し、地図座標の 3-DoF 姿勢 `(x, y, yaw)` を
**1 スキャンから初期位置なしで**推定する ROS 2 パッケージです。

**大域位置推定 (GLOBAL) だけを行います。**求めた姿勢を `/initialpose` へ流し、以降の
追跡は AMCL などに任せる分担を想定しています (CBGL と同じ位置づけ)。オドメトリ・
状態機械・ICP 精密化は持ちません。

## 成績 (9 地図 x 15 外乱条件 x 40 姿勢 = 5400 試行)

| 手法 | 成功率 | 1 スキャンあたり [s] |
|---|---:|---:|
| **本手法** | **86.9%** | **0.010** (30x30 m 地図) / 0.025 (52x50 m) |
| BBS (Olson/Hess 型 分枝限定相関) | 62.5% | 0.013 / 0.037 |
| Radon サイノグラム法 (姉妹パッケージの最良構成) | 85.7% | 0.060 / 0.084 |

成功判定は「位置誤差 < 1.0 m かつ 角度誤差 < 15 度」。McNemar で BBS 比
p = 1.6e-278。条件・地図別の内訳と限界は
**[docs/benchmark.md](docs/benchmark.md)** にあります。

**BBS には探索の完全性の保証があり、本手法にはありません。**それでも差が付くのは
網羅性ではなくスコアの識別力の差です。BBS の尤度場は「点が壁の近くにあるか」しか
見ませんが、本手法は壁の**向き**まで一致を要求します。

## 手法

テンプレート (スキャン) と地図を 3 チャネルの場
`f = (mass, lam*mass*nx, lam*mass*ny)` に変換し、姿勢のスコアを

```text
s(p, alpha) = < f_map , R_alpha f_scan >_p / sqrt(E_scan)
```

とします。**分母をテンプレート側だけにする**のが要点で、候補位置ごとの地図側
エネルギーでも割ると候補間の比較可能性が壊れて順位付けが落ちます (実測 -12 pt)。

探索は画像ピラミッドの粗密探索です。粗段は角度ごとに 1 回の逆 DFT で**全位置の
スコア場**を一度に得て、角度ごとに上位ピークを NMS 付きで保持します。細段は候補ごとに
狭い角度・位置窓を疎な点列の直接相関で精密化します。詳細は
[docs/design.md](docs/design.md)。

## 依存関係

ROS 2 / `ament_cmake`、OpenCV、OpenMP (任意)。PCL は不要です。

## ビルド

```bash
colcon build --packages-select oriented_field_localization
source install/setup.bash
```

ROS を用意せずに検証する場合は Docker の最小環境で回せます。

```bash
./docker/run_ci.sh
```

## 起動

```bash
ros2 launch oriented_field_localization global_localization.launch.py \
  map_yaml_path:=/absolute/path/to/map.yaml
```

`map_yaml_path` を空にすると transient-local / reliable QoS の `/map` を待ちます。
推定はサービスで開始します。

```bash
ros2 service call \
  /oriented_field_localization/global_localization std_srvs/srv/Empty '{}'
```

スキャン受信ごとに自動実行する場合は `auto_localize: true` にします。

## ROS インターフェース

| 種別 | 名前 | 型 | 用途 |
|---|---|---|---|
| subscribe | `scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR スキャン |
| subscribe | `map` | `nav_msgs/msg/OccupancyGrid` | `map_yaml_path` 未指定時 |
| publish | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 採択した姿勢 |
| publish | `~/candidates` | `geometry_msgs/msg/PoseArray` | 候補の可視化・診断 |
| service | `~/global_localization` | `std_srvs/srv/Empty` | 探索の開始 |
| TF | `map -> base_frame` | TF2 | `publish_tf: true` のときだけ |

## 主要パラメータ

既定値は [config/params.yaml](config/params.yaml) が正です。

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `match_resolution` | `0.05` | マッチング格子 `[m/px]` |
| `max_range` | `10.0` | テンプレートに使う最大レンジ `[m]` |
| `min_range` | `0.0` | 至近レンジゲート `[m]`。0 で無効 |
| `margin_pixels` | `284` | 地図外周のゼロパディング `[px]` |
| `pyramid_levels` | `3` | 粗密探索の段数 |
| `coarse_angle_step` | `6` | 粗段の角度刻み `[deg]` |
| `peaks_per_angle` | `8` | 粗段で角度ごとに保持するピーク数 |
| `candidate_pool_size` | `15` | 残す候補数 |
| `wfrac_margin` | `1.05` | 拮抗時に WFRAC で選び直す比率。0 で無効 |
| `global_min_margin` | `1.0` | top1/top2 がこれ未満なら publish しない |

重要な不変条件:

```text
margin_pixels >= ceil(max_range / match_resolution)                    # 必須
margin_pixels >= ceil(sqrt(2) * max_range / match_resolution) + 1      # 推奨
```

1 つめを下回るとテンプレートが地図の縁で切れ、縁付近の姿勢を構造的に探索できません
(ノードが自動で引き上げて警告します)。2 つめを満たすと相関の外挿が 0 になり、
粗段の DFT が最小になります (満たさないと粗段のコストが最大 2 倍になります)。

### 実測で決めた既定値

- `pyramid_levels: 3` — 2 段は 2 倍遅く精度は同等
- `coarse_angle_step: 6` — 3 度は +0.4 pt で 1.7 倍遅い。9 度以上は時間が減らず精度だけ落ちる
- `wfrac_margin: 1.05` — 9 地図で較正。1.3 (サイノグラム法向けの値) を当てると -1 pt
- `normal_weight: 1.0` — 0 (壁質量のみ) にすると -1.5 pt
- スレッド数は**物理コア数**に合わせること。論理コア数まで上げると 2 次元 FFT が
  SMT で取り合いになり 1.4 倍遅くなります

## テスト

```bash
colcon test --packages-select oriented_field_localization
colcon test-result --verbose
```

ROS 非依存の単体テストは、パラメータの不変条件、既知姿勢の復元、候補プールの性質
(スコア降順・NMS 分離・上限)、WFRAC の符号を検査します。

## 評価ハーネス

BBS ベースラインと同一スキャンで比較する最小セットを [eval/](eval/) に置いています。
第三者データセットを同梱しないため、合成地図を生成して回す構成です。

```bash
cd eval && ./run_compare.sh out 40
```

`make_scans` の姿勢・外乱の乱数系列は姉妹パッケージ `radon_global_localization` の
`disturb_eval2 --dump` と同一なので、同じ地図・同じ試行数なら**バイト同一のスキャン**が
得られ、両パッケージの数値を直接並べられます。

## 姉妹パッケージとの関係

`radon_global_localization` は同じ表現をラドン変換 (サイノグラム) 空間で照合します。
本パッケージはその**画像空間対照**として始まり、9 地図の比較で成功率・速度とも上回った
ため独立させたものです。経緯・機構の分析・撤回した仮説は
`radon_global_localization/docs/image_space_control.md` に記録されています。

なお `radon_global_localization` は TRACK (追跡)、オドメトリ、PLICP 精密化、状態機械を
持つ**連続運用向けの完成したパッケージ**です。本パッケージは大域探索のみなので、
用途が重なるわけではありません。

## 構成

```text
oriented_field_localization/
├── include/oriented_field_localization/oriented_field_matcher.hpp
├── src/
│   ├── oriented_field_matcher.cpp   # ROS 非依存のマッチングライブラリ
│   └── ofl_node.cpp                 # ROS 2 ノード (大域探索のみ)
├── launch/global_localization.launch.py
├── config/params.yaml
├── tests/test_matcher.cpp
├── eval/                            # BBS との比較ハーネス
│   ├── make_scans.cpp               # 姿勢サンプリング + 外乱スキャン生成
│   ├── ofl_eval.cpp                 # 本手法の評価器
│   ├── bbs_eval.cpp                 # BBS ベースライン
│   ├── make_synthetic_map.py
│   ├── summarize.py
│   └── run_compare.sh
├── docker/                          # ビルド・テスト用の最小 ROS 2 環境
└── docs/
    ├── design.md                    # 設計と既知の制約
    ├── benchmark.md                 # BBS / Radon との比較
    └── en/                          # 上記の英語版
```

## ライセンス

[MIT License](LICENSE)
