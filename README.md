# Oriented Field Localization

English | [日本語](README.ja.md)

A ROS 2 package that localizes a 2D LiDAR scan against an occupancy grid by **FFT
correlation of an oriented field (wall mass + sensor-facing unit normals) in image space**,
recovering the 3-DoF map pose `(x, y, yaw)` **from a single scan with no initial guess**.

**It does global localization only.** The pose is published on `/initialpose`, and tracking
is left to AMCL or similar (the same division of labour as CBGL). There is no odometry, no
state machine, and no ICP refinement.

## Results (9 maps x 15 disturbance conditions x 40 poses = 5400 trials)

| Method | Success | Per scan [s] |
|---|---:|---:|
| **This package** | **86.9%** | **0.010** (30x30 m map) / 0.025 (52x50 m) |
| BBS (Olson/Hess branch-and-bound correlative matching) | 62.5% | 0.013 / 0.037 |
| Radon sinogram method (sibling package, best configuration) | 85.7% | 0.060 / 0.084 |

Success is "position error < 1.0 m and angle error < 15 deg". McNemar against BBS gives
p = 1.6e-278. The per-map and per-condition breakdown, and the limits, are in
**[docs/benchmark.md](docs/benchmark.md)**.

**BBS has a completeness guarantee and this method does not.** The gap is therefore not
about exhaustiveness but about score discrimination: the BBS likelihood field only asks
whether a point lies near a wall, while this method also requires the wall **orientation**
to agree.

## Method

Both the template (scan) and the map become a three-channel field
`f = (mass, lam*mass*nx, lam*mass*ny)`, and a pose is scored as

```text
s(p, alpha) = < f_map , R_alpha f_scan >_p / sqrt(E_scan)
```

**Keeping the denominator template-side only is the key point.** Dividing also by the
map-side energy at the candidate position rebuilds the reference signal per candidate,
which destroys comparability between candidates and costs about 12 pt of top-1 accuracy.

The search is a coarse-to-fine image pyramid. The coarse stage gets **the score at every
position** from one inverse DFT per angle and keeps the top NMS'd peaks per angle; the fine
stage refines each candidate over a narrow angle and position window by direct correlation
over a sparse point list. See [docs/design.md](docs/design.md).

## Dependencies

ROS 2 / `ament_cmake`, OpenCV, OpenMP (optional). PCL is not needed.

## Build

```bash
colcon build --packages-select oriented_field_localization
source install/setup.bash
```

To verify without a local ROS installation, use the minimal Docker environment:

```bash
./docker/run_ci.sh
```

## Running

```bash
ros2 launch oriented_field_localization global_localization.launch.py \
  map_yaml_path:=/absolute/path/to/map.yaml
```

Leave `map_yaml_path` empty to wait for a transient-local / reliable `/map`. Trigger a
search with the service:

```bash
ros2 service call \
  /oriented_field_localization/global_localization std_srvs/srv/Empty '{}'
```

Set `auto_localize: true` to run on every incoming scan.

## ROS interface

| Kind | Name | Type | Purpose |
|---|---|---|---|
| subscribe | `scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR scan |
| subscribe | `map` | `nav_msgs/msg/OccupancyGrid` | when `map_yaml_path` is unset |
| publish | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | accepted pose |
| publish | `~/candidates` | `geometry_msgs/msg/PoseArray` | candidates, for RViz / diagnosis |
| service | `~/global_localization` | `std_srvs/srv/Empty` | start a search |
| TF | `map -> base_frame` | TF2 | only when `publish_tf: true` |

## Key parameters

[config/params.yaml](config/params.yaml) is authoritative.

| Parameter | Default | Meaning |
|---|---:|---|
| `match_resolution` | `0.05` | matching grid `[m/px]` |
| `max_range` | `10.0` | maximum range used for the template `[m]` |
| `min_range` | `0.0` | close-range gate `[m]`; 0 disables |
| `margin_pixels` | `284` | zero padding around the map `[px]` |
| `pyramid_levels` | `3` | coarse-to-fine levels |
| `coarse_angle_step` | `6` | coarse angle step `[deg]` |
| `peaks_per_angle` | `8` | peaks kept per angle at the coarse level |
| `candidate_pool_size` | `15` | candidates retained |
| `wfrac_margin` | `1.05` | re-select by WFRAC only when scores are this close; 0 disables |
| `global_min_margin` | `1.0` | do not publish if top1/top2 is below this |

Important invariants:

```text
margin_pixels >= ceil(max_range / match_resolution)                    # required
margin_pixels >= ceil(sqrt(2) * max_range / match_resolution) + 1      # recommended
```

Below the first, the template is clipped at the map border and poses near the edge become
structurally unreachable (the node raises it and warns). Meeting the second makes the
correlation padding zero and the coarse DFT minimal; missing it can double the coarse cost.

### Defaults chosen by measurement

- `pyramid_levels: 3` — two levels are 2x slower at the same accuracy
- `coarse_angle_step: 6` — 3 deg buys 0.4 pt for 1.7x the time; 9 deg and above lose
  accuracy without saving time
- `wfrac_margin: 1.05` — calibrated over nine maps; using 1.3 (the sinogram method's value)
  costs about 1 pt
- `normal_weight: 1.0` — dropping to 0 (wall mass only) costs 1.5 pt
- Size the thread count by **physical** cores: going to the logical core count makes the
  2-D FFTs contend over SMT and runs 1.4x slower

## Tests

```bash
colcon test --packages-select oriented_field_localization
colcon test-result --verbose
```

The ROS-independent unit test checks the parameter invariants, recovery of known poses,
the candidate pool's properties (score ordering, NMS separation, size cap), and the sign of
WFRAC.

## Evaluation harness

[eval/](eval/) holds a minimal harness that compares against the BBS baseline on identical
scans. No third-party dataset is bundled, so it generates a synthetic map.

```bash
cd eval && ./run_compare.sh out 40
```

`make_scans` uses the same random sequence for poses and disturbances as
`disturb_eval2 --dump` in the sibling `radon_global_localization` package, so the same map
and trial count give **byte-identical scans** and the two packages' numbers line up.

## Relationship to the sibling package

`radon_global_localization` matches the same representation in Radon (sinogram) space. This
package started as its **image-space control** and was split out after a nine-map comparison
put it ahead on both success rate and speed. The history, the mechanism, and the hypotheses
that were retracted are recorded in
`radon_global_localization/docs/image_space_control.md`.

Note that `radon_global_localization` is a **complete package for continuous operation**,
with tracking, odometry, PLICP refinement, and a state machine. This package only does the
global search, so they do not simply substitute for one another.

## Layout

```text
oriented_field_localization/
├── include/oriented_field_localization/oriented_field_matcher.hpp
├── src/
│   ├── oriented_field_matcher.cpp   # ROS-independent matching library
│   └── ofl_node.cpp                 # ROS 2 node (global search only)
├── launch/global_localization.launch.py
├── config/params.yaml
├── tests/test_matcher.cpp
├── eval/                            # comparison harness against BBS
│   ├── make_scans.cpp               # pose sampling + disturbed scan generation
│   ├── ofl_eval.cpp                 # evaluator for this method
│   ├── bbs_eval.cpp                 # BBS baseline
│   ├── make_synthetic_map.py
│   ├── summarize.py
│   └── run_compare.sh
├── docker/                          # minimal ROS 2 environment for build and test
└── docs/
    ├── design.md                    # design and known limits
    ├── benchmark.md                 # comparison against BBS / Radon
    └── en/                          # English versions of the above
```

## License

[MIT License](LICENSE)
