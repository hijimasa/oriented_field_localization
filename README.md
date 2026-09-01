# Oriented Field Localization

English | [日本語](README.ja.md)

A ROS 2 package that localizes a 2D LiDAR scan against an occupancy grid by **FFT
correlation of an oriented field (wall mass + sensor-facing unit normals) in image space**,
recovering the 3-DoF map pose `(x, y, yaw)` **from a single scan with no initial guess**.

It searches the whole map on initialisation and re-initialisation (GLOBAL), then follows
the pose with a local search around a prior once converged (TRACK). There is no ICP
refinement, no multi-scan accumulation, and no internal odometry.

## Results (9 maps x 15 disturbance conditions x 40 poses = 5400 trials)

| Method | Success | Per scan [s] |
|---|---:|---:|
| **This package, GLOBAL** | **86.9%** | **0.010** (30x30 m map) / 0.025 (52x50 m) |
| **This package, TRACK** (prior error <= 3 m / 30 deg) | **96.6%** | **0.005** / 0.005 |
| BBS (Olson/Hess branch-and-bound correlative matching) | 62.5% | 0.013 / 0.037 |
| Radon sinogram method (sibling package, best configuration) | 85.7% | 0.060 / 0.084 |

In a continuous Gazebo drive (240 s, 121 m) it holds a **median error of 0.037 m with 99.8% within
0.5 m**, and recovers from a kidnap in **0.6 s** ([docs/en/simulation.md](docs/en/simulation.md)).
**Closed-loop with Nav2**, with unmapped moving obstacles, its total time above 0.5 m of error over
a 300 s drive was **0.1 -- 0.6 s in each of four runs** (AMCL under the same conditions went badly
wrong in 2 of 3 runs, for 22 s and 106 s;
[docs/en/nav2_closed_loop.md](docs/en/nav2_closed_loop.md)).

Success is "position error < 1.0 m and angle error < 15 deg". McNemar against BBS gives
p = 1.6e-278. **Inside its window, TRACK pulls in regardless of how large the prior error
is**, and it helps most on the self-similar corridor maps (50.7% -> 84.5%). **Its time does
not depend on the map size.** The per-map and per-condition breakdown, and the limits, are in
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
| subscribe | `odom` | `nav_msgs/msg/Odometry` | when `use_odometry: true` |
| publish | `~/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | accepted pose (every time) |
| publish | `/initialpose` | as above | only on the first accept out of GLOBAL (handoff to AMCL) |
| publish | `~/candidates` | `geometry_msgs/msg/PoseArray` | candidates, for RViz / diagnosis |
| service | `~/global_localization` | `std_srvs/srv/Empty` | drop tracking and restart from GLOBAL |
| TF | `map -> odom` or `map -> base_frame` | TF2 | selected by `tf_mode` |

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
| `enable_track` | `true` | false stays in GLOBAL forever |
| `track_search_m` | `3.0` | TRACK position search radius `[m]` |
| `track_angle_window_deg` | `30` | TRACK angle search window `[deg]` |
| `track_after_accepts` | `3` | consecutive accepts before switching to TRACK |
| `max_consecutive_rejects` | `5` | consecutive rejects before falling back to GLOBAL |
| `track_max_wfrac` | `0.35` | reject in TRACK when the fraction of points off the walls exceeds this. **This is what detects a wrong lock.** Use `0.45`--`0.50` where unmapped obstacles are expected |
| `use_odometry` | `true` | propagate the prior with `/odom` |
| `publish_initialpose` | `true` | publish `/initialpose` on the first lock (and on re-acquisition after losing track) |
| `initialpose_repeat` | `5` | how many times to republish `/initialpose`. **Not spent while there is no subscriber** |
| `tf_mode` | `none` | `none` / `map_to_odom` (REP-105) / `map_to_base` |

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

The ROS-independent unit test (19 checks) covers the parameter invariants, recovery of known
poses, the candidate pool's properties (score ordering, NMS separation, size cap), the sign
of WFRAC, and TRACK's pull-in and window boundary.

## Continuous-drive validation in Gazebo

Driving the robot inside Gazebo Classic, both **open loop** (control on ground truth, the localizer
a passive observer) and **closed loop** (Nav2 drives on the localizer's output alone).

```bash
./sim/run_sim.sh  out 240                        # open loop
KIDNAP_AT=120 ./sim/run_sim.sh out_kidnap 240
./sim/run_nav2.sh out_nav2 300                   # closed loop (Nav2)
LOC=amcl DYNAMIC=1 ./sim/run_nav2.sh out_amcl 300   # against AMCL with moving obstacles
```

A single wall definition generates both the world and the occupancy grid, so they cannot disagree.
Measurements and limits are in [docs/en/simulation.md](docs/en/simulation.md) (open loop) and
[docs/en/nav2_closed_loop.md](docs/en/nav2_closed_loop.md) (closed loop); the setup is described in
[sim/README.en.md](sim/README.en.md).

**This validation found and fixed two defects.**

1. **A jump gate cannot detect a wrong lock.** Once the track lands somewhere wrong, every
   subsequent prior comes from that wrong place, so nothing looks like a jump. Adding WFRAC — which
   does not consult the prior — turned "never recovers" into 0.6 s.
2. **The `/initialpose` handoff was lost to a startup race.** It was published once, with volatile
   QoS, at the first lock; if the subscriber (AMCL) activated later it was silently dropped. It is
   now republished, and **the repeat budget is not spent while there is no subscriber**.

What the closed loop showed:

- **What separates the methods is not accuracy but how long they stay wrong.** Instantaneous worst
  case is 8 -- 14 m for both. But total time above 0.5 m of error stayed under a second in all eight
  runs of this package, while AMCL went wrong for 22 s and 106 s in 2 of 3 dynamic runs and never
  came back in 2 of 2 kidnaps
- **A global localizer's jumps are not free.** The local costmap lives in the odom frame, so a jump
  invalidates every trajectory, aborts `follow_path`, and costs 2.5 s of standing still
- **The default `track_max_wfrac` of 0.35 is a value for a static map.** With unmapped obstacles the
  healthy WFRAC rises to nearly 0.50, and the default rejects 50 perfectly good observations per
  300 s. Use 0.45 -- 0.50 there: the error is unchanged, but needless GLOBAL restarts drop from
  15-18 to 3-9

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

Note that `radon_global_localization` is a **more built-out package**, with PLICP
refinement, internal scan-to-scan odometry, and multi-scan accumulation. This package
focuses on the GLOBAL and TRACK searches themselves.

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
├── sim/                             # continuous-drive validation in Gazebo
│                                     #   (open loop: run_sim.sh, closed loop: run_nav2.sh)
│   ├── make_env.py                  # world, map and route from one wall definition
│   ├── models/robot.urdf
│   ├── drive_node.py                # path following (ground truth) and error recording
│   └── run_sim.sh
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
    ├── simulation.md                # continuous-drive validation (open loop)
    ├── nav2_closed_loop.md          # closed loop with Nav2, and dynamic obstacles
    └── en/                          # English versions of the above
```

## License

[MIT License](LICENSE)
