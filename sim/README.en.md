# Continuous-drive validation in Gazebo

English | [日本語](README.md)

The offline harness (`../eval/`) only evaluates one scan at a time. Here the robot drives inside
Gazebo Classic, which shows what offline cannot.

**Open loop** (`run_sim.sh`, control on ground truth) — the localizer as a passive observer

- following while odometry drift accumulates
- recovery from a kidnap (teleport)
- what TRACK actually buys (against running GLOBAL on every scan)

**Closed loop** (`run_nav2.sh`, Nav2 drives on the localizer's output) — what the error does to navigation

- what a localization jump does to control, the costmaps and planning
- degradation with unmapped moving obstacles (against AMCL)
- whether the whole navigation stack recovers from a kidnap

Results are in [../docs/simulation.md](../docs/simulation.md) (open loop) and
[../docs/nav2_closed_loop.md](../docs/nav2_closed_loop.md) (closed loop).

## Layout

```text
sim/
├── make_env.py         world SDF, occupancy grid and route from one wall definition
│                       (--dynamic puts unmapped moving obstacles in the world only)
├── models/robot.urdf   differential drive + 2D LiDAR + ground-truth plugin
├── obstacle_node.py    drives the dynamic obstacles along fixed trajectories
├── gt_tf_node.py       map -> odom from ground truth: a "perfect localizer"
│
├── drive_node.py       [open loop] path following (on ground truth) and error recording
├── summarize_run.py    [open loop] summary of run.csv
├── run_sim.sh          [open loop] runs the above inside Docker
│
├── nav2_params.yaml    [closed loop] Nav2 configuration (identical in every condition)
├── nav2_drive_node.py  [closed loop] goal dispatch and recording, navigation included
├── summarize_nav2.py   [closed loop] summary of one run
├── compare_nav2.py     [closed loop] several runs in one table
├── costmap_phantoms.py [closed loop] counts phantom obstacles burned into the costmap
└── run_nav2.sh         [closed loop] runs the above inside Docker
```

### Open loop and closed loop

`run_sim.sh` measures the localizer as a **passive observer** (control runs on ground truth).
`run_nav2.sh` has **Nav2 drive on the localizer's output alone**, so a bad estimate bends the
path, the costmaps and the tracking with it. Both are needed: without the first you cannot
measure the localizer in isolation, and without the second you cannot measure what its error
does to navigation.

### One source of truth for the walls

`make_env.py` generates **both the world SDF and the occupancy grid** from a single set of wall
segments. Authoring the world and the map separately would let them disagree, and that disagreement
would show up as localization error, making the measurement meaningless. The route is planned on the
same map through cells with at least 0.9 m clearance, so "passable on the map but a wall in Gazebo"
cannot happen.

The environment is a 24 x 18 m office with outer walls, three rooms, a diagonal partition and
pillars, and **deliberately asymmetric clutter** (in a symmetric environment a 180-degree rotated
pose gives the same observation, which makes localization untestable).

### Why the controller uses ground truth

`drive_node.py` follows the path using `/ground_truth`. The controller is not under test, and
driving on the localizer's output would put the robot into a wall the moment it slipped, ending the
measurement. The localizer sees only `/scan` and `/odom` and never affects the motion.

### Odometry

`libgazebo_ros_diff_drive` with `odometry_source=0` (wheel encoders), so it genuinely drifts with
slip and the TRACK prior has realistic quality. The recorded `dr_*` columns are "the pose obtained by
integrating only the odometry from the initial ground-truth pose" — the baseline for **what the
localizer is correcting**.

### LiDAR placement

The LiDAR sits where its xy matches `base_link`. The localizer returns the pose of the scan origin,
so offsetting the sensor would add a systematic bias against ground truth (base_link).

## Running

### Open loop (the localizer alone)

```bash
./sim/run_sim.sh out 240          # 240 s continuous drive
KIDNAP_AT=120 ./sim/run_sim.sh out_kidnap 240      # teleport at 120 s
OFL_ARGS="-p enable_track:=false" ./sim/run_sim.sh out_notrack 240   # no TRACK
```

### Closed loop (Nav2 drives on the localizer's output)

```bash
./sim/run_nav2.sh out_nav2 300                     # OFL, static
LOC=amcl ./sim/run_nav2.sh out_amcl 300            # swap in AMCL
LOC=gt   ./sim/run_nav2.sh out_gt 300              # ground truth (ceiling of the stack)
DYNAMIC=1 ./sim/run_nav2.sh out_dyn 300            # unmapped moving obstacles
KIDNAP_AT=150 ./sim/run_nav2.sh out_kid 300        # kidnap in the closed loop
DUMP_COSTMAP=1 ./sim/run_nav2.sh out_cm 300        # also save the final global costmap
LOC=amcl_ofl ./sim/run_nav2.sh out_seed 300        # OFL seeds the initial pose, AMCL tracks
```

To put several runs side by side:

```bash
python3 sim/compare_nav2.py OFL=out_nav2 AMCL=out_amcl GT=out_gt
```

**Nothing but the localizer (`LOC`) changes between conditions.** Changing anything else mixes
"difference in navigation" into "difference in localization".

### Dynamic obstacles

`make_env.py --dynamic` puts three 40 cm walkers and one 70 x 50 cm cart **into the world only**
(they are never rasterized into the occupancy grid). `obstacle_node.py` moves them back and forth
through `/gazebo/set_entity_state`. Their poses are an **explicit function of simulation time**, so
the trajectories are identical no matter which localizer is under test (driving them with velocity
commands would let physics jitter change the trajectory run to run, mixing the obstacles into the
comparison). The links are `kinematic`, so teleporting them does not break physics and they still
collide.

Needs an image with Gazebo Classic and `gazebo_ros` (set `IMAGE`; default
`oriented-field-localization-sim:humble`). No network required (it runs with `--network none`).

Outputs are `out/run.csv` (a 20 Hz time series) plus each process's log; closed-loop runs also write
`run_events.json` (goals sent and their outcome). Summarize with `summarize_run.py` /
`summarize_nav2.py`.

## Notes

- **Check the real-time factor.** With ODE at 500 iter/s and a 360-ray sensor, running one at a time
  gives 1.00x (every closed-loop condition did). Running several at once drops it. `run_nav2.sh`
  records wall-clock time in the CSV and `summarize_nav2.py` reports the factor. **Check it before
  reading "milliseconds per scan" as deployment headroom.**
- Setting `real_time_update_rate=0` ("as fast as possible") runs hundreds of times faster than real
  time and nothing can keep up; `make_env.py` pins it to 500 iter/s.
- **The dynamic obstacles do not move when pushed** (`kinematic`). Real people and carts do, so the
  behaviour when wedged is harsher here than in reality.
- **DWB is not allowed to reverse** (`min_vel_x: 0.0`). Nose into a corner and it can only rotate
  out. Distance driven and "fraction of time moving" carry a lot of navigation-side accident — do
  not read them as a localization comparison.
