# Continuous-drive validation in Gazebo

English | [日本語](README.md)

The offline harness (`../eval/`) only evaluates one scan at a time. Here the robot drives inside
Gazebo Classic and the localizer is evaluated as a **passive observer**, which shows three things
offline cannot:

- following while odometry drift accumulates
- recovery from a kidnap (teleport)
- what TRACK actually buys (against running GLOBAL on every scan)

## Layout

```text
sim/
├── make_env.py        generates the world SDF, occupancy grid, and route from one wall definition
├── models/robot.urdf  differential drive + 2D LiDAR + ground-truth plugin
├── drive_node.py      path following (on ground truth) and error recording
├── summarize_run.py   summary of run.csv
└── run_sim.sh         runs all of the above inside Docker
```

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

```bash
./sim/run_sim.sh out 240          # 240 s continuous drive
KIDNAP_AT=120 ./sim/run_sim.sh out_kidnap 240      # teleport at 120 s
OFL_ARGS="-p enable_track:=false" ./sim/run_sim.sh out_notrack 240   # no TRACK
```

Needs an image with Gazebo Classic and `gazebo_ros` (set `IMAGE`; default
`bac_gazebo_runtime:humble`). No network required (it runs with `--network none`).

Outputs are `out/run.csv` (a 20 Hz time series) and `out/{gzserver,ofl,drive}.log`.
`summarize_run.py` reports the median, 95th percentile and maximum error, the fraction within 0.5 m,
and the recovery time after a kidnap.

## Notes

- **The real-time factor is not 1.0.** With ODE at 500 iter/s and a 360-ray sensor this machine runs
  at about 0.4x. The localizer keeps up with the sensor, but check the factor before reading
  "milliseconds per scan" as deployment headroom.
- The world is static. No dynamic obstacles. The offline harness covers stationary phantoms with its
  `phantom*` conditions, but **continuously moving obstacles are measured nowhere**.
- Setting `real_time_update_rate=0` ("as fast as possible") runs hundreds of times faster than real
  time and nothing can keep up; `make_env.py` pins it to 500 iter/s.
