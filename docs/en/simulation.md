# Continuous-drive validation in Gazebo

English | [日本語](../simulation.md)

Measured: 2026-09-01 (re-measured 2026-09-02)

The offline harness ([benchmark.md](benchmark.md)) only evaluates one scan at a time. Here the
robot drives for 240 s (about 121 m) inside Gazebo Classic, to see the **three things offline
cannot show**:

- following while odometry drift accumulates
- recovery from a kidnap (teleport)
- what TRACK actually buys (against running GLOBAL on every scan)

The environment and how to run it are in [sim/README.md](../../sim/README.md).

## Conditions

- A 24 x 18 m office (outer walls, three rooms, a diagonal partition, pillars). **A single wall
  definition generates both the world SDF and the occupancy grid**, so the two cannot disagree
- Differential drive with a 360-beam / 10 m / 10 Hz 2D LiDAR (Gaussian noise, sigma = 0.01 m).
  Odometry comes from the wheel encoders (`odometry_source=0`) and genuinely slips
- A 79 m closed route planned on the map through cells with at least 0.9 m clearance.
  **Path following uses ground truth**: the controller is not what is under test, and driving on
  the localizer's own output would put the robot into a wall the moment it slipped, ending the
  measurement. The localizer sees only `/scan` and `/odom` and never affects the motion
- 240 s recorded at 20 Hz (4801 samples). The error is "the latest estimate against ground truth
  at that instant"
- Default parameters (`config/params.yaml`), 6 threads

## Results

| Scenario | median | 95% | max | within 0.5 m | yaw median / max |
|---|---:|---:|---:|---:|---:|
| **Baseline (GLOBAL -> TRACK)** | **0.037 m** | 0.073 m | 1.43 m | **99.8%** | 0.32 / 31.4 deg |
| No TRACK (GLOBAL every scan) | 0.065 m | 0.118 m | **19.09 m** | 97.8% | 0.55 / **179.4** deg |
| Kidnap, before (to 120 s) | 0.051 m | 0.099 m | 11.77 m | 99.1% | 0.47 / 135.3 deg |
| Kidnap, after (120-240 s) | 0.061 m | 0.063 m | 11.21 m | **99.6%** | 0.00 / 92.6 deg |
| *reference: odometry alone* | *5.76 m* | — | *12.50 m* | — | — |

121.7 m driven; 10 ms of compute per scan (about 10% duty against the 10 Hz sensor);
23 ms with TRACK disabled.

> The table above is the 2026-09-02 re-measurement. Ground truth was raised from 20 to 50 Hz,
> lowering the floor of the error measurement from 0.025 to 0.010 m, so it was measured again.
> **The 2026-09-01 measurement (median 0.039 m / 99.7% within 0.5 m / 0.6 s recovery / 19.45 m
> and 179.8 deg without TRACK) reproduces.** The median barely moved when the floor was lowered,
> so **0.037 m is the localizer's own error, not the floor of the measurement.**

**The localizer holds 0.04 m while odometry alone drifts to 5.8 m.**

### TRACK earns its place

Turning TRACK off and running GLOBAL on every scan produces **180-degree wrong locks** (max error
19.09 m / 179.4 deg; 97.8% within 0.5 m against 99.8%). The median barely moves, so what TRACK buys
is not average accuracy but **not answering wrongly at ambiguous moments** — the same mechanism the
offline measurement showed on the corridor maps (GLOBAL 50.7% -> TRACK 84.5%).

## A defect this validation found and fixed: a jump gate cannot detect a wrong lock

**The first implementation never recovered from the kidnap.** For the 120 s after the teleport the
error stayed at a median of 7.97 m and 90 deg, with 0.1% within 0.5 m.

The mechanism is clear. TRACK accepted or rejected on the **jump from the prior**, but the prior is
built from the previous accepted pose. Once the track lands somewhere wrong, every subsequent prior
comes from that wrong place, so each solution looks like it did not jump. A wrong lock is
self-consistent, and a jump gate cannot catch it in principle. In the run, not one rejection fired,
and the score (~42) and margin (~1.9) were indistinguishable from healthy ones.

**The fix**: judge with a quantity that does not depend on the prior. `track_max_wfrac` was added:
in TRACK, if the accepted candidate's WFRAC (the fraction of scan points that do not land on a map
wall) exceeds the threshold, the scan is rejected. WFRAC never consults the prior, so it jumps at a
wrong lock.

| | WFRAC |
|---|---:|
| Healthy (median / 95th percentile) | 0.000 / 0.000 |
| Wrong lock (max) | 0.740 |

The separation is stark and the 0.35 threshold sits in the gap. Result:

| | Recovery | within 0.5 m after kidnap |
|---|---|---:|
| Jump gate only (before) | **never** | 0.1% |
| + WFRAC gate (after) | **0.6 s** | 99.6% |

**Of 60 rejections, 59 came from WFRAC and only 1 from the jump gate.** The jump gate catches a
one-off mismatch but does nothing about a broken track.

After the fix, a shorter wrong lock unrelated to the kidnap (t = 55.8-56.5 s, up to 11.9 m) was also
caught and recovered from in 0.7 s, so **wrong locks are not a kidnap-only phenomenon**.

## Limits of this validation

- **Check the real-time factor.** Run one at a time, Gazebo sustains 1.00x here (the 2026-09-02
  re-measurement, and every closed-loop condition). The 0.38x recorded on 2026-09-01 is a
  measurement on the same machine that did not separate out other load running at the time. The
  localizer keeps up with the sensor without dropping scans, so it is not the bottleneck — but
  **read "10 ms per scan" against the factor** before treating it as deployment headroom.
- **The world is static.** Moving obstacles and the closed loop with Nav2 are measured separately in
  [nav2_closed_loop.md](nav2_closed_loop.md).
- **One environment, one robot, one speed profile.** All three scenarios were run twice (2026-09-01
  and 09-02) with the same conclusions, but outliers such as the maximum error still deserve less
  trust than the median. The closed-loop side repeats the static condition four times and finds the
  excursion count varying between 0 and 2 ([nav2_closed_loop.md](nav2_closed_loop.md)).
- **Path following uses ground truth.** That isolates the localizer, but by the same token it does
  not measure the effect of localization error on navigation. That is measured in
  [nav2_closed_loop.md](nav2_closed_loop.md) (conclusion: the excursions themselves last under a
  second, but each one invalidates the local costmap and the transformed global plan, aborts
  `follow_path`, and costs 2.5 s of standing still).
- Recovery time (0.6 s) is defined as the first instant after the kidnap at which the error stayed
  below 0.5 m for 2 s. The GLOBAL search itself finds the pose in a single scan (about 25 ms).
