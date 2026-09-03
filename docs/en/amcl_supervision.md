# Supervising AMCL and reseeding it

English | [日本語](../amcl_supervision.md)

Measured 2026-09-03.

With `supervise_amcl: true` the node stops publishing `map -> odom` itself and
**supervises AMCL instead**. It hands AMCL an initial pose at startup, then checks
AMCL's pose against the map on every scan and only reseeds it through `/initialpose`
when it judges AMCL to be broken. Nothing changes on the AMCL side beyond
`set_initial_pose: false`.

This document measures that configuration in the same environment as
[nav2_closed_loop.md](nav2_closed_loop.md).

## Result

**Over eight 300 s closed-loop runs, supervision all but removes the time spent
wrong under both kidnap and dynamic obstacles, and restores goal completion from
30-40% to over 90%.**

| Condition | Median error | Time > 0.5 m | Goals | Distance driven | Reseeds |
|---|---:|---:|---:|---:|---:|
| kidnap, unsupervised (2 runs) | 1.398 / 1.267 m | **183.3 / 182.1 s** | 3/10, 3/11 | 27.1 / 26.0 m | 14 / 14 |
| kidnap, supervised (2 runs) | **0.058 / 0.053 m** | **7.8 / 5.3 s** | 14/15, 14/15 | 131.6 / 133.7 m | 14 / 9 |
| dynamic, unsupervised | 2.600 m | **253.3 s** | 4/11 | 42.7 m | 24 |
| dynamic, supervised | **0.059 m** | **0.0 s** | 16/17 | 136.0 m | **4** |
| static, unsupervised | 0.055 m | 0.0 s | 17/18 | 136.4 m | 4 |
| static, supervised | 0.056 m | 0.0 s | 16/17 | 136.6 m | 4 |

**Distance driven is listed because error statistics alone do not describe this
condition.** When localization is wrong, Nav2 throws the path away and enters
recovery, so a run with large error is also a run that goes nowhere. The
unsupervised kidnap runs covered 26-27 m in 300 s against 131-134 m supervised;
"it did not drive" is the real damage, not "the error was 1.4 m".

## Two separate mechanisms are at work

**The most important finding here is that the larger improvement comes not from the
supervision but from suppressing reseeds.**

### 1. The reseed gate (large effect)

The node used to publish `/initialpose` unconditionally every time **it** lost
tracking and re-acquired from GLOBAL. But the reasons OFL loses tracking (a blocked
view, a crowd of objects absent from the map) have **nothing to do with whether AMCL
is broken**. Dumping a fresh particle cloud on a healthy AMCL creates a pose jump by
itself, and because the local costmap lives in the odom frame the whole trajectory is
invalidated and `follow_path` aborts ([nav2_closed_loop.md](nav2_closed_loop.md)).

`initialpose_repeat: 5` then **amplifies one track loss into five reseeds**. In the
dynamic-obstacle run OFL lost tracking 4 times and AMCL reseeded 24 times.

In supervised mode **every reseed, including the startup handoff, is gated on whether
AMCL actually disagrees with us**. If it points at the same place, reseeding fixes
nothing and leaves only the jump. Effect: 24 reseeds down to 4 (dynamic obstacles).

The static pair shows the mechanism cleanly. The supervised run lost tracking 3
times, yet AMCL reseeded only at startup (4) because the gate suppressed 15 — and it
matched the unsupervised run, which happened to lose tracking 0 times.

### 2. WFRAC supervision (rare, but catches what nothing else does)

Across the six supervised runs the supervisor fired **once**, on AMCL after a kidnap:

```text
amcl looks lost (wfrac 0.620 vs self 0.000, apart 1.55 m, 94.1 deg); reseeding
```

**A position threshold cannot catch this mis-lock.** AMCL was only 1.55 m away, well
inside anything you would use as a jump threshold. What catches it is the **94 degrees
of yaw** and the **WFRAC of 0.620 against our own 0.000**.

### Zero false positives

Across the six supervised runs — one of them with dynamic obstacles absent from the
map — there was **not one spurious reseed**, which is what the decision rule is
designed for.

Obstacles missing from the map push the healthy WFRAC itself close to 0.50, so an
absolute threshold calibrated on a static world (0.35) misfires. Comparing against
**our own WFRAC measured on the same scan** cancels that, because the obstacles land
on both poses equally. The observed separation was 0.000 healthy against 0.620
mis-locked — a gap of 0.62.

## Statistical calibration of the thresholds (synthetic, added later)

Measured 2026-09-03

The closed loop produced a single supervisor firing, so the thresholds were not
statistically calibrated. Observing a broken AMCL in closed loop at scale is
expensive, so instead the failure modes were **synthesised as pose displacements**
on top of the [eval/](../../eval/) scan dump (15 disturbance conditions x 40 poses),
and the full per-scan rule (geometry: absolute WFRAC and excess; disagreement:
0.5 m or 20 deg) was scored per class (`eval/run_reseed_margin.sh`, 1,800 samples
per class):

| Class | Displacement | Fire rate |
|---|---|---:|
| healthy AMCL | <= 0.15 m / <= 5 deg | **0.0% (0/1800)** |
| drifted | 0.15–1.5 m / <= 20 deg | 72.5% |
| mis-lock | 1–3 m, any yaw | **99.5%** |
| kidnap | >= 3 m, any yaw | **99.7%** |

What it shows:

- **Zero false fires across all 15 disturbance conditions**, including 25 unmapped
  obstacles (phantom25) and half the field of view occluded (sector180). The
  excess-over-own-baseline term cancels the disturbances as designed.
- **The firing boundary is set by the disagreement gate, not by the WFRAC
  thresholds.** The fire rate steps from ~0% to 97% at 0.5 m; the geometry stage is
  already 94% positive by 0.3–0.4 m. Sweeping `supervise_max_wfrac` /
  `supervise_wfrac_excess` over the whole 0.25–0.50 / 0.05–0.30 grid leaves healthy
  at 0% and detection at >= 97% everywhere. **The defaults (0.35 / 0.15) are not
  sitting on a cliff.**
- The band containing the one real event (1.55 m / 94 deg) detects at 99.5% per
  scan; the misses concentrate in sector180 (93.3%).
- This measures the per-scan test. The deployed rule adds the 10-consecutive-scan
  count and the minimum interval on top, which push false fires further down and add
  about one second of detection latency.

## Closed-loop re-measurement with the receipt acknowledgement (added later)

Measured 2026-09-03

With the repeat amplification cut off by the receipt acknowledgement (see Limits),
one supervised kidnap run and one supervised static run were re-taken:

| Condition | Median error | Time > 0.5 m | Goals | AMCL scatters |
|---|---:|---:|---:|---:|
| kidnap, supervised + ack | 0.051 / 0.056 m (pre/post) | **0.80 s** | 14/15 | **3** (was 14 / 9) |
| static, supervised + ack | 0.056 m | 0.0 s | 16/17 | **1** (was 4) |

- The startup handoff was acknowledged 0.1 s after the first publish — AMCL came
  back with a pose next to the seed — and the remaining four repeats were dropped
  (static scatters 4 -> 1).
- In the kidnap run the recovery `/initialpose` was cut off the same way; 3 scatters
  in 300 s against 14 / 9 with the old implementation. Time wrong shrank from
  7.8 / 5.3 s to 0.80 s, with a 0.9 s recovery from the kidnap. A scatter is itself
  a pose jump, so fewer of them means less post-convergence disturbance — though
  this is a one-run-per-condition comparison and run-to-run spread is not separated.
- Median error and goals match the old implementation; no regression is visible.

## Conditions

Identical environment, robot, route and Nav2 configuration to
[nav2_closed_loop.md](nav2_closed_loop.md). **The only thing that changes between
conditions is `supervise_amcl`.**

- `LOC=amcl_ofl` — unsupervised. OFL publishes `/initialpose` on first lock and
  unconditionally on every later re-acquisition (the previous behaviour)
- `LOC=amcl_sup` — supervised. Same node with `supervise_amcl: true`
- Neither gives AMCL a human-supplied initial pose (`set_initial_pose: false`)
- Kidnap at 120 s; dynamic obstacles are 3 walkers and 1 cart, none baked into the map

To reproduce:

```bash
./sim/docker/build_image.sh                              # once
KIDNAP_AT=120 LOC=amcl_ofl ./sim/run_nav2.sh out_base 300
KIDNAP_AT=120 LOC=amcl_sup ./sim/run_nav2.sh out_sup  300
```

## Limits

- **Few runs.** Two per kidnap condition, one each for dynamic and static. The kidnap
  pair is consistent 2/2 (both unsupervised runs above 180 s, both supervised under
  8 s), but dynamic and static have a single run each.
- **The supervisor fired only once in closed loop.** Six runs establish that it does
  not misfire, and the thresholds are now backed by the synthetic calibration above
  (0/1800 false fires, stable across the whole threshold grid). But the calibration
  approximates "broken" as a pose displacement; it does not reproduce how a particle
  filter actually degrades (multi-modality, covariance inflation). Closed-loop firing
  events still need to accumulate.
- **The `initialpose_repeat` amplification is now cut off by a receipt
  acknowledgement** (after these measurements were taken): once a post-seed
  `amcl_pose` lands near the seed, the remaining repeats are dropped
  ([seed_handoff.hpp](../../include/oriented_field_localization/seed_handoff.hpp),
  22 unit tests). Repeats also carry the seed forward by odometry before publishing
  (a repeat while driving used to scatter at a stale pose). **The table above still
  carries all five repeats**; the re-measurement with the acknowledgement is in the
  section above — scatters fell 14 / 9 -> 3 (kidnap) and 4 -> 1 (static), with no
  visible regression.
- **One run in nine was invalid.** Nav2 dropped the action response for the first
  goal (`bt_navigator.rclcpp_action: Failed to send goal response (timeout)`), the
  driver waited the full 300 s and the robot never moved. It was excluded and redone.
  This is a harness startup flake unrelated to localization, and **you cannot notice
  it without looking at distance driven** — always read that column.
- Supervision only runs on scans where OFL itself is in TRACK and self-consistent; a
  one-shot GLOBAL solution may have drawn an ambiguous pose and cannot serve as the
  baseline.
