# Closed-loop validation with Nav2, and dynamic obstacles

English | [日本語](../nav2_closed_loop.md)

Measured: 2026-09-02

The continuous-drive validation in [simulation.md](simulation.md) is **open loop**:
path following uses ground truth, so the localizer is a passive observer and a bad
estimate never affects driving. Here that restriction is removed — **Nav2 drives on
the localizer's output alone** — which makes measurable what open loop cannot reach.

- What a localization **jump** does to control, the costmaps, and planning
- What the error finally becomes in metres of clearance, and in goals reached
- Whether the whole navigation stack, not just the localizer, recovers from a kidnap
- Which method breaks first when **unmapped moving obstacles** are present

This validation led to recalibrating the two acceptance thresholds
(`max_accept_jump_m`, `global_min_margin`) and changing their defaults (Result 4).
**The median does not move; the excursions almost disappear.**

The conclusion first. **What separates the methods in a closed loop is not accuracy
but how long they stay wrong.** Total time with position error above 0.5 m in a
300 s run:

| Condition | This package (OFL) | AMCL |
|---|---:|---:|
| Static (OFL x4 / AMCL x1) | 0.7 / 1.0 / 0.0 / 0.0 s | 0.0 s |
| Dynamic obstacles (OFL x4 / AMCL x3) | 0.2 / 0.6 / 0.1 / 0.5 s | **106.1 / 22.4 / 0.0 s** |
| 150 s after a kidnap (x2 each) | 0.5 / 1.2 s | **149.9 / 149.9 s** (all of it) |

Every OFL error stayed **under a second in total** across the eight static and dynamic
runs (within 1.2 s including the kidnaps), and repaired itself. AMCL is the steadier of
the two in a static world, but with dynamic obstacles it went badly wrong in **2 of 3
runs**, and after a kidnap in 2 of 2. The instantaneous worst case is 8 -- 14 m for both,
so **what separates them is not accuracy but whether they can come back after going
wrong.**

The two are not exclusive. A configuration where **OFL supplies the initial pose and
AMCL tracks** (`LOC=amcl_ofl`) was measured too, and drives as well as AMCL does without
anyone handing it an initial pose.

**The table above was measured at the then-current defaults.** After the threshold
recalibration of Result 4, the static condition gives a maximum error of 0.33 -- 0.46 m
with zero excursions (3 runs) and the dynamic one 0.33 / 0.10 m (2 runs): **the tail
disadvantage against AMCL in a static world is essentially gone** (AMCL's maximum was
0.213 m, against 0.16 -- 0.46 m here). What remains is the structural difference in
step-to-step smoothness (discontinuity 95%: 0.050 m against 0.014 m — per-scan
independent estimation against a recursive filter).

## Setup

- Environment, robot and route are the same 24 x 18 m office as
  [simulation.md](simulation.md); one wall definition generates both the world SDF
  and the occupancy grid
- Nav2 (Humble): `map_server` + NavFn + DWB + `behavior_server` + `bt_navigator`;
  robot radius 0.28 m, inflation 0.5 m, control at 10 Hz, 5 x 5 m rolling local costmap
- **Seven goals** resampled along the route at 11 m of arc length, sent in turn via
  `NavigateToPose`. About two laps in 300 s
- **Only the localizer (whatever publishes map -> odom) is swapped.** Everything else
  is identical across conditions
  - `ground truth` — map -> odom from the true pose. The **ceiling of the navigation stack**
  - `OFL` — this package (`tf_mode: map_to_odom`, `auto_localize: true`)
  - `AMCL` — `nav2_amcl` (likelihood field, 500-2000 particles). **Given the true
    initial pose** (AMCL has no global localization, so it cannot start without one —
    a condition favourable to AMCL)
- Dynamic obstacles: three 40 cm walkers (0.7-0.9 m/s) and one 70 x 50 cm cart
  (0.45 m/s). **None of them are in the map.** Their poses are an explicit function of
  simulation time and are teleported, so the trajectories are identical across conditions
- Error is TF `map -> base_link` (the pose Nav2 actually uses) against ground truth,
  published at 50 Hz. The real-time factor was 1.00x in every condition

### What "discontinuity fed into control" means

Measuring a global localizer's jumps by the change in `map -> odom` is **wrong**.
Wheel odometry slips and jumps by up to 0.8 m in a single step, and the localizer
cancels that every scan, so the change in `map -> odom` picks up the odometry noise
wholesale (measured max 0.85 m, while the estimate error over the same interval stayed
below 0.09 m).

What enters control is `map -> base`, and with the real motion removed — that is, the
one-step change of the error vector `est - gt`. Every "discontinuity" below is that.

> **Results 1 -- 3 were measured at the then-current defaults**
> (`max_accept_jump_m: 2.0`, `global_min_margin: 1.0`). Both were recalibrated in
> Result 4 and the defaults changed; at the current defaults the same drives show
> almost no excursions (maximum error 0.33 -- 0.46 m over three static runs, zero
> excursions).

## Result 1: static environment

| Condition | Error median | 95% | max | time > 0.5 m | discontinuity 95% / max | goals |
|---|---:|---:|---:|---:|---:|---:|
| Ground truth (ceiling) | 0.010 m | 0.010 | 0.017 m | 0.0 s | 0.001 / 0.012 m | 17 |
| **OFL** run 1 | 0.036 m | 0.075 | 3.483 m | 0.7 s | 0.050 / 1.948 m | 15 |
| **OFL** run 2 | 0.039 m | 0.077 | 3.758 m | 1.0 s | — | 16 |
| **OFL** run 3 | 0.042 m | 0.077 | 0.175 m | 0.0 s | — | 17 |
| **OFL** run 4 | 0.039 m | — | 0.137 m | 0.0 s | — | 16 |
| AMCL | 0.050 m | 0.139 | 0.213 m | 0.0 s | 0.014 / 0.052 m | 17 |

OFL is better on the median (0.036-0.042 against 0.050). It also produces 0 -- 2
sub-second excursions per 300 s (2, 2, 0, 0 across four runs). AMCL produced none.
**In a purely static world AMCL is the steadier of the two.**

### The excursions are a property of the place, not of chance

The excursions at t=34.4 s (run 1) and t=33.2 s (run 2) had **the same shape at the
same place**.

| | true | estimate | error |
|---|---|---|---:|
| run 1 | (18.17, 10.56) | (20.10, 10.55) | 1.93 m |
| run 2 | (17.95, 10.52) | (19.93, 10.50) | 1.97 m |

Both landed on a solution shifted by **exactly 2.0 m along the corridor** between the
south wall of the top-right room (y=11.5) and the pillar at (19.5, 9). Position along
a locally self-similar corridor is undetermined — the known weakness of this method
(GLOBAL 50.7% on the corridor map in [benchmark.md](benchmark.md)) showing up
unchanged in continuous driving.

### The jump stops Nav2

Right after the t=34.4 s excursion, `controller_server` produced, within 0.5 s:

```text
A scoring function failed to prepare        (x4)
No valid trajectories out of 314!
Resulting plan has 0 poses in it.           (x5, 0.1 s apart)
[follow_path] [ActionServer] Aborting handle.
```

and the robot stood still for 2.5 s. The mechanism is plain. **The local costmap lives
in the odom frame**, so when `map -> odom` jumps, every obstacle written into it moves
relative to the robot. The robot now sits inside cells it marked itself and all
trajectories become invalid. At the same time the global plan (map frame), transformed
into the local costmap, comes out empty and `follow_path` aborts.

That run had three aborts in total, two of which coincide with the excursions (t=34.3
and t=188.2). **No collision and no failed goal**, but the price is a 2.5 s stall per
event.

> **Do not read distance driven as a localization comparison.** Run 1 covered only
> 125.0 m, but 19.5 s of its 26.0 s of stalls came from DWB falling into a local
> minimum at a doorway at t=65 s, where the position error was 0.047 m with no
> discontinuity at all. Run 3, with zero excursions, covered 136.4 m — the same as
> AMCL's 136.5 m.

## Result 2: unmapped moving obstacles

The same 300 s with three walkers and a cart in motion.

| Condition | Error median | max | **time > 0.5 m** | goals | contacts |
|---|---:|---:|---:|---:|---:|
| Ground truth (ceiling) | 0.000 m | 0.146 m | 0.0 s | 15 | 0 |
| **OFL** (default 0.35) run 1 | 0.038 m | 8.477 m | **0.2 s** | 16 | 0 |
| **OFL** (default 0.35) run 2 | 0.039 m | 11.796 m | **0.6 s** | 15 | 0 |
| **OFL** (WFRAC 0.50) run 1 | 0.040 m | 1.183 m | **0.1 s** | 15 | 0 |
| **OFL** (WFRAC 0.50) run 2 | 0.041 m | 12.211 m | **0.5 s** | 15 | 0 |
| AMCL run 1 | 0.092 m | 14.280 m | **106.1 s** | 12 | **1** |
| AMCL run 2 | 0.075 m | 1.518 m | **22.4 s** | 14 | 0 |
| AMCL run 3 | 0.059 m | 0.267 m | **0.0 s** | 16 | 0 |

**AMCL drifted for long stretches in 2 of 3 runs, and in one it diverged at t=220 s and
never came back** (max 14.28 m / 179 deg). Run 3 completed without trouble. **The
breakdown does not happen every time — but when it does there is no way back.**
The trigger is identifiable.

| t | true pose | to obstacle | position error | yaw error |
|---:|---|---:|---:|---:|
| 216 s | (13.7, 13.6) | 0.82 m | 0.042 m | 0.7 deg |
| 217 s | (13.4, 13.5) | **0.22 m** | 0.059 m | 6.2 deg |
| 220 s | (13.0, 13.6) | 0.25 m | 0.358 m | 2.0 deg |
| 222 s | (12.6, 14.8) | 0.24 m | **1.906 m** | **103 deg** |

An unmapped walker came within 0.22 m of the front of the robot and occupied a wide
angular span of the scan; five seconds later the heading was off by 103 degrees. The
likelihood-field model could not absorb it even with `z_rand = 0.4`, and the particle
set collapsed onto a wrong mode. **AMCL has no global localization, so once it falls it
cannot get back.** There was real cost: 5.45% of samples were within 0.30 m of an
obstacle (2.70% in the ground-truth condition), and in one sample the body overlapped one.

OFL produced 8 -- 12 m excursions in 3 of its 4 runs too, so **the two are not different
in instantaneous worst-case error.** The difference is entirely in duration (OFL
0.1 -- 0.6 s, AMCL 0 -- 106 s).

Single-step odometry jumps (>0.15 m) numbered zero in every condition, so this is not
a wheel-odometry artefact. **It is a failure of the sensor model.**

OFL's excursions all repair within a second because of 50-53 WFRAC rejections leading
to 15-18 GLOBAL re-acquisitions — exactly the "detect the wrong lock and take it again"
path added in [simulation.md](simulation.md).

### The WFRAC threshold was calibrated in a static world

The default `track_max_wfrac` of 0.35 comes from a separation measured **in a still
world**: 0.000 when healthy against 0.740 at a wrong lock. When unmapped obstacles
enter the field of view, the healthy WFRAC itself rises and the margin disappears.

| | accepted WFRAC median | max | WFRAC rejections | GLOBAL re-acquisitions | time > 0.5 m |
|---|---:|---:|---:|---:|---:|
| static, threshold 0.35 | 0.000 | 0.350 | 8 | 3 | 0.7 s |
| dynamic, threshold 0.35 (2 runs) | 0.068 | 0.350 / 0.348 | **50 / 53** | **15 / 18** | 0.2 / 0.6 s |
| dynamic, threshold 0.50 (2 runs) | 0.069 | 0.497 / 0.500 | **2 / 8** | **3 / 9** | 0.1 / 0.5 s |

"Accepted WFRAC max 0.350" at threshold 0.35 means it is **pinned against the threshold
itself**. Raising it to 0.50 cuts rejections from 50-53 to 2-8 and GLOBAL restarts from
15-18 to 3-9.

**The error itself does not improve**, though: time above 0.5 m goes from 0.2/0.6 s to
0.1/0.5 s, which is inside the run-to-run spread. Those excursions come from the
corridor self-similarity described above, which no threshold can prevent. **What raising
the threshold buys is fewer needless rejections and GLOBAL restarts** — and a GLOBAL
restart costs 2-3x the compute of TRACK and re-stakes the pose on the ambiguity of a
whole-map search, so that is worth having on its own.

One more caution. **Even at 0.50 the accepted maximum reaches 0.497 / 0.500 — again at
the threshold.** The clean static-world separation of "0.000 healthy against 0.740 wrong"
narrows sharply once unmapped obstacles are present: the healthy side stretches to 0.5.
There is still room before 0.740, but nothing like the static margin.

> **Set `track_max_wfrac` to 0.45 -- 0.50 where unmapped obstacles are expected.**
> The default of 0.35 is a value for a static map and will reject a great many
> perfectly good observations.

## Result 3: kidnap recovery in the closed loop

The robot is teleported to (20.0, 3.5) at t=150 s. Two runs each.

| Condition | median after | recovery | **time > 0.5 m after** | goals | **failed goals** |
|---|---:|---|---:|---:|---:|
| **OFL** | 0.027 / 0.038 m | **0.7 / 0.6 s** | **0.5 / 1.2 s** | 10 / 17 | **0 / 0** |
| AMCL | 7.681 / 8.304 m | **never** | **149.9 / 149.9 s** (all) | 10 / 11 | 3 / 0 |
| **OFL + dynamic + WFRAC 0.5** | 0.035 m | **0.8 s** | **0.7 s** | 12 | **0** |

The recovery time (0.6-0.8 s) is the same as in the open-loop validation. What only the
closed loop shows is **the shape of the failure**. AMCL aborted three goals and, worse,

- **declared it had reached a goal while standing 9.895 m away from it**

That harm exists only in a closed loop. Open loop would merely record "position error
9.9 m"; closed loop has the navigation stack **treat the goal as reached and move on.**

Even in the hardest condition (dynamic obstacles plus a kidnap) OFL recovered in 0.8 s
and completed 12 goals with no failures.

### A wrong lock scars the costmap — but not if it is short

**Scans integrated at a wrong pose are burned into the map as walls where nothing is.**
Those cells are actually free space, so later rays rarely pass through to clear them:
they stay in the global costmap and distort planning. We saved the final global costmap
and counted lethal cells more than 0.6 m away from any wall of the static map as
"phantom obstacles".

| Run | time spent wrong | phantom lethal cells | blobs |
|---|---:|---:|---:|
| OFL, static | 0.0 s | **3** | 3 |
| OFL, kidnap | 1.2 s | **3** | 3 |
| AMCL, kidnap | 149.9 s | **412** | **53** |

**The initial hypothesis — that OFL's 71 s stall came from phantom walls burned in
during a wrong lock — was measured and refuted.** OFL is wrong for under a second, so
there is nothing to burn in (3 cells). The 71 s stall did not reproduce: repeating the
same kidnap gave zero stalls and 146.9 m driven. It was a one-off event of DWB getting
wedged in a corner.

In the run where **AMCL was wrong for 150 s, 412 cells in 53 blobs of nonexistent wall
were left behind.** A localization failure persists not only as error at the time but
**as degradation of the map afterwards.**

## Result 4: recalibrating the two thresholds

The excursions in Result 1 (a solution shifted 1.93 / 1.97 m along the corridor) passed
**just inside** the then-current `max_accept_jump_m` of 2.0 m. The next scan, where the
error grew to 3.50 m, was caught by the same gate (`TRACK reject (jump 3.50)`). At
0.5 m/s and 10 Hz the real motion between scans is 0.05 m, so 2.0 m is a 40x margin.
Tightening it should catch these.

### Tightening works — and opens another hole

`max_accept_jump_m: 0.5`, four static runs:

| | excursions | total > 0.5 m | max error (4 runs) | GLOBAL re-acquisitions |
|---|---:|---:|---|---|
| 2.0 (then default) | 4 | 1.7 s | 3.48 / 3.76 / 0.18 / 0.14 m | 3 / 9 / 6 / 6 |
| **0.5** | 1 | **0.1 s** | 5.85 / **0.15 / 0.13 / 0.13** m | 9 / 3 / 3 / 3 |

Three runs came down to a 0.13 -- 0.15 m maximum. The mechanism is directly visible:
runs that rejected jumps of 1.15 -- 3.35 m in the corridor at (18, 10.5) had no excursion
afterwards. Dynamic obstacles (max 0.18 / 0.10 m) and kidnaps (recovery 0.6 -- 0.8 s, no
failed goals) did not get worse.

**But one run produced 5.85 m.** That run had zero jump rejections; the cause was
different.

```text
t=284.5-284.9  TRACK reject (wfrac 0.36→0.53) -> GLOBAL
t=285.01       GLOBAL accept (11.90, 10.15) margin 1.13   <- correct
t=285.11       GLOBAL accept ( 7.65, 15.50) margin 1.02   <- 5.85 m off
t=285.21       GLOBAL accept (11.90, 10.15) margin 1.32 -> TRACK
```

**Tightening the jump gate raises the number of TRACK rejections, and with it the number
of GLOBAL restarts. Each restart is a lottery ticket on the ambiguity of a whole-map
search.** The open loop showed the same thing.

| open loop, 240 s | max error | GLOBAL restarts | GLOBAL accepts with margin < 1.05 |
|---|---:|---:|---|
| 2.0 (then default), 2 runs | 1.43 / 1.40 m | 6 / 12 | none / none |
| 0.5 only, 3 runs | 0.39 / **11.85** / **6.28** m | 9 / 15 / 18 | none / **[1.02]** / **[1.01]** |

**Exactly the runs that went badly wrong contain exactly one GLOBAL accept with a margin
of 1.01 -- 1.02.** The runs with no excursion contain none. Across the eight static
closed-loop runs, only 1 of 42 GLOBAL accepts had a margin below 1.05 — and it is the one
that landed 5.85 m off (the other 41 were followed by errors under 0.1 m).

### The two have to change together

`global_min_margin` (do not publish if top1/top2 is below this) defaulted to 1.0, i.e.
disabled. Setting it to 1.05 and applying both:

| open loop, 240 s, 3 runs | median | max error | total > 0.5 m | excursions |
|---|---|---|---:|---:|
| 2.0 + 1.0 (then default) | 0.037 / 0.058 | 1.43 / 1.40 m | 0.2 s | 3 -- 4 |
| 0.5 + 1.0 | 0.053 / 0.056 / 0.038 | 0.39 / 11.85 / 6.28 m | 0.0 s | 0 -- 2 |
| **0.5 + 1.05 (current default)** | 0.038 / 0.058 / 0.039 | **0.33 / 0.46 / 0.42 m** | **0.0 s** | **0 / 0 / 0** |

The closed loop agrees: zero excursions across three static runs (max 0.38 / 0.17 /
0.16 m), two dynamic runs (0.33 / 0.10 m) and two kidnaps (0.8 s recovery each, no failed
goals).

**The median does not move at all.** Every setting falls inside the 0.037 -- 0.058 m
run-to-run spread (the control run at the old default was the highest, at 0.058 m).
**The settings show up only in the maximum error and the time spent above 0.5 m.**

> Partway through this work the call was "make the jump gate a default and leave the
> margin as a recommendation". That was **wrong**: the first creates the exposure the
> second closes, so they have to go in together.

### One side effect closed

`global_min_margin > 1` could not be a default while a single `~/global_localization`
request consumed exactly one scan under `auto_localize: false`: **a rejection would leave
the caller with nothing at all.** A rejection on the margin gate now does not consume the
request (it retries on the next scan), which closes that hole.

### What a threshold cannot prevent

With TRACK disabled and GLOBAL running every scan, **the 180-degree flipped wrong lock is
not caught by the margin either** (the open loop still shows a maximum of 18.99 m /
179.3 deg). A flipped solution scores highly in its own right, so its ratio to the runner-up
is wide. Catching that is WFRAC's job, and it does not arise with TRACK enabled.

## Result 5: smoothing only the output (off by default)

Even after the recalibration, the one-step discontinuity remained larger than AMCL's
(95%: 0.049 against 0.014 m). The difference comes not from estimation accuracy but from
**how updates are applied**. AMCL does not update its filter until the robot has moved
`update_min_d = 0.2 m`; in between, map -> odom is constant, so map -> base is pure
odometry — perfectly smooth. This package applies a correction on every scan (10 Hz).

So a path was added that damps **only the published pose**. The internal prior (the
starting point of the next TRACK) stays the raw accepted pose, so pull-in, the WFRAC
test and kidnap detection are all unchanged. What is damped is only the **correction**
away from "the previous output propagated by odometry"; the real motion passes through
untouched.

### A rate limit, not a first-order lag

In TRACK, an accepted pose is already within `max_accept_jump_m` of the prior, so the
one-step correction already has a hard bound. The measurements show that bound at work:
**the 2.211 m maximum discontinuity at the old default of 2.0 m was the gate value
itself** (at the new 0.5 m default the maximum is 0.371 m, with no violations). To keep
that "the bound is guaranteed" property, the smoothing is a slew-rate limit too.

### Three variants measured

| | closed static: median / max / disc. 95% | open base: median / max | open, after kidnap |
|---|---|---|---:|
| no smoothing (x3) | 0.0381 / 0.379 / 0.0494 | 0.038 / 0.325 | **0.070** |
| fixed 0.02 m/scan (x3) | 0.0366 / 0.146 / **0.0200** | 0.047 / 0.468 | 0.150 |
| + saturation escape | — | — | 0.110 |
| **proportional to motion** (x2) | 0.0375 / **0.126** / 0.0308 | 0.038 / 0.870 | **0.086** |

**A fixed cap cannot serve both motion regimes.** It works while cruising, but during the
tight maneuvering in a room after a kidnap the position error doubled (0.070 -> 0.150 m).
Measurement found the cause: odometry error grows at 0.21 m/s while cruising and at
**0.60 m/s** during that maneuvering, against a fixed correction authority of
0.02 m/scan = 0.20 m/s — 3x short. What cannot be absorbed settles as **a steady lag of
about 0.15 m, below the 0.3 m bypass**, and a bypass that only looks at magnitude cannot
see that sustained saturation.

Two remedies were added.

1. **Saturation escape** (`smooth_saturate_scans`): once the limiter has clamped for that
   many scans in a row, give up and snap, bounding the lag in scans. Alone: 0.150 -> 0.110 m
2. **Correction authority proportional to motion** (`smooth_gain`): odometry error grows
   with translation and rotation (the same form as AMCL's `alpha1..alpha4`), so the
   authority should too. In Kalman terms this is process noise scaled with motion, raising
   the gain at speed. 0.150 -> **0.086 m**, essentially back to no smoothing

### Why the default is off

The proportional variant removes the lag as intended and gives the best closed-loop
maximum error of the three (0.379 -> 0.126 m). It is still not the default.

- **The only thing it reliably improves is the one-step discontinuity** (95%:
  0.049 -> 0.031 m). After the recalibration of Result 4 removed the large jumps,
  **no measurement shows that discontinuity costing anything**
- It couples the output to odometry quality and adds three parameters of tuning surface
- The single number that got worse (open-loop maximum, 0.870 m) is unexplained at n=1.
  That said, t≈75.8 s is a site that turns ambiguous regardless of configuration, and the
  three runs without smoothing gave 0.325 / 0.460 / 0.420 m there. One run cannot separate
  the smoothing from run-to-run variation

**It is worth enabling for gentle motion (Nav2 and the like)**: recommended settings are
`smooth_base_m: 0.005` and `smooth_gain: 0.5`.

## A defect this validation found and fixed: the `/initialpose` handoff is lost to a startup race

`publish_initialpose` exists so that OFL can fix the first pose, publish it to
`/initialpose`, and leave the rest to AMCL. Building exactly that (`LOC=amcl_ofl`)
showed that **AMCL never initialised and never drove.**

```text
ofl_node : TRACK accept (3.05, 8.00, 356.0 deg) ...   <- localization is fine
amcl     : AMCL cannot publish a pose or update the transform.
           Please set the initial pose...             <- never received it
```

`/initialpose` was published **once, at the first lock**, with volatile QoS. If Nav2's
lifecycle startup (map_server, then amcl configure/activate) finishes after that first
lock, there is no subscriber yet and the message is **silently dropped**. AMCL's
subscription is volatile, so making the publisher transient_local would not reach a
late-joining subscriber either.

**Fix, in two steps**: republishing once a second up to `initialpose_repeat` times
(default 5) was tried first and **still did not arrive**. AMCL's activate turned out to
be **6 seconds** after OFL's first lock, outside the 5 s window. Waiting by count is
itself a dependency on startup order.

The final form **does not spend the repeat budget while there is no subscriber**: it
waits for `get_subscription_count() > 0` and republishes then, bounded by
`initialpose_wait_s` (default 60 s). That removes the dependency on startup order.

Verified (`LOC=amcl_ofl`, static, 300 s):

| | error median | max | within 0.5 m | goals | initial pose |
|---|---:|---:|---:|---:|---|
| AMCL (given the true initial pose) | 0.050 m | 0.213 m | 100.0% | 17 | **supplied by a human** |
| **OFL initialises -> AMCL tracks** | 0.056 m | 0.165 m | 100.0% | 16 | **not needed** |

`initialPoseReceived` appears five times and AMCL completes the drive **without being
given an initial pose at all**, matching the run where it was handed the true one.
**AMCL's smoothness as a recursive filter (discontinuity max 0.057 m) and this package's
ability to start with no prior compose.**

## Limits of this validation

- **One environment, one robot, one speed profile.** Repeats: static OFL x4, dynamic
  OFL x4, dynamic AMCL x3, kidnap x2 each; everything else once. **The breakdowns vary a
  lot between runs.** AMCL's dynamic condition gave 106.1 / 22.4 / 0.0 s — the third run
  had no trouble at all. The correct statement is not "AMCL breaks with dynamic
  obstacles" but "**it broke in 2 of 3 runs, and when it broke it did not come back**".
  What was stable was the median, and that OFL's time above 0.5 m stayed under a second
  across the eight static and dynamic runs (within 1.2 s over all eleven, kidnaps
  included)
- **DWB is not allowed to reverse** (`min_vel_x: 0.0`). Nose into a corner and it can
  only rotate out; that produced a permanent 71 s stall in the first `OFL + kidnap` run
  (position error 0.02-0.04 m at the time, i.e. localization was fine). **Distance
  driven and "fraction of time moving" carry a lot of navigation-side accident.**
- **AMCL is given the true initial pose.** OFL is given nothing. The asymmetry favours
  AMCL and is deliberate
- **The dynamic obstacles are kinematic and do not move when pushed.** Real people and
  carts do, so the behaviour when wedged is harsher here than in reality
- **The goal tolerance is 0.25 m in xy.** The median "true residual at arrival" of
  0.23-0.26 m is essentially that tolerance, not localization accuracy. What is
  meaningful is a value far outside it (AMCL's 9.895 m after a kidnap)
- The phantom-obstacle count is "lethal cells more than 0.6 m from a static wall", which
  **also counts correctly recorded dynamic obstacles**. It is therefore not compared
  across the dynamic conditions
- Nav2's costmap writes obstacles permanently (`clearing: true` still needs a later ray
  through the cell). Over long runs the costmap drifts toward being more crowded than
  reality; 300 s runs do not separate this effect
