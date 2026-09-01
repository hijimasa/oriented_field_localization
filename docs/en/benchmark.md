# Benchmark — against BBS and the Radon sinogram method

English | [日本語](../benchmark.md)

Measured: 2026-09-01

## What is compared

| Method | What it is | Bundled |
|---|---|---|
| **This method (OFL)** | FFT correlation of an oriented field (wall mass + sensor-facing normals) in image space, coarse-to-fine over an image pyramid | yes |
| **BBS** | Olson (2009) / Hess (2016) style correlative branch-and-bound: a likelihood field plus a multi-resolution bound, searched over the whole map and all 360 degrees, **guaranteed to find the highest-scoring pose in the window** | yes (`eval/bbs_eval.cpp`) |
| Radon sinogram method | NCC-HF from the sibling package `radon_global_localization` | no (numbers quoted) |

BBS reads the same dumped scans, so that comparison is **paired**. The Radon numbers come
from the sibling package's evaluator run on the same scan set (reproduction at the end).

## Conditions

- **Nine maps**
  - Four synthetic maps (`eval/make_synthetic_map.py`, seeds 7 / 11 / 23 / 42, 30 x 30 m at
    0.02 m/px). No third-party data; a fixed seed regenerates them exactly
  - Two Intel Research Lab maps (29.7 x 28.8 m at 0.02 m/px, 52.6 x 50.2 m at 0.05 m/px).
    **Their redistribution terms are unverified, so they are not bundled**; substitute them
    if you have them
  - Three corridor maps (15 x 16 m at 0.05 m/px), corridor-dominated and strongly
    self-similar
- **40 poses x 15 disturbance conditions = 600 trials** per map, **5400 trials** over nine maps
- Success: position error < 1.0 m and angle error < 15 deg
- Sensor: 360 beams, 10 m maximum range, `match_resolution` 0.05 m/px
- The close-range gate is **0.8 m for both methods** (see below)
- Times are the median per-scan search time. AMD Ryzen 9 5950X, **16 threads** (the physical
  core count; going to 32 logical makes the 2-D FFTs contend over SMT and runs 1.4x slower)
- Significance by McNemar (exact binomial, two-sided)

## Results

| Map | OFL top-1 | **OFL + WFRAC** | BBS | OFL [s] | BBS [s] |
|---|---:|---:|---:|---:|---:|
| synthetic_s7 | 91.7% | **94.5%** | 71.0% | 0.010 | 0.013 |
| synthetic_s11 | 87.8% | **91.7%** | 67.7% | 0.010 | 0.013 |
| synthetic_s23 | 90.8% | **93.2%** | 79.0% | 0.010 | 0.013 |
| synthetic_s42 | 94.8% | **96.8%** | 78.2% | 0.010 | 0.013 |
| intel_lab | 99.0% | **99.5%** | 76.3% | 0.010 | 0.020 |
| intel_log | 99.2% | **99.2%** | 89.7% | 0.025 | 0.037 |
| corridor15x | 73.8% | **73.5%** | 37.0% | 0.007 | 0.001 |
| corridor17 | 82.3% | **83.3%** | 34.3% | 0.007 | 0.001 |
| corridor_zigzag | 50.5% | **50.7%** | 29.7% | 0.006 | 0.001 |
| **Pooled (5400 trials)** | **85.6%** | **86.9%** | **62.5%** | | |

McNemar (BBS vs OFL+WFRAC): **1453 succeeded only for OFL, 136 only for BBS, p = 1.6e-278.**

**BBS has a proven completeness guarantee** (it always finds the highest-scoring pose in the
window) and this method does not. The gap is therefore not about exhaustiveness but about
**score discrimination**: the BBS likelihood field only asks whether a scan point lies near
a wall, while this method also requires the wall **orientation** to agree.

### Per condition (pooled, 5400 trials)

| Condition | OFL top-1 | OFL + WFRAC | BBS |
|---|---:|---:|---:|
| `clean` | 91.9% | 93.1% | 64.7% |
| `phantom3` / `8` / `15` / `25` (N obstacles absent from the map) | 89.2 / 87.5 / 84.4 / 80.0% | 90.3 / 88.3 / 85.8 / 80.6% | 64.2 / 64.2 / 62.8 / 53.3% |
| `sector60` / `120` / `180` (a sector blocked by close walls) | 88.1 / 82.8 / 75.3% | 89.2 / 83.9 / 73.9% | 64.2 / 56.7 / 39.7% |
| `dropout90` / `180` (a sector with no returns) | 83.3 / 75.3% | 85.3 / 75.8% | 66.7 / 65.8% |
| `mapremove3` / `8` (N wall patches removed from the map) | 95.8 / 98.6% | 97.2 / 98.6% | 68.6 / 73.3% |
| `thin50` / `75` / `90` (beams thinned at random) | 88.1 / 87.2 / 75.8% | 89.2 / 90.0 / 82.8% | 64.4 / 65.3 / 64.2% |

The widest gaps are on `mapremove` (map-versus-site differences) and `sector` (being
surrounded by people or carts); the narrowest are `dropout180` and `thin90`.

### Effect of the close-range gate

`min_range` (discard returns closer than this) measured at 0 and 0.8 m, 5400 trials:

| | min_range = 0 | min_range = 0.8 m |
|---|---:|---:|
| OFL + WFRAC | 86.5% | **86.9%** |
| BBS | 57.6% | **62.5%** |

**BBS is far more sensitive to the gate** (+4.9 pt vs +0.4 pt): it only looks at proximity
to a wall, so the close wall of a blocked sector scores as evidence. This method also checks
orientation, so it is less fooled. **The table above gives both methods the 0.8 m gate**,
i.e. it is set in BBS's favour.

The package default is `min_range: 0.0`, because 0.8 m is a robot-size-dependent value and
cutting it uniformly on a small platform throws away real information.

### TRACK (local search)

The local search used after convergence, measured on the same 5400 scans by giving it a
**prior perturbed at random from ground truth** (a stand-in for odometry error).

| Map | GLOBAL | TRACK (prior error <= 1 m / 10 deg) | TRACK (<= 3 m / 30 deg) | GLOBAL [ms] | TRACK [ms] |
|---|---:|---:|---:|---:|---:|
| synthetic_s7 | 94.5% | **100.0%** | **100.0%** | 9.6 | 4.6 |
| synthetic_s11 | 91.7% | **100.0%** | **100.0%** | 9.6 | 4.5 |
| synthetic_s23 | 93.2% | **100.0%** | **100.0%** | 9.6 | 4.4 |
| synthetic_s42 | 96.8% | **100.0%** | **100.0%** | 9.8 | 4.6 |
| intel_lab | 99.5% | **100.0%** | **100.0%** | 9.5 | 3.8 |
| intel_log | 99.2% | **100.0%** | **100.0%** | 24.5 | 4.5 |
| corridor15x | 73.5% | 91.2% | 90.5% | 6.7 | 4.2 |
| corridor17 | 83.3% | 93.7% | 94.7% | 6.9 | 4.3 |
| corridor_zigzag | 50.7% | 82.5% | 84.5% | 5.6 | 2.5 |
| **Pooled** | **86.9%** | **96.4%** | **96.6%** | | |

- **Inside the window the prior's size does not matter.** 1 m and 3 m give the same result,
  and going outside it (4 m / 40 deg) drops to about 92%. That is the designed behaviour
- **The corridor maps gain the most** (50.7% -> 84.5%): where a single scan cannot resolve a
  self-similar environment, a prior removes the ambiguity
- **TRACK's time does not depend on map size.** On the largest map it is 4.5 ms against
  GLOBAL's 24.5 ms (5.4x)

**This is a simulation of prior error drawn from a uniform distribution, not real odometry
error.** Continuous-driving behaviour (accumulated drift, sustained dynamic obstacles,
recovery from kidnap) is not measured.

### Against the Radon sinogram method

The sibling package's evaluator on **the same 5400 scans** (WFRAC threshold calibrated per
method, same 16 threads):

| Configuration | Success | Time [s] synthetic / intel_log / corridor |
|---|---:|---:|
| Radon, deployed default (levels=2) | 85.2% | 0.070 / 0.104 / 0.051 |
| Radon levels=3 | 84.3% | 0.053 / 0.065 / 0.044 |
| Radon, best (levels=3 + coarse position grid step 2) | 85.7% | 0.060 / 0.084 / 0.045 |
| Radon, fastest (+ halved sinogram rows) | 84.2% | 0.025 / 0.035 / 0.020 |
| **This method (OFL + WFRAC)** | **86.9%** | **0.010 / 0.025 / 0.007** |

**+1.2 to +2.7 pt in success and 2.5-6x in speed.** The two share the representation (wall
mass + sensor-facing normals) and differ only in whether the matching happens in **sinogram
space or image space**. The history and the mechanism are in
`radon_global_localization/docs/image_space_control.md`. In short:

- In sinogram space a position is not an index but a sinusoid across 180 rows, so the
  position search cannot be reduced to an FFT and has to use a coarse grid (step 4). The
  image-space FFT correlation returns the whole position field for free.
- **Even on the same candidate set, the image-space score is the better discriminator**
  (feeding image-space candidates through the Radon score drops top-1 from 85.2% to 73.5%).
  The difference is in the ranking, not in candidate generation.

## Limits

- **Nine maps and 5400 trials is enough within this setting, but it is not a
  generalisation.** The sensor is one configuration (360 beams, 10 m) and
  `match_resolution` is fixed at 0.05. The three corridor maps are only 15 x 16 m, so a 10 m
  scan covers most of the map — outside the "local patch vs whole map" assumption.
- **The BBS implementation is an evaluation baseline**, not Cartographer's own: a
  straightforward likelihood field (sigma 0.1 m, truncated at 0.5 m) plus multi-resolution
  branch-and-bound. Tuning it moves the numbers.
- **This method has no completeness guarantee.** It descends from the argmax of a coarse
  grid, so a true peak outside that basin is lost. Multiple peaks, NMS, and the pyramid are
  mitigations, not a guarantee. BBS is stronger in theory, and that should be said plainly.
- The corridor results (49-83%) show that **a self-similar environment does not resolve from
  a single scan**. That ambiguity belongs to odometry or motion.
- Times come from one x86 machine; the ratio may differ on embedded hardware.
- **TRACK is only measured as a one-shot local search.** The state machine (GLOBAL <-> TRACK
  transitions, jump rejection, kidnap recovery) is checked by unit tests for its invariants,
  but its behaviour over a continuous drive is not measured.

## Reproduction

```bash
cd eval
./run_compare.sh out 40          # OFL vs BBS on one synthetic map
```

To reproduce the nine-map table, generate the four synthetic maps with
`eval/make_synthetic_map.py --seed {7,11,23,42}`, supply the Intel and corridor maps
yourself, and per map run

```bash
OFL_MAP_PGM=<map>.pgm OFL_MAP_RES=<res> OFL_NEAR_WALL_M=8 ./out/make_scans 40 > scans.csv
OFL_MAP_RES=<res> OFL_MIN_RANGE=0.8 ./out/ofl_eval scans.csv <map>.pgm > ofl.csv
OFL_MAP_RES=<res> ./out/bbs_eval scans.csv <map>.pgm 1.0 0.05 0.8 > bbs.csv
python3 summarize.py ofl=ofl.csv bbs=bbs.csv
```

`make_scans` uses the same random sequence for poses and disturbances as
`disturb_eval2 --dump` in `radon_global_localization`, so **the same map and trial count
give byte-identical scans in both packages** and the numbers can be placed side by side.
