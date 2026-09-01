# Design

English | [日本語](../design.md)

## Goal and non-goals

Estimate the map-frame pose `(x, y, yaw)` from one 2D `LaserScan` and an occupancy grid,
**with no initial guess**.

**It does not track.** The pose goes to `/initialpose` and everything after that is left to
AMCL or similar (the same division of labour as CBGL). There is no odometry, no state
machine, no ICP refinement, and no multi-scan accumulation. Build those on top if a
continuous deployment needs them.

## Score

Both the template (scan) and the map become a three-channel field:

```text
f(p) = ( mass(p), lam * mass(p) * nx(p), lam * mass(p) * ny(p) )   image coords (y down)
```

- `mass` is wall occupancy (0 / 1). On the map side it is the occupancy grid binarised and
  dilated 3x3; on the scan side the beam endpoints dropped onto the same grid and dilated
- `(nx, ny)` is the **unit normal facing the observer**. On the map side it is the gradient
  of the free-space mask (pointing into free space); on the scan side it is perpendicular to
  the tangent through adjacent beams, flipped toward the sensor. A thin wall with free space
  on both sides produces no map-side gradient, and there the field degenerates to the mass
  channel alone

The score of a pose `(p, alpha)` is

```text
s(p, alpha) = < f_map , R_alpha f_scan >_p / sqrt(E_scan)
```

where `R_alpha` rotates both the positions and the vector values, and `E_scan` is the total
template-side energy.

**Keeping the denominator template-side only is the key point.** Dividing also by the
map-side energy at the candidate position (a cosine normalisation) rebuilds the reference
signal per candidate, which destroys comparability between candidates and degrades the
ranking: top-1 measured 85.2% without it and 73.1% with it
(`radon_global_localization/docs/image_space_control.md`, section 5).

## Search

A coarse-to-fine image pyramid.

```text
coarse level (0.2 m/px by default)
  for alpha over the full circle in coarse_angle_step increments
    rotate the template canvas by alpha -> forward DFT
    multiply against the map-side spectra (precomputed at load) and take ONE inverse DFT
    -> the score at every position, at once
    keep peaks_per_angle NMS'd peaks for this angle
  NMS the candidates down to candidate_pool_size
fine levels (0.1 -> 0.05 m/px)
  refine each candidate over +-refine_angle_window deg x +-refine_search_m by direct
  correlation over a sparse point list, then NMS
  (intermediate_pool_size at intermediate levels, candidate_pool_size at the last)
```

**Getting the whole position score field at once is what this formulation buys.** Matching
the same representation in sinogram space makes a position a sinusoid across 180 rows rather
than an index, which cannot be reduced to an FFT over position and has to be sampled on a
coarse grid. That difference shows up in the results (benchmark.md).

### The correlation padding can be minimal

Sensor positions are only ever placed inside the unpadded map, so the pixels the template
touches lie within `[margin - R, margin + map_w - 1 + R]`, where R is the footprint radius
(`max_range` / the level's resolution). The map is already zero-padded by `margin`, so
**only `max(0, R - margin)` extra is needed**. Satisfying
`margin_pixels >= ceil(sqrt(2) * max_range / match_resolution) + 1` makes the padding zero
and the DFT minimal.

Padding naively by `R` grows the DFT side by about 1.4x and doubles the coarse-stage cost.

## Acceptance

Only when the top-1 and top-2 scores are within `wfrac_margin` of each other does **WFRAC**
(the fraction of scan points that do not land on a map wall) re-select the candidate with
the smallest value. When the scores are not close, nothing is touched: it is a tie-break,
not a selector.

The threshold 1.05 was calibrated over nine maps and amounts to "almost never fires". The
sinogram method depends on WFRAC for +4.4 pt; this method gains only +1.4 pt, because its
top-1 is already strong.

`global_min_margin` suppresses publishing when top1/top2 falls below it; the default 1.0 is
effectively off. Raise it where a wrong commit is costly.

## Coordinates and conventions

- Image coordinates are x right, y down. The conversion to world folds in the map origin,
  the resolution, and the y flip
- The scan grid puts the sensor at the centre pixel and rotates with the same sign as the
  world yaw (because image y points down, a positive `getRotationMatrix2D` angle is the yaw)
- `OccupancyGrid` has row 0 at minimum y, so it is flipped on load to match the map_server
  convention (row 0 = maximum y)

## Invariants

- `margin_pixels >= ceil(max_range / match_resolution)`. Below this the template is clipped
  at the map border and poses near the edge become structurally unreachable. The node raises
  it and warns
- `margin_pixels >= ceil(sqrt(2) * max_range / match_resolution) + 1` makes the correlation
  padding zero (a performance recommendation, not a correctness condition)
- `coarse_angle_step` divides 360
- No estimate is produced without a map, or with fewer than 8 valid beams
- Candidates come back score-descending and NMS-separated

`tests/test_matcher.cpp` checks these.

## Known limits

- **There is no completeness guarantee.** The search descends from the argmax of a coarse
  grid, so a true peak outside that grid's basin is missed. BBS (Hess 2016) uses a bounded
  branch-and-bound and always finds the maximum inside the search window. BBS is stronger
  in theory
- Long corridors and periodic wall layouts give several poses the same observation. An
  ambiguity that one scan cannot resolve needs odometry, motion, or another sensor (the
  49-83% success on the corridor maps is that limit being measured)
- The coarse cost scales with **map area x angle count**, while the sinogram method's scales
  with the map diameter, so **a large enough map should eventually favour the latter**
  (it does not flip up to 52 x 50 m)
- A large map costs precomputation memory and initialisation time (three channels of
  spectra per level)
- Finite range, occlusion, and map-versus-site differences break the ideal correlation
