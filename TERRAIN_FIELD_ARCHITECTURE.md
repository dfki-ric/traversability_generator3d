# TerrainField — MLS → TravMap3D redesign

Status: **in construction** (see TODO roadmap at the bottom).
Owner: haider_khan.lodhi@dfki.de. Drafted 2026-07-07.

This document specifies the redesign of `traversability_generator3d`'s MLS→traversability
conversion. It replaces the per-cell RANSAC + threshold-classification + yaw-sampled
orientation pipeline with a layered, uncertainty-aware, robot-independent terrain model and
an exact, analytic robot lens. The external contract (the `TravGenNode` graph consumed by
`ugv_nav4d`) is unchanged — the planner never notices.

---

## 1. Design goals

1. **Planner-invariant.** Output remains `TraversabilityMap3d<TravGenNode*>` with populated
   `nodeType`, `slope`, `plane`, `cost` (via node userData), `allowedOrientations`, and
   connectivity. `ugv_nav4d` compiles and runs unchanged.
2. **Estimate, don't classify.** Internals are continuous quantities with uncertainty;
   classes exist only at the adapter (L4).
3. **Robot-independent core.** L0–L2 never see the robot. Robot semantics (clearance,
   attitude limits, cost) are stateless functions (L3). Changing the robot re-queries; it
   does not regenerate the map.
4. **Exact orientation reasoning.** No yaw sampling, no obstacle inflation. Clearance and
   feasible-heading intervals are closed-form against a Euclidean distance field.
5. **Incremental by construction.** Tile-based recomputation with dirty tracking; full-map
   generation is seed-independent (the "start cell is an obstacle ⇒ empty map" failure
   class is structurally gone).

## 2. Layer overview

```
        slam/maps                                        ugv_nav4d (unchanged)
   MLSMapSloped (patches:                                        ▲
   height, extent, normal)                                       │ TravGenNode graph
          │                                                      │
          ▼                                                      │
 ┌─────────────────┐  dirty tiles  ┌──────────────────┐   ┌──────┴───────────┐
 │ L0  Ingest &    │──────────────▶│ L1  TerrainField │──▶│ L4  TravMap      │
 │     TileStore   │               │  (ground layers) │   │     Adapter      │
 └─────────────────┘               └────────┬─────────┘   └──────┬───────────┘
                                            │                    │ point queries
                                            ▼                    ▼
                                   ┌──────────────────┐   ┌──────────────┐
                                   │ L2  Structure    │──▶│ L3  Robot    │◀── RobotModel
                                   │  ESDF / overhead │   │     Lens     │    (config)
                                   └──────────────────┘   └──────────────┘
```

- **L0 Ingest:** wraps the incoming MLS, maintains a dirty-tile set, publishes immutable
  snapshots (shadow-copy + atomic `shared_ptr` swap).
- **L1 Terrain estimation:** patch triage (SUPPORT vs STRUCTURE), layer building by
  region-growing on height continuity, robust weighted per-cell plane fit. Output per
  (cell, layer): `height, heightSigma, normal, roughness, confidence`.
- **L2 Structure field:** per-layer blocking mask (vertical structure in the robot height
  band, step edges from *robust* heights, overhead intrusion), exact 2D ESDF
  (Felzenszwalb), per-tile obstacle-point buckets for exact interval queries.
- **L3 Robot lens:** stateless. Capsule-vs-ESDF clearance; **exact feasible-heading
  intervals** (law-of-cosines blocked arcs, unioned, complemented); attitude (incline)
  intervals; continuous cost blend.
- **L4 Adapter:** materializes the legacy `TravGenNode` graph from L1–L3. Classes derived:
  no feasible θ → `OBSTACLE`; restricted θ-set → `PARTIALLY_TRAVERSABLE`; full circle →
  `TRAVERSABLE`; low confidence → `UNKNOWN`; known-next-to-unknown → `FRONTIER`.

## 3. Where accuracy comes from (vs the legacy pipeline)

| Legacy failure | Root cause | TerrainField fix |
|---|---|---|
| Wall patches pull RANSAC plane → wrong slope/height | geometry+semantics mixed in one fit | L1 triage excludes STRUCTURE patches from ground fit *by construction* |
| False obstacles from single noisy patches | max-difference step test | L2 step edges from robust (weighted, outlier-shed) heights |
| Corridor cells falsely blocked / unsafely allowed | 15° yaw sampling padded into wedges | L3 closed-form feasible-θ intervals — exact |
| Binary flicker at slope/step thresholds | hard classification | continuous cost; hard cutoff only at true no-go |
| Stacked-patch / wrong-layer selection | nearest-z layer choice | connectivity-based layer region-growing |
| Empty map when the seed is an obstacle; serial flood | seed-coupled generation | tile-parallel full-map generation |

## 4. Module map (files)

```
src/terrain_field/Types.hpp              shared data model (TerrainCell, StructureCell,
                                         PatchSample, RobotModel, params)        [M0]
src/terrain_field/AngleIntervalSet.hpp/.cpp  circular interval arithmetic        [M0]
src/terrain_field/Esdf.hpp/.cpp          exact 2D Euclidean distance transform   [M1]
src/terrain_field/HeadingIntervals.hpp/.cpp  exact feasible-heading intervals    [M2]
src/terrain_field/TerrainEstimation.hpp/.cpp triage + layers + robust fit        [M3]
src/terrain_field/AttitudeIntervals.hpp/.cpp incline-limit intervals + Cost      [M4]
src/terrain_field/TerrainField.hpp/.cpp  orchestrator L1+L2 over a grid          [M5]
src/TerrainFieldGenerator.hpp/.cpp       L4 adapter → TravGenNode graph          [M6]
test/terrain_field/test_*.cpp            standalone tests (g++ + Eigen only)
```

All modules M0–M4 are dependency-light (std + Eigen), buildable and testable standalone:

```
g++ -O2 -std=c++14 -I src -I /usr/include/eigen3 test/terrain_field/test_X.cpp \
    src/terrain_field/X.cpp [deps...] -o /tmp/test_X && /tmp/test_X
```

## 5. Key algorithms

### 5.1 L1 robust per-cell fit
For cell c, layer L: gather SUPPORT patches of L in the 3×3 neighborhood. Weighted LSQ
plane on patch means, weights `w = 1/(σ²_patch + σ0²) · w_spatial` (own cell 1.0,
neighbors 0.6). One robust reweight (Tukey, c = 2.5·MAD of residuals), refit. Outputs
height (plane at cell center), normal (up-oriented), roughness (weighted RMS residual),
`confidence = min(1, n/5) · exp(−roughness / maxStepHeight)`. Cells with neighborhood
support but no own patch: `INTERPOLATED` (bounded to 1 cell).

### 5.2 L1 patch triage & layers
STRUCTURE if `|normal.z| < structureNormalZ` (default 0.5) **or** patch thickness
`> 2·maxStepHeight`. Layers: BFS over cells; SUPPORT patches in adjacent cells join a
layer if `|Δh| ≤ maxStepHeight`; same-cell patches never share a layer.

### 5.3 L2 blocking mask & ESDF
Cell blocks its layer if: STRUCTURE occupies `[ground, ground + hBand]` above it
(`hBand` = deployment max robot height — a field config, not the robot's), **or** step to
an adjacent ground cell `> maxStepHeight`, **or** another layer's underside intrudes into
the band (also recorded as `overhead`). ESDF: exact Felzenszwalb O(n) squared-distance
transform per layer, tile-parallel.

### 5.4 L3 exact feasible headings (replaces computeSafeOrientations)
Robot = disks at signed offsets `d_k` along the body axis, radius `r = sizeY/2 + margin`.
- `esdf(p) ≥ halfDiagonal` → all headings free (majority of cells — zero extra work).
- `esdf(p) < r` → no heading (blocked at every θ).
- else (**maybe band**): for each disk offset d and obstacle point o with
  `ρ = ‖o−p‖ ∈ (|d|−r, |d|+r)`:
  - if `ρ + |d| ≤ r`: full circle blocked;
  - else blocked arc centered at `φ = atan2(o−p)` (+π if d<0) with half-width
    `Δ = acos((d² + ρ² − r²) / (2|d|ρ))` (clamped).
  Union blocked arcs → complement = **exact** allowed set.

### 5.5 L3 attitude intervals (port of legacy incline limiting)
Same semantics as legacy `computeAllowedOrientations` (slope < min → all; > maxSlope →
none; else wedge of width `interpolate(slope)` about the slope azimuth, mirrored ±π,
`allowForwardDownhill` gating the downhill wedge) — computed analytically, emitted as
intervals. **Final allowed set = clearance ∩ attitude.**

### 5.6 L3 continuous cost
`cost = w_slope·slope + w_rough·roughness + w_clear·max(0, 1 − esdf/costFunctionDist)
      + w_conf·(1 − confidence)` — emitted as the per-node cost the planner already sums.

## 6. Config compatibility

| Legacy param | Fate |
|---|---|
| gridResolution, maxSlope, maxStepHeight | unchanged meaning |
| robotSizeX/Y, robotHeight | move to RobotModel (lens) |
| enableInclineLimitting, inclineLimittingMinSlope/Limit, allowForwardDownhill | attitude intervals (analytic) |
| costFunctionDist | clearance falloff in cost (now always available) |
| minTraversablePercentage | confidence threshold |
| articulatedSuspension | attitude estimator variant (normal vs footprint corners) |
| obstacleInflationMultiplier | **retired** → `safetyMargin` meters on capsule radius |
| numYawSamples | **retired** — intervals are exact |

New: `useTerrainField` (bool, default off until validated) selects the new pipeline.

## 7. Validation strategy

- **Property tests** per module (see TODOs): ESDF exact vs brute force; interval math vs
  brute-force θ sweep at ≤0.02° with tolerance only at interval endpoints; attitude
  intervals vs legacy oracle; L1 wall-pull test (wall patches must not shift ground
  height by more than noise), ramp = one layer, bridge = two layers.
- **Differential harness (P0):** dump legacy vs TerrainField maps cell-by-cell
  (`nodeType/slope/cost/height`) on parking_deck.ply + location maps; every diff must be
  explainable.
- **End-to-end:** ugv_nav4d GUI scenarios with RS final path + goal-shot enabled.

## 8. Decisions & deletion schedule

- The legacy generator is **not deleted yet**: it is the differential baseline and the
  fallback behind `useTerrainField`. Deletion happens at P4 (below) once diffs are
  reviewed: `computePlaneRansac`, `checkStepHeightAABB/OBB` (generation-time use),
  `computeSafeOrientations`, `checkCollisionForYaw`, `inflateObstacles`, and the
  seed-flood `expandAll` path.
- Soil map: folded into L3 cost later (P4); `getSoilMap()` consumers unaffected until then.
- C++14 (project standard); modules are std+Eigen only so they stay unit-testable without
  ROCK.

---

## TODO roadmap

### M0 — foundations (in progress)
- [x] Architecture document (this file)
- [x] `terrain_field/Types.hpp` — shared data model
- [x] `terrain_field/AngleIntervalSet` — circular interval arithmetic + brute-force test

### M1–M4 — core modules (parallel build + adversarial verify)
- [x] M1 `Esdf` — exact Felzenszwalb 2D EDT. Verified: exact equality vs brute force on
      300 random grids + exhaustive enumeration of ALL masks for every shape with
      w·h ≤ 16 (~1M grids), strips to 401, 257×131 float-exactness stress. 0 mismatches.
- [x] M2 `HeadingIntervals` — law-of-cosines blocked arcs. Verified: 9.4M brute-force
      membership checks (0.02° sweeps) incl. exact annulus boundaries, mirror/periodicity
      properties. 0 mismatches. Convention pinned: measure-zero tangent contacts block.
- [x] M3 `TerrainEstimation` — triage + layer region-growing + robust Tukey fit.
      Verified: wall-pull err 0.009 m (naive 0.331 m), ramp = 1 layer exact, bridge = 2
      layers, outlier isolation, determinism under patch-order permutation, INTERPOLATED
      bounded to 1 cell (needs ≥ 2 carriers). Layer ids capped at 65534 (documented).
- [x] M4 `AttitudeIntervals` + `Cost` — analytic port of legacy incline limiting.
      Verified: 1.6M membership checks vs an independently rewritten legacy oracle
      (1e-7 boundary guard), NaN-safety at vertical/degenerate normals, cost
      monotonicity. Note: enableInclineLimitting=false → full circle (the legacy
      slope>maxSlope obstacle check is applied by the pipeline classification, M6).

### M5 — orchestrator
- [x] `TerrainField` class over L1+L2 (per-layer blocking mask / ESDF / overhead,
      obstacle collection for the maybe band). Cross-module integration test
      `test/terrain_field/test_terrain_field.cpp` PASSES (exact ESDF values, wall-pull
      end-to-end, near-wall partial headings: parallel allowed / toward-wall blocked).
      Tiling/incremental snapshots deferred to a later phase (full-map recompute per
      setMLSGrid for now).
- [x] MLS ingest (`PatchSample` extraction) — implemented in the pipeline (M6), bins
      MLS patches into trav-grid cells by world position; variance proxied from patch
      thickness.

### M6 — adapter + wiring (P1–P3 of migration)
- [x] Integrated as `TraversabilityGenerator3d::expandAllTerrainField()`
      (TerrainFieldPipeline.cpp) instead of a sibling class — reuses trMap plumbing,
      so all consumers (GUI, planner, dumps) work unchanged. All public `expandAll`
      overloads branch on `config.useTerrainField`; `setMLSGrid` invalidates.
      Emits the full `TravGenNode` graph: plane/slope/slopeDirection(+atan2)/dense ids/
      allowedOrientations (exact intervals)/nodeType/cost, 8-neighborhood connections
      (|Δh| ≤ maxStepHeight), FRONTIER marking, map-border clearance (virtual border
      obstacles in the heading intervals + border distance in the inflation test).
      Syntax-checked against the installed maps/base/PCL headers.
- [x] `useTerrainField` flag in `TraversabilityConfig` (default false); ugv_nav4d
      `ConfigLoader` reads it; both `parameters.yaml` files set it (currently true).
- [x] travgen3d debug GUI upgraded: "Use TerrainField Pipeline" checkbox (live A/B
      toggle; config change invalidates the generated map via setConfig/clearTrMap),
      yaml load/sync, start-pose requirement waived for the seed-independent pipeline.
- [x] CMake: terrain_field sources added to rock_library; standalone tests unaffected.

### Multi-level fix (2026-07-08)
Parking-deck testing showed lower decks under upper decks turning to obstacle.
Reproduced offline (`test/terrain_field/repro_multilevel.cpp`):
- **Layer decomposition is correct** — flat levels stay single ids; ramps merge with
  their floor into one drivable id; vertically-overlapping levels get distinct ids. No
  fragmentation (an earlier suspicion was a too-loose height tolerance in the probe).
- **Root cause: overhead block band was too tall.** It was
  `(ground+maxStepHeight, ground+maxStepHeight+robotHeight)`, so a ceiling at
  `robotHeight < clearance < robotHeight+maxStepHeight` wrongly blocked the whole floor
  beneath — every lower deck with < ~2.45 m clearance was wiped.
- **Fix:** band is now `(ground+maxStepHeight, ground+robotHeight)` — a ceiling clears
  iff it is higher than the robot body. `TerrainField::compute` params renamed to
  `climbClearance`/`bodyHeight` to make the semantics unambiguous; empty-band guard
  added. Repro: clearances 2.2/2.4/3.0 m now clear a 2.0 m robot; 1.8 m still blocks.
  All module + integration tests still pass.
- **Tuning note:** `robotHeight` must be the true robot body height — a ceiling must be
  higher than it to be drivable underneath.

### Known open — floating phantom surfaces (unresolved)
Elevated near-horizontal surfaces (railing tops, car roofs, the underside of a deck
above) pass the SUPPORT triage and become "floor" fragments disconnected from any real
deck — they show up as red/blue patches floating in the air. The legacy flood hid these
(it only grew from a ground seed); seed-independent generation keeps them. A
connected-component pruning approach was prototyped and then reverted; not yet solved.

### P0/P4 — validation & retirement (open)
- [ ] Floating phantom surfaces (railings / car tops / deck undersides) — see above.
- [ ] Stairs: a staircase has sub-`maxStepHeight` steps, so treads classify traversable
      and risers obstacle → striping, and the base may disconnect from the lower floor.
      A 6×3 m robot cannot use stairs — needs macro-slope / step-pattern detection.
- [ ] Rebuild library + ugv_nav4d (user does builds), load a location map in the GUI,
      compare TerrainField vs legacy maps visually (flip `useTerrainField`).
- [ ] Differential dump tool legacy-vs-new + run on parking_deck.ply and location maps
- [ ] End-to-end GUI validation with RS + goal-shot
- [ ] Cost scaling review: node cost = round(100 · terrainCost) — validate magnitude
      against motion base costs in real plans
- [ ] Tiling + dirty-region incremental updates in `TerrainField`
- [ ] Delete legacy internals per §8; retire `numYawSamples`, `obstacleInflationMultiplier`
