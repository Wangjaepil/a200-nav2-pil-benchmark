# S5 Dynamic Obstacle Diversity v2

## What changed

The base `benchmark.sdf` does **not** need to change. It stays a neutral 200 m x 200 m world with the Contact system enabled. `run_case.py` injects each case's dynamic models into the active world.

S5 v2 extends the dynamic-motion schema while keeping the old one-segment format compatible.

Old/simple motion still works:

```yaml
motion:
  end: {x: ..., y: ..., yaw: ...}
  speed_mps: 0.7
  trigger: {type: robot_distance, distance_m: 3.75}
```

New multi-stage motion:

```yaml
motion:
  speed_mps: 0.6
  trigger:
    type: robot_distance
    distance_m: 5.0
  waypoints:
    - {x: ..., y: ..., yaw: ..., speed_mps: 0.6, hold_sec: 2.0}
    - {x: ..., y: ..., yaw: ..., speed_mps: 0.8}
```

`hold_sec` makes the obstacle stop at that waypoint before continuing. A later waypoint can go back toward the start, enabling return / retreat behaviors. The controller follows every waypoint deterministically, so the benchmark is reproducible.

## Files to replace

Copy these to `~/nav_benchmark/scripts/`:

- `benchmark_dynamic.py`
- `dynamic_obstacle_controller.py`
- `generate_s5_cases.py`
- `test_s5_v2.py` (new, focused QA only)

No Pi file, Nav2 YAML, predictive-safety C++ file, BT XML, or `benchmark.sdf` change is required for this S5 scenario expansion.

## Regenerate the 20 S5 YAMLs

Back up the existing directory first if desired, then:

```bash
python3 ~/nav_benchmark/scripts/generate_s5_cases.py \
  --output ~/nav_benchmark/cases/S5 \
  --force
```

The generated files are also included in `cases/S5/` in this bundle, so you can copy them directly instead of running the generator.

## QA

```bash
python3 -m py_compile \
  ~/nav_benchmark/scripts/benchmark_dynamic.py \
  ~/nav_benchmark/scripts/dynamic_obstacle_controller.py \
  ~/nav_benchmark/scripts/generate_s5_cases.py \
  ~/nav_benchmark/scripts/test_s5_v2.py

python3 ~/nav_benchmark/scripts/test_s5_v2.py
```

Expected final line:

```text
S5_V2_QA_OK cases=20 complex_actors=13 box_actors=5 multi_actor_cases=3
```

The older `test_benchmark_suite.py` has one S5 test that assumes every obstacle is a single start-to-end segment. If you still run that full test suite, apply `test_benchmark_suite_s5_v2.patch` or update that test to iterate `spec.motion_points`.

## Scenario matrix

| Case | Main behavior | Actor composition |
|---|---|---|
| S5_01 | Baseline perpendicular crossing, unchanged regression anchor | 1 medium cylinder |
| S5_02 | Reverse-side, slow small pedestrian | 1 small cylinder |
| S5_03 | Fast large crossing | 1 large cylinder |
| S5_04 | Very slow crossing, early trigger | 1 small cylinder |
| S5_05 | Oblique crossing | 1 cylinder |
| S5_06 | Merge from side -> travel along robot route | 1 cylinder, 3-stage path |
| S5_07 | Merge -> head-on approach toward robot | 1 cylinder, 2-stage path |
| S5_08 | Merge -> slow same-direction blocker ahead | 1 box/cart, 2-stage path |
| S5_09 | Enter route -> stop and remain there | 1 cylinder |
| S5_10 | Enter route -> stop 2 s -> resume crossing | 1 cylinder, hold/resume |
| S5_11 | Cross route -> short hold -> reverse back | 1 cylinder, reversal |
| S5_12 | Approach route -> hesitate -> retreat | 1 cylinder, hold/retreat |
| S5_13 | Zigzag across route multiple times | 1 cylinder, 3-stage zigzag |
| S5_14 | Seeded random-like wandering | 1 small cylinder, 4-stage deterministic random path |
| S5_15 | Large moving cart full crossing | 1 large box |
| S5_16 | Wide cart merges -> blocks / moves along route | 1 very wide box, 2-stage path |
| S5_17 | Two sequential crossings with different speed/size | 2 cylinders |
| S5_18 | Head-on actor + lateral crosser | 2 cylinders |
| S5_19 | Cart enters route -> stops -> returns to origin side | 1 box, stop/return |
| S5_20 | Mixed high-complexity traffic | zigzag cylinder + merging box + fast crosser |

## Design notes

- "Random" is intentionally seeded/deterministic. Truly runtime-random motion would make A/B benchmark comparison and reproduction weaker.
- Dynamic shapes remain `cylinder` and `box` because the existing S5 dynamic SDF/contact pipeline intentionally supports those two collision shapes.
- S5 remains dynamic-only (`obstacles: []`). Static-obstacle geometry stays covered by S1-S4, so S5 measures dynamic response without mixing two benchmark objectives.
- Every generated dynamic path is statically checked to intersect the robot's direct route within the configured path tolerance.
- Complex motion durations are kept below 20 s per actor so the controller has a realistic chance to complete and produce valid stimulus evidence before navigation terminates.
