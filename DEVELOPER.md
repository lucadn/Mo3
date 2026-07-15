# Mo3D — Developer Documentation

This document describes the internal architecture of Mo3D for contributors who want to understand, extend, or debug the codebase. For installation and basic usage, see the top-level [`README.md`](../README.md).

## Architecture overview

```
Mo3D_mobility_main.py        <- entry point / simulation driver
Mo3D_params.yaml              <- configuration (read by utils/read_data.py)
│
├── mobility/
│   ├── node.py                 Node            — per-agent state (id, x/y/z)
│   ├── mobilityUtils.py         instantiate_nodes, load/save obstacles,
│                                distribute_nodes_outside_obstacles,
│                                get_obstacles_from_layout
│   └── mobility.py              Mobility        — the simulation engine
│
└── utils/
    ├── read_data.py              read_params     — YAML loader
    ├── scenarioGeneration.py     Machine, scenarioMachineDistribution
    │                             — auto-generated regular factory layouts
    └── plot_data.py              plot_curves, plot_3d_layout,
                                  plot_obstacle_avoidance — visualization
```

The main script wires these together: it reads the config, builds the node list and obstacle list, constructs a `Mobility` instance, and then repeatedly calls `Mobility.update_mobility_vectors()` in a loop, once per simulated timestep.

## The `Mobility` class (`mobility/mobility.py`)

`Mobility` is the core simulation engine. It holds per-node state as parallel NumPy arrays (`x`, `y`, `z`, `v`, `theta`, `phi`, one entry per node) rather than one object per node, for performance. `Node` objects are used only for initial placement and for reading back final coordinates.

### The five rules

Each of the five Mo3 rules is implemented as one (or a small group of) methods, each gated by its own update period and, except for Individual Mobility, its own enable/disable flag from `Mo3D_params.yaml`:

| Rule | Method(s) | Flag | Period param |
|---|---|---|---|
| 1. Individual Mobility | `individual_mobility()` → `boundless()` | always on | `T_IM` |
| 2. Correlated Mobility | `correlated_mobility()`, `load_binding_matrix()` | `correlatedMobilityFlag` | `T_CM` |
| 3. Collision Avoidance | `collision_avoidance()` (3D), `collision_avoidance_2D()` (legacy 2D path) | `collisionAvoidanceFlag` | `T_CA` |
| 4. Obstacle Avoidance | `obstacle_avoidance()`, plus geometry helpers (`rectangle_min_distance`, `rectangle_forbidden_range`, `ellipse_min_distance`, `ellipse_forbidden_range`, `find_tangent_directions`) | `obstacleAvoidanceFlag` | `T_OA` |
| 5. Upper Bounds Enforcement | `upper_bounds_enforcement(v0, theta0, phi0)` | `upperBoundsEnforcementFlag` | `T_UB` (fixed to `dt`; do not change) |

`individual_mobility()` currently only supports one model, `"Boundless"` (implemented in `boundless()`); any other value in `individual_mobility_model` triggers a hard `sys.exit`. This is the natural place to plug in an alternative individual-mobility model — see "Extending the model" below.

### The update loop: `update_mobility_vectors(t, nodeList)`

Called once per timestep from the main script. On each call it:

1. Recomputes the pairwise distance matrix `dMatrix` for all nodes.
2. Snapshots the current speed/direction (`v0`, `theta0`, `phi0`).
3. Applies rules 1–4 in order, each only if `t` aligns with that rule's update period (checked via `math.remainder(t, T) < 1e-10`) and, for rules 2–4, only if the corresponding flag is enabled.
4. Applies rule 5 (`upper_bounds_enforcement`) unconditionally if enabled, using the pre-update snapshot to clamp any violations introduced by the previous steps.
5. Integrates the resulting speed/direction into new `x`, `y`, `z` positions, and appends them to `xPath`/`yPath`/`zPath` if `pathSave` is enabled.

Because rules run in a fixed sequence within a single timestep, a later rule can override an earlier one's output for that step — e.g., Obstacle Avoidance can override a heading chosen by Correlated Mobility, and Upper Bounds Enforcement can clamp anything that would otherwise exceed configured limits.

### Correlated Mobility grouping strategies

`correlated_mobility()` supports two strategies for choosing a node's target when it must move to stay grouped (`groupingStrategy` in the YAML):

- `1` — move toward the centroid of group mates (an extension beyond the original paper).
- `2` — move toward the closest group mate not already in the connected set (the strategy described in the referenced paper).

### Obstacle geometry

Two obstacle-avoidance code paths exist:

- Rectangle-based (`rectangle_min_distance`, `rectangle_forbidden_range`) — used for the 2D obstacle model.
- Ellipse-based (`ellipse_min_distance`, `ellipse_forbidden_range`, `find_tangent_directions`) — used for 3D obstacle avoidance.

Both compute a "forbidden range" of headings that would bring the node too close to an obstacle, which `obstacle_avoidance()` then merges across obstacles and uses to redirect the node.

## Supporting modules

### `mobility/node.py` — `Node`

A minimal container: `node_id`, `x`/`y`/`z` coordinates, with getters/setters. Positions are authoritative in `Mobility`'s internal arrays during the simulation; `Node` objects are updated at initialization and read back at the end of the run in the main script.

### `mobility/mobilityUtils.py`

- `instantiate_nodes(n)` — builds a list of `Node` objects with sequential IDs.
- `load_obstacles(path)` / `save_obstacles(path, list)` — read/write the obstacle-list text format (`Obstacle <id> <x> <y> <z> <dx> <dy> <dz>` per line).
- `get_obstacles_from_layout(machine_array)` — converts a list of `Machine` objects (from `scenarioGeneration.py`) into the same obstacle-list representation, so auto-generated and file-based scenarios feed the rest of the pipeline identically.
- `distribute_nodes_outside_obstacles(...)` — rejection-sampling placement of nodes so none start inside an obstacle (currently supports `"Uniform"` distribution only; up to 1000 attempts per node before failing).

### `utils/scenarioGeneration.py` — `Machine`, `scenarioMachineDistribution`

`Machine` represents a single rectangular obstacle (e.g., a factory machine) with a center, size, and derived bounding box. `scenarioMachineDistribution(...)` tiles machines across the configured area on a regular grid, given a machine size and inter-machine spacing — this is what runs when `scenario.scenarioGeneration: true`.

### `utils/read_data.py` — `read_params`

Thin wrapper around `yaml.load(..., Loader=yaml.FullLoader)`, returning a nested dict mirroring `Mo3D_params.yaml`'s structure.

### `utils/plot_data.py`

- `plot_curves(...)` — generic 2D line-plot helper (Matplotlib), not currently called from the main pipeline; available for post-processing scripts.
- `plot_3d_layout(m)` — takes a `Mobility` instance and renders obstacles plus node trajectories in an interactive 3D Plotly figure. This is what the main script calls at the end of a run.
- `plot_obstacle_avoidance(...)` — 2D diagnostic plot of a single obstacle-avoidance heading adjustment; useful when debugging the Obstacle Avoidance module.

## Extending the model

- **New individual-mobility model:** add a new method to `Mobility` (parallel to `boundless()`), and add a branch in `individual_mobility()` keyed on `self.type`.
- **New grouping strategy:** add a new branch to the `groupingStrategy` check inside `correlated_mobility()`.
- **New obstacle shape:** add a `..._min_distance` / `..._forbidden_range` pair following the existing rectangle/ellipse pattern, and dispatch to it from `obstacle_avoidance()`.
- **New node-distribution statistic:** add a branch to `distribute_nodes_outside_obstacles()` in `mobilityUtils.py` (currently `"Uniform"` is the only supported value; anything else calls `sys.exit`).

## Known limitations / housekeeping notes

- `collision_avoidance_2D()` is a legacy 2D-only path kept alongside the 3D `collision_avoidance()`; it is not currently invoked from `update_mobility_vectors()` and may be a candidate for removal or for gating behind a config flag.
- `mobility.py` imports `DataFrame` from `pandas.core.frame` and `node.py` imports `pandas`, `operator`, neither of which are currently used in either file — safe to remove, or a sign of planned-but-unimplemented functionality.
- `Mo3D_mobility_main.py` appends the parent and grandparent of the current working directory to `sys.path`; this is only needed if you intend to run the script from outside the project root, which is not the documented/supported usage in this repo.
- The `simulation.tot_simulation_time_s` key seen in some older configs is unused by the current main script, which instead reads `simulation.nMaxUpdates` directly.

## Testing changes

There is currently no automated test suite. Before submitting changes, at minimum re-run the default scenario end-to-end:

```bash
python Mo3D_mobility_main.py
```

and confirm it completes without errors and produces a plausible 3D trajectory plot for the configured number of nodes.
