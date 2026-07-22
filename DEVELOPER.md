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
    ├── generate_binding_matrix.py generate_binding_matrix
    │                             — optionally auto-writes the binding-matrix
    │                               file from a declarative switch sequence
    │                               (see "Binding matrix auto-generation" below)
    └── plot_data.py              plot_curves, plot_3d_layout,
                                  plot_obstacle_avoidance — visualization
```

The main script wires these together: it reads the config, optionally auto-generates the binding-matrix file, builds the node list and obstacle list, constructs a `Mobility` instance, and then repeatedly calls `Mobility.update_mobility_vectors()` in a loop, once per simulated timestep.

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

### Binding matrix auto-generation

The binding matrix used by Correlated Mobility can either be authored by hand as a text file (see `mobility.BMfilename` in the YAML, and the format notes above `load_binding_matrix`), or generated automatically at the start of each run from a declarative sequence in `Mo3D_params.yaml`, under `mobility.binding_matrix`:

```yaml
mobility:
  BMfilename: "Mo3D_BindingMatrix.txt"
  binding_matrix:
    auto_generate: true
    switches:
      - type: all_together   # every node bound to every other node (all-ones matrix)
        nextSwitch: 4         # simulation time (s) at which the next entry takes effect
      - type: independent     # no node bound to any other (identity matrix)
        nextSwitch: 6
      - type: all_together
        # no nextSwitch on the last entry: it remains in force for the rest of the run
```

If `auto_generate` is `true`, `Mo3D_mobility_main.py` calls `generate_binding_matrix()` (`utils/generate_binding_matrix.py`) once at startup, which **overwrites** the file at `mobility.BMfilename` with a matrix (and `nextSwitch` lines) built from the `switches` list, before the normal `load_binding_matrix()` file-based loading path runs unchanged. Only two `type` values are currently supported (`all_together`, `independent`); anything else raises a `ValueError`. If `auto_generate` is `false`, the existing `BMfilename` file is used as-is and `switches` is ignored.

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
- `load_obstacles(path)` / `save_obstacles(path, list)` — read/write the obstacle-list text format. Each line is `Obstacle <type> <x_obs> <y_obs> <z_obs> <p1> <p2> <p3>`, where `<type>` is `1` (parallelepiped) or `2` (elliptic cylinder). The meaning of `<p1> <p2> <p3>` depends on `<type>`: for a parallelepiped they are the full sizes `(Δx, Δy, Δz)`; for an elliptic cylinder they are the semi-axes `(a, b)` of the base plus the full vertical extent `Δz`. `z_obs`/`Δz` jointly determine the vertical segment the obstacle occupies (floor, ceiling, or full-height, per the three configurations described in the paper).
- `get_obstacles_from_layout(machine_array)` — converts a list of `Machine` objects (from `scenarioGeneration.py`) into the same obstacle-list representation, so auto-generated and file-based scenarios feed the rest of the pipeline identically.
- `distribute_nodes_outside_obstacles(...)` — rejection-sampling placement of nodes so none start inside an obstacle, and (optionally) no closer than `min_inter_node_distance` to any other already-placed node (the main script passes `d_CA_min` here, so initial placement can't itself start two nodes within the Collision Avoidance safety margin). Currently supports `"Uniform"` distribution only; up to 1000 attempts per node before failing.

### `utils/scenarioGeneration.py` — `Machine`, `scenarioMachineDistribution`

`Machine` represents a single rectangular obstacle (e.g., a factory machine) with a center, size, and derived bounding box. `scenarioMachineDistribution(...)` tiles machines across the configured area on a regular grid, given a machine size and inter-machine spacing — this is what runs when `scenario.scenarioGeneration: true`.

### `utils/read_data.py` — `read_params`

Thin wrapper around `yaml.load(..., Loader=yaml.FullLoader)`, returning a nested dict mirroring `Mo3D_params.yaml`'s structure.

### `utils/plot_data.py`

- `plot_curves(...)` — generic 2D line-plot helper (Matplotlib), not currently called from the main pipeline; available for post-processing scripts.
- `plot_3d_layout(m, trajectory_path=None, layout_path=None)` — takes a `Mobility` instance and writes two separate 3D Plotly figures as standalone HTML files: `layout_path` (obstacles only) and `trajectory_path` (obstacles plus every node's trajectory). If either path is left as `None`, it defaults to `/results/...` when `/results` exists and is writable (the Code Ocean convention), otherwise to a local `results/` folder relative to the current working directory. This is what the main script calls at the end of a run; it no longer opens an interactive window by default.
- `plot_obstacle_avoidance(...)` — 2D diagnostic plot of a single obstacle-avoidance heading adjustment; useful when debugging the Obstacle Avoidance module.

## Extending the model

- **New individual-mobility model:** add a new method to `Mobility` (parallel to `boundless()`), and add a branch in `individual_mobility()` keyed on `self.type`.
- **New grouping strategy:** add a new branch to the `groupingStrategy` check inside `correlated_mobility()`.
- **New obstacle shape:** add a `..._min_distance` / `..._forbidden_range` pair following the existing rectangle/ellipse pattern, and dispatch to it from `obstacle_avoidance()`.
- **New node-distribution statistic:** add a branch to `distribute_nodes_outside_obstacles()` in `mobilityUtils.py` (currently `"Uniform"` is the only supported value; anything else calls `sys.exit`).

## Known limitations / housekeeping notes

- `collision_avoidance_2D()` is a legacy 2D-only path kept alongside the 3D `collision_avoidance()`; it is not currently invoked from `update_mobility_vectors()` and may be a candidate for removal or for gating behind a config flag.
- `mobility.py` imports `DataFrame` from `pandas.core.frame`, `matplotlib.pyplot`, and `pdb`, and `node.py` imports `pandas`, `operator`; none of these are currently used in either file — safe to remove, or a sign of planned-but-unimplemented functionality.
- `Mo3D_mobility_main.py` appends the parent and grandparent of the current working directory to `sys.path`; this is only needed if you intend to run the script from outside the project root, which is not the documented/supported usage in this repo.
- The `simulation.tot_simulation_time_s` key seen in some older configs is unused by the current main script, which instead reads `simulation.nMaxUpdates` directly.

## Testing changes

There is currently no automated test suite. Before submitting changes, at minimum re-run the default scenario end-to-end:

```bash
python Mo3D_mobility_main.py
```

and confirm it completes without errors and produces plausible 3D layout and trajectory HTML files for the configured number of nodes.