# Mo3D

Mo3D is a simulation framework implementing an extended version of Mo3 — a Modular Mobility Model for future generation wireless networks.

Mo3 is a rule-based correlated mobility model that provides accuracy and flexibility on par with behavioral mobility models, while preserving the intuitiveness of popular reference-based group mobility models. Mo3 defines five rules — 1) Individual Mobility, 2) Correlated Mobility, 3) Collision Avoidance, 4) Obstacle Avoidance, and 5) Upper Bounds Enforcement — that provide a set of features on par with or beyond the current state of the art. Mo3 adjusts the speed and direction of nodes without introducing acceleration vectors as in behavioral models, avoiding the associated complexity. Each rule is mapped one-to-one onto an independent module, which can be activated, deactivated, or replaced, enabling future extensions.

While the original formulation of Mo3 provided limited support for tridimensional mobility, the extended version provided in Mo3D enables full mobility modeling for agents moving in 3D spaces, such as swarms of UAVs, flocks of birds, or banks of fish.

## Repository contents

| Path | Description |
|---|---|
| `Mo3D_mobility_main.py` | Main script: loads configuration, builds the scenario, runs the simulation, and (optionally) plots the resulting trajectories. |
| `Mo3D_params.yaml` | Configuration file used to set up the model (scenario, node, and mobility-module parameters). |
| `Mo3D_ObstacleList.txt`, `Mo3D_ObstacleList_BiRex.txt` | Obstacle list inputs for the Obstacle Avoidance module (custom scenario and "Bi-Rex" scenario respectively). |
| `Mo3D_BindingMatrix.txt` | Binding matrix input for the Correlated Mobility module. |
| `mobility/mobility.py` | `Mobility` class: implements the five Mo3D rules and drives the per-step update loop. |
| `mobility/node.py` | `Node` class: position/identity container for a simulated agent. |
| `mobility/mobilityUtils.py` | Helpers to instantiate nodes, load/save obstacle lists, and place nodes outside obstacles. |
| `utils/read_data.py` | Loads `Mo3D_params.yaml` into a Python dictionary. |
| `utils/scenarioGeneration.py` | `Machine` class and automatic factory-floor layout generation (regular obstacle placement). |
| `utils/plot_data.py` | 2D/3D plotting utilities for trajectories and obstacle-avoidance diagnostics. |

See `docs/DEVELOPER.md` for a deeper look at the module architecture and how to extend it.

## Requirements

- Python 3.10+ (developed and tested on Python 3.10)
- Dependencies listed in [`requirements.txt`](requirements.txt): `numpy`, `pandas`, `matplotlib`, `PyYAML`, `scipy`, `plotly`

## Installation

```bash
git clone https://github.com/lucadn/Mo3.git Mo3D
cd Mo3D
python -m venv venv
source venv/bin/activate        # on Windows: venv\Scripts\activate
pip install -r requirements.txt
```

> **Note:** the repository is currently published under the name `Mo3` on GitHub. The code's internal imports (e.g. `from mobility.node import Node`) are relative to the project root, so the folder does not strictly need to be renamed — what matters is that you run the main script **from inside the project root** (see below), so that `mobility/` and `utils/` are importable as top-level packages.

## Usage

Run the simulation from the project root:

```bash
cd Mo3D
python Mo3D_mobility_main.py
```

This will:
1. Read simulation parameters from `Mo3D_params.yaml`.
2. Instantiate the requested number of nodes and place them outside any configured obstacles.
3. Either auto-generate a regular obstacle layout (`scenario.scenarioGeneration: true`) or load one from an obstacle-list file (`scenario.scenarioGeneration: false`).
4. Run the mobility engine for `simulation.nMaxUpdates` update steps, applying the five Mo3D rules according to each module's configured update period and enable/disable flag.
5. Print the start/end coordinates of each node to the console.
6. If `mobility.pathSave: true`, plot the resulting 3D trajectories using Plotly.

### Configuring a run

All simulation behavior is controlled through `Mo3D_params.yaml`, organized into three blocks:

- **`scenario`** — factory/area dimensions, obstacle source (auto-generated or loaded from file).
- **`node`** — number of nodes and their initial spatial distribution.
- **`mobility`** — per-module parameters (update periods, thresholds, speed/turn-rate bounds) and four boolean flags (`correlatedMobilityFlag`, `collisionAvoidanceFlag`, `obstacleAvoidanceFlag`, `upperBoundsEnforcementFlag`) to enable or disable individual rules.
- **`simulation`** — `nMaxUpdates`, the total number of update steps to run.

Two example scenarios are provided as commented-out blocks inside the `scenario` section of the YAML file: a custom layout and a "Bi-Rex" layout (`length`/`width`/`height` plus the matching `OLfilename`). Uncomment the desired block (and comment out the other) to switch between them.

### Expected output

With `pathSave: true`, the framework keeps a full path history (`xPath`, `yPath`, `zPath`) for every node and, at the end of the run, opens an interactive 3D Plotly figure showing:
- the configured obstacles (as gray 3D boxes), and
- each node's trajectory and starting position.

## Citing this work

When using this software in scientific publications, please refer to:

- L. De Nardis and M. G. Di Benedetto, "Mo3: a Modular Mobility Model for future generation mobile wireless networks," *IEEE Access*, Volume 10, April 1, 2022, pp. 34085–34115. DOI: 10.1109/ACCESS.2022.3161541
- D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments," submitted to *SoftwareX*, 2026.

## License

Released under the GNU Affero General Public License v3 (AGPLv3) — see [`LICENSE`](LICENSE).
