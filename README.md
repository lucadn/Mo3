# Mo3D
Mo3D is a simulation framework implementing an extended version of the Mo3 - Modular Mobility Model for future generation wireless networks. 
Mo3 is a rule-based correlated mobility model that provides accuracy and flexibility on par with behavioral mobility models, while preserving the intuitiveness of popular reference-based group mobility models. Mo3 defines five rules: 1) Individual Mobility, 2) Correlated Mobility, 3) Collision Avoidance, 4) Obstacle Avoidance and 5) Upper Bounds Enforcement, that provide a set of features on par with or beyond the current state of the art. Mo3 adopts an approach in the definition of rules that adjusts speed and direction of nodes without introducing acceleration vectors as in behavioral models, and avoids thus the associated complexity issue. Rules are furthermore mapped one-to-one on five modules, that can be independently activated as well as replaced, opening the way to future extensions and modifications. 
While the original formulation of Mo3 provided limited support for tridimensional mobility, the extended version provided in Mo3D and released in this repository enables full mobility modeling for agents moving in 3D spaces, such as swarms of UAV, flocks of birds, and banks of fish. The repository provides a python implementation that includes:
1) a configuration file “Mo3D_params.yaml” used to setup the model;
2) input txt files defining inputs for the Correlated Mobility and Obstacle Avoidance modules
3) a main script “Mo3D_mobility_main.py” used to start and periodically update the model, and to save the generated mobility patterns to file;
4) a “mobility” folder that includes the “mobility.py” file, containing the mobility class definition and the corresponding methods implementing the five rules of the model, the “node.py” file, containing the node class definition for nodes moving in the simulation, and the “mobilityUtils.py” file that provides utilities to load obstacles from file or from an external area layout;
5) a “utils” folder containing utilities to load settings from the configuration file, plot the mobility patterns, and generate a layout with regular obstacle placement.

When using this software in scientific publications, please refer to the following works:

L. De Nardis and M. G. Di Benedetto, "Mo3: a Modular Mobility Model for future generation mobile wireless networks," IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541

D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments," submitted to Software X, 2026.
