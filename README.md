# Mo3
Mo3: a Modular Mobility Model for future generation wireless networks
This repository includes the implementation of a new rule-based correlated Modular Mobility Model, named Mo3, that provides accuracy and flexibility on par with behavioral mobility models, while preserving the intuitiveness of popular reference-based group mobility models. Mo3 defines five rules: 1) Individual Mobility, 2) Correlated Mobility, 3) Collision Avoidance, 4) Obstacle Avoidance and 5) Upper Bounds Enforcement, that provide a set of features on par with or beyond the current state of the art. Mo3 adopts a new approach in the definition of rules, that adjusts speed and direction of nodes without introducing acceleration vectors as in behavioral models, and avoids thus the associated complexity issue. Rules are furthermore mapped one-to-one on five modules, that can be independently activated as well as replaced, opening the way to future extensions and modifications.
Mo3 supports furthermore tridimensional mobility for selected rules, in particular Individual Mobility, Correlated Mobility and Upper Bounds Enforcement, supporting thus mobility modeling for agents moving in 3D spaces, such as swarms of UAV, flocks of birds, and baks of fish.
The repository includes a full Matlab implementation and an implementation in OMNeT++ 5.5 of a subset of the modules.
The Matlab implementation includes a main script to setup the model and a set of supporting functions implementing each of the 5 modules defined above.
The OMneT++ implementation includes the the Individual Mobility, Correlated Mobility, Collision Avoidance and Upper Bound Enforcement Mo3 modules in the bidimensional case, as well as supporting functions to collect the simulation data.
(c) Luca De Nardis, 2022

When using this software in scientific publications, please refer to the following work:

L. De Nardis and M. G. Di Benedetto, "Mo3: a Modular Mobility Model for future generation mobile wireless networks," IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
