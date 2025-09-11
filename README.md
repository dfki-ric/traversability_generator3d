# traversability_generator3d

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

A C++ library for **3D traversability estimation from Multi-Level Surface (MLS) maps**, developed at the [DFKI Robotics Innovation Center](https://robotik.dfki-bremen.de/en/startpage.html).

The library takes an MLS map as input and produces a `TraversabilityMap3d` enriched with slope, step height, orientation constraints, and soil semantics. This enables safe and robot-specific navigation in unstructured outdoor environments.

## Features

- Input: **Multi-Level Surface (MLS) maps**  
- Plane fitting with **PCL RANSAC**  
- Slope and step height checks using **AABB/OBB** methods with **CGAL**
- Orientation-dependent motion constraints (incline limiting)  
- Robot footprint-aware **obstacle and frontier inflation**  
- **Soil-aware traversability** (sand, gravel, rocks, concrete) with uncertainty propagation  
- ROS/ROS2 compatibility and RViz visualization support  

## Dependencies

- [Eigen3](https://eigen.tuxfamily.org/)  
- [PCL (>=1.7)](https://pointclouds.org/) with segmentation module  
- [CGAL](https://www.cgal.org/) (Core)  
- [Boost.Serialization](https://www.boost.org/)  
- [Rock base-types](https://github.com/rock-core/base-types)  
- [Rock maps](https://github.com/dfki-ric/slam-maps)  
- [vizkit3d_debug_drawings](https://github.com/rock-gui/gui-vizkit3d_debug_drawings.git) (optional, for visualization)  

## Node types

The core output of `traversability_generator3d` is a `TraversabilityMap3d`, where each cell is represented as a node enriched with slope, step height, soil information, etc.  
To support planning and decision-making, nodes are classified into the following categories:

- 🟩 **Traversable**: The robot can stand (with its center) on this patch in at least one orientation without hitting an obstacle.
- 🟥 **Obstacle**: There is no way that the robot can stand (with its center) on this patch.
- 🟦 **Frontier**: Borders to the end of the map. Should be traversable (I am not 100% sure about this. check the code!)
- 🟪 **Unknown**: This is a virtual patch that serves as boundary for algorithms. This patch does not exist in reality. Patches also become unknown if there is not enough support in the MLS to be sure that a patch exists in this location.
- ⬛ **Hole**: This is part of the map specification but is not used by ugv_nav4d. It might be used elsewhere but the planner cannot handle it.
- 🟨 **Unset**: This is the starting state of a new patch. It should not be visible in a fully explored map. If you see a yellow patch after map expansion is done, you have found a bug in the `TraversabilityMapGenerator` and should investigate.
- 🟧 **Inflated Obstacle**: An obstacle patch expanded to include a safety buffer around the actual obstacle. These are not traversable.
- <img width="28" height="28" alt="image" src="https://github.com/user-attachments/assets/775162ac-be4c-4a32-a24b-8ab41f5cea99" style="border: none;" />**Inflated Frontier**: A frontier patch that has been inflated for planning safety margins. Indicates proximity to exploration boundaries.

<img width="1028" height="763" alt="image" src="https://github.com/user-attachments/assets/82e77547-74d2-40ff-9c15-a0d37f7187a1" />

### Why node types matter

This rich classification scheme provides more than just a binary safe/unsafe map. It enables planners to:  
- Prioritize safe traversal while accounting for soil costs.  
- Use **frontier nodes** to guide exploration and map growth.  
- Maintain safety margins by considering **inflated nodes**.  

Together, these node types form the backbone of the `TraversabilityMap3d`, making it both a **planning tool** and a **research framework** for robust traversability estimation.

## Acknowledgements

This library was developed at the [DFKI Robotics Innovation Center](https://robotik.dfki-bremen.de/en/startpage.html) and funded by the **German Federal Ministry of Education and Research (BMBF)**.  

## License

This project is released under the [BSD-3-Clause License](LICENSE).
