---
title: 'traversability_generator3d: A C++ library for 3D traversability estimation from MLS maps'
tags:
  - robotics
  - traversability
  - 3D mapping
  - terrain analysis
authors:
  - name: Muhammad Haider Khan Lodhi
    orcid: 0009-0008-1199-3489
    affiliation: 1
affiliations:
  - name: Robotics Innovation Center, German Research Center for Artificial Intelligence (DFKI), Bremen, Germany
    index: 1
date: 2025-09-15
bibliography: paper.bib
---

# Summary

`traversability_generator3d` is a C++ library that converts Multi-Level Surface (MLS) [@triebel2006multilevel] maps into 3D traversability maps. Unlike traditional 2D elevation-based maps, MLS preserves multiple surface hypotheses per cell, making it well-suited for environments with rubble, vegetation, bridges, or overlapping terrain layers. The library builds on the MLS representation to provide robot-specific traversability analysis where each grid cell is enriched with slope, step height, plane fitting results, allowed motion orientations, and optional soil semantics. The result is a `TraversabilityMap3d` suitable for autonomous navigation. The library is designed for real-world robotic applications including construction robotics, disaster response, search-and-rescue, and planetary exploration, where terrain is uneven, uncertain, and multi-layered.

![Comparison between the original Multi-Level Surface (MLS) map (left) and the generated TraversabilityMap3d (right). The MLS encodes the underlying 3D terrain structure, whereas the TraversabilityMap3d highlights navigable and non-navigable regions for path planning.](figures/combined_map.png)

# Statement of need

Safe and efficient navigation requires robots to distinguish where they can move from where they cannot. This is especially critical for outdoor field robots, walking excavators, and planetary rovers that face slopes, steps, loose soil, or debris. Existing approaches often use 2.5D elevation maps [@fankhauser2018probabilistic] that store only one surface per XY cell, losing critical information when multiple layers overlap, voxel/octree maps [@hornung2013octomap] preserve 3D structure but are memory-intensive and less tailored for traversability, and purely semantic approaches [@cramer2018traversability; @wiesmann2021traversability] that classify terrain appearance but often neglect physical constraints like slope or robot footprint.  

`traversability_generator3d` addresses these limitations by:

- Taking MLS as input, retaining multi-level terrain structure.  
- Performing plane fitting and slope analysis per patch, allowing orientation-dependent motion constraints.  
- Checking step height and body collisions with robot-specific geometry (AABB/OBB checks).  
- Supporting soil-aware traversability, where semantic labels (sand, gravel, rocks, concrete) are integrated with uncertainty.  
- Producing a 3D traversability map with rich node types that planners can use directly.   

## Input

The library consumes Multi-Level Surface (MLS) maps, which store multiple surface hypotheses per grid cell. This allows the library to reason about environments with overlapping structures (e.g. bridges, rubble, vegetation), unlike simpler 2.5D elevation maps.

## Processing pipeline

The traversability generation proceeds in several stages:

1. Plane fitting and slope estimation  

   - Local surface patches are extracted from the MLS and fitted with planes using RANSAC (via PCL).
   - Each patch’s slope angle and slope direction are computed from the fitted plane normal.

2. Step height evaluation  

   - Both axis-aligned (AABB) and oriented bounding box (OBB) checks are performed to test whether nearby patches exceed the robot’s configured maximum step height.
   - This ensures that steps or discontinuities larger than the robot’s physical capability are marked as obstacles.

3. Orientation-dependent traversability

   - On steep slopes, the library restricts the allowed orientations of motion (incline limiting).
   - For example, forward uphill/downhill motion may be permitted.

4. Footprint-aware obstacle inflation 

   - Obstacles and frontiers are grown outward by the robot’s footprint radius.
   - This guarantees that planned paths account for the full robot body, not just its center point.

5. Soil-aware traversability

   - In addition to geometry, the library maintains a parallel `TraversabilityMap3d<SoilNode*>` to model soil types.  
   - Soil samples (sand, gravel, rocks, concrete) can be injected with associated uncertainty.
   - A Gaussian propagation mechanism distributes soil probabilities spatially, and costs are updated accordingly.
   - Certain soils may be forbidden entirely (e.g., robots not allowed to enter sand), in which case nodes are automatically converted to obstacles.  

6. Map expansion

   - The library grows the traversability map outward from user-defined start nodes.
   - Unknown areas are classified as `FRONTIER` nodes, guiding exploration.
   - Expansion can be bounded by distance to restrict computation to a region of interest.  

## Output

The result is a `TraversabilityMap3d` structure with two layers:  

- Geometric traversability (`TravGenNode`) containing slope, plane, step height, allowed orientations, cost, and node type.
- Soil traversability (`SoilNode`) containing soil type probabilities and semantic costs.  

# Traversability node types

Each cell in the `TraversabilityMap3d` is represented as a `TravGenNode` with geometric, semantic, and cost attributes.  
Nodes are categorized as:

- `TRAVERSABLE`: safe patches where the robot can move.  
- `OBSTACLE`: unsafe regions due to excessive slope, step height, body collisions, or forbidden soil.  
- `FRONTIER`: expansion boundaries between known and unknown areas.  
- `INFLATED_OBSTACLE`: obstacles grown outward to include robot footprint.  
- `INFLATED_FRONTIER`: expanded frontiers with safety margins.  
- `UNKNOWN`: regions with insufficient or missing MLS data.  
- `HOLE`: gaps where too few patches exist to support safe navigation.  
- `UNSET`: unclassified nodes during initialization.  

![Traversability map with semantic node classifications.](figures/traversability_map_nodes.png)

This classification enables downstream planners to reason not only about safe/unsafe areas but also about uncertainty and exploration frontiers.

# Soil-aware traversability

In addition to geometry, `traversability_generator3d` models soil composition. Each `SoilNode` stores probability distributions over soil types. By default, costs are adapted as follows: 

- `Concrete`: low cost (preferred).  
- `Sand/Gravel`: higher cost (risky terrain).  
- `Rocks`: costly or forbidden depending on configuration.  
- `Unknown soil`: assigned maximum cost.  

![Map layers for terrain assessment. Left: Traversability map showing navigable (green) and obstacle (red) regions. Middle: Soil map with sand (yellow), gravel (green), and unknown (light blue). Right: Fused soil–traversability map, where sand is treated as non-traversable and thus marked as obstacle (red).](figures/soilmap.png)


Through configuration, users can forbid traversal on certain soils, automatically converting affected nodes to obstacles. This enables integration of perceptual information (e.g. from ground-penetrating radar, visual classifiers, or tactile sensors) with geometric terrain reasoning.

# Acknowledgements

The traversability_generator3d library was initiated and is currently developed at the Robotics Innovation Center of the German Research Center for Artificial Intelligence (DFKI) in Bremen, together with the Robotics Group of the University of Bremen. The development was started in the scope of the Entern project (50RA1406), which has been funded by the German Aerospace Center (DLR) with funds from the German Federal Ministry for Economic Affairs and Climate Action (BMWK). 

We would also like to acknowledge the contributors of the traversability_generator3d repository, including the developers visible on the project's GitHub contributors page, for their valuable efforts and dedication. 

# References
