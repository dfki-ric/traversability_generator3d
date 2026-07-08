#pragma once

// Continuous terrain cost blend (TERRAIN_FIELD_ARCHITECTURE.md §5.6):
//
//   cost = wSlope * slope + wRough * roughness
//        + wClear * max(0, 1 - esdf / costFunctionDist)
//        + wConf  * (1 - confidence)
//
// The clearance term is disabled when costFunctionDist <= 0. Header-only.

#include "Types.hpp"

#include <algorithm>
#include <cmath>

namespace traversability_generator3d
{
namespace terrain_field
{

/** Continuous cost blend per doc §5.6; returns >= 0. */
inline double terrainCost(const TerrainCell& cell, float esdfMeters, const CostWeights& w)
{
    const double nz = std::min(1.0, std::max(-1.0, static_cast<double>(cell.normal.z())));
    const double slope = std::acos(nz);

    double cost = w.wSlope * slope
                + w.wRough * static_cast<double>(cell.roughness)
                + w.wConf * (1.0 - static_cast<double>(cell.confidence));

    if (w.costFunctionDist > 0.0)
        cost += w.wClear * std::max(0.0, 1.0 - static_cast<double>(esdfMeters)
                                              / w.costFunctionDist);

    return std::max(0.0, cost);
}

}
}
