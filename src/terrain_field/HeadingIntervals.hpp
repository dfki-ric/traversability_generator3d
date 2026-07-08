#pragma once

// Exact feasible-heading intervals for a disk-capsule robot against point obstacles.
// Part of the TerrainField robot lens (see TERRAIN_FIELD_ARCHITECTURE.md §5.4).
//
// The robot footprint is modeled as disks of a common radius placed at signed offsets
// along the body axis. For each (disk offset, obstacle point) pair the headings at
// which that disk overlaps the obstacle form a single arc, obtained in closed form via
// the law of cosines. The union of all blocked arcs is complemented to yield the exact
// allowed heading set — no yaw sampling, no obstacle inflation.

#include "AngleIntervalSet.hpp"

#include <Eigen/Core>
#include <vector>

namespace traversability_generator3d
{
namespace terrain_field
{

/** Headings theta for which NO disk of the capsule collides with any obstacle point.
 *  The robot at heading theta places disk centers at p + d_k * (cos theta, sin theta)
 *  for each signed offset d_k; a disk collides iff some obstacle point is within
 *  distance radius of its center. Returns the exact allowed set. */
AngleIntervalSet feasibleHeadings(const Eigen::Vector2d& p,
                                  const std::vector<double>& diskOffsets,
                                  double radius,
                                  const std::vector<Eigen::Vector2d>& obstacles);

}
}
