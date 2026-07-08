#pragma once

// Attitude (incline-limit) heading intervals — analytic port of the legacy
// TraversabilityGenerator3d::computeAllowedOrientations (see
// TERRAIN_FIELD_ARCHITECTURE.md §5.5).
//
// Semantics (exactly the legacy behavior):
// - enableInclineLimitting == false          -> full circle (no attitude restriction)
// - slope >= maxSlope                        -> empty set (the legacy returns false,
//                                               which the adapter maps to OBSTACLE)
// - slope <  inclineLimittingMinSlope        -> full circle
// - otherwise a wedge of half-width
//     interpolate(slope, (minSlope, pi/2) -> (maxSlope, inclineLimittingLimit))
//   centered on the uphill azimuth (direction of steepest ascent); if
//   allowForwardDownhill the mirrored (downhill-facing) wedge is allowed as well.
//
// The final allowed heading set of a cell is clearance ∩ attitude (§5.4 ∩ §5.5).

#include "AngleIntervalSet.hpp"
#include "Types.hpp"

#include <Eigen/Core>

namespace traversability_generator3d
{
namespace terrain_field
{

/** Allowed headings from terrain attitude. slopeDir = azimuth of steepest ascent
 *  (radians, from the cell normal); slope in radians. */
AngleIntervalSet attitudeHeadings(double slope, double slopeDir, const RobotModel& robot);

/** Convenience overload deriving slope and slopeDir from a z-up unit normal. */
AngleIntervalSet attitudeHeadings(const Eigen::Vector3f& normal, const RobotModel& robot);

}
}
