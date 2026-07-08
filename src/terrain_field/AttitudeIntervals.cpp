#include "AttitudeIntervals.hpp"

#include <algorithm>
#include <cmath>

namespace traversability_generator3d
{
namespace terrain_field
{

namespace
{
/** Linear interpolation through (x0, y0) and (x1, y1) — identical to the legacy
 *  TraversabilityGenerator3d::interpolate. */
double interpolate(double x, double x0, double y0, double x1, double y1)
{
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}
}

AngleIntervalSet attitudeHeadings(double slope, double slopeDir, const RobotModel& robot)
{
    // Legacy: computeAllowedOrientations is only invoked when incline limiting is
    // enabled; disabled means no attitude restriction at all.
    if (!robot.enableInclineLimitting)
        return AngleIntervalSet::full();

    // Legacy: slope >= maxSlope -> computeAllowedOrientations returns false (OBSTACLE).
    if (slope >= robot.maxSlope)
        return AngleIntervalSet::empty();

    // Legacy: below the limiting threshold all orientations are allowed.
    if (slope < robot.inclineLimittingMinSlope)
        return AngleIntervalSet::full();

    // Wedge half-width shrinks linearly from pi/2 (at inclineLimittingMinSlope) to
    // inclineLimittingLimit (at maxSlope). Only reachable with minSlope <= slope
    // < maxSlope, so minSlope < maxSlope holds and the division is well defined.
    const double limitRad = interpolate(slope, robot.inclineLimittingMinSlope, M_PI_2,
                                        robot.maxSlope, robot.inclineLimittingLimit);
    const double startRad = slopeDir - limitRad;
    const double width = 2.0 * limitRad;

    AngleIntervalSet allowed;
    // forward (uphill-facing) wedge, centered on the ascent azimuth
    allowed.add(startRad, width);
    // mirrored (downhill-facing) wedge, gated exactly as the legacy does
    if (robot.allowForwardDownhill)
        allowed.add(startRad - M_PI, width);
    return allowed;
}

AngleIntervalSet attitudeHeadings(const Eigen::Vector3f& normal, const RobotModel& robot)
{
    Eigen::Vector3f n = normal;
    const float norm = n.norm();
    if (norm > 0.f)
        n /= norm;
    if (n.z() < 0.f)
        n = -n; // enforce z-up orientation

    const double nz = std::min(1.0, std::max(-1.0, static_cast<double>(n.z())));
    const double slope = std::acos(nz);

    // Azimuth of steepest ascent: the horizontal part of the projection of z-up onto
    // the plane, proj = z - (z.n)n, is (-nz*nx, -nz*ny); with nz >= 0 its azimuth is
    // atan2(-ny, -nx) (matches the legacy computeSlopeDirection + atan2).
    const double slopeDir = std::atan2(-static_cast<double>(n.y()),
                                       -static_cast<double>(n.x()));

    return attitudeHeadings(slope, slopeDir, robot);
}

}
}
