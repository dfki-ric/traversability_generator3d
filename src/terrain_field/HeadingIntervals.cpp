#include "HeadingIntervals.hpp"

#include <algorithm>
#include <cmath>

namespace traversability_generator3d
{
namespace terrain_field
{

namespace
{
const double CENTER_DISK_EPS = 1e-12; //!< |d| below this is the heading-invariant disk
}

AngleIntervalSet feasibleHeadings(const Eigen::Vector2d& p,
                                  const std::vector<double>& diskOffsets,
                                  double radius,
                                  const std::vector<Eigen::Vector2d>& obstacles)
{
    AngleIntervalSet blocked;

    for (const auto& o : obstacles)
    {
        const Eigen::Vector2d rel = o - p;
        const double rho = rel.norm();

        for (double d : diskOffsets)
        {
            const double ad = std::fabs(d);

            if (ad < CENTER_DISK_EPS)
            {
                // center disk stays at p for every heading: heading-independent
                if (rho <= radius)
                    return AngleIntervalSet::empty(); // every heading collides
                continue;
            }
            if (rho >= ad + radius)
                continue; // obstacle out of reach of this disk at any heading
            if (rho + ad <= radius)
                return AngleIntervalSet::empty(); // obstacle inside every placement

            // Disk center sweeps the circle of radius |d| around p; the headings with
            // |o - (p + d*u(theta))| < radius form one arc. Law of cosines on the
            // triangle (p, disk center, o) gives its half-width. For rho < |d| - radius
            // the argument exceeds 1: after clamping delta == 0, correctly no block.
            double cosArg = (d * d + rho * rho - radius * radius) / (2.0 * ad * rho);
            cosArg = std::min(1.0, std::max(-1.0, cosArg));
            const double delta = std::acos(cosArg);
            if (delta <= 0.0)
                continue;

            const double phi = std::atan2(rel.y(), rel.x());
            const double center = (d > 0.0) ? phi : phi + M_PI;
            blocked.add(center - delta, 2.0 * delta);
        }
    }

    return blocked.complement();
}

}
}
