// Verification of AttitudeIntervals + Cost against a self-contained oracle that
// reimplements the legacy TraversabilityGenerator3d::computeAllowedOrientations
// (incl. interpolate() and the base::AngleSegment wedge semantics).
// Build: g++ -O2 -std=c++14 -I ../../src -I /usr/include/eigen3
//            test_attitude_intervals.cpp
//            ../../src/terrain_field/AttitudeIntervals.cpp
//            ../../src/terrain_field/AngleIntervalSet.cpp -o test_attitude && ./test_attitude

#include "terrain_field/AttitudeIntervals.hpp"
#include "terrain_field/Cost.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using traversability_generator3d::terrain_field::AngleIntervalSet;
using traversability_generator3d::terrain_field::CostWeights;
using traversability_generator3d::terrain_field::RobotModel;
using traversability_generator3d::terrain_field::TerrainCell;
using traversability_generator3d::terrain_field::attitudeHeadings;
using traversability_generator3d::terrain_field::terrainCost;

namespace
{
const double TWO_PI = 2.0 * M_PI;

double norm02pi(double a)
{
    double v = std::fmod(a, TWO_PI);
    if (v < 0.0)
        v += TWO_PI;
    if (v >= TWO_PI)
        v -= TWO_PI;
    return v;
}

/** Circular distance between two angles, in [0, pi]. */
double circDist(double a, double b)
{
    const double d = norm02pi(a - b);
    return std::min(d, TWO_PI - d);
}

// ---------------------------------------------------------------------------
// ORACLE: line-by-line port of the legacy wedge construction.
//
// Legacy TraversabilityGenerator3d::interpolate (TraversabilityGenerator3d.cpp:349):
//     return y0 + (x - x0) * (y1 - y0)/(x1-x0);
//
// Legacy computeAllowedOrientations (TraversabilityGenerator3d.cpp:275):
//     if (slope >= maxSlope) return false;                        // -> OBSTACLE
//     if (slope < inclineLimittingMinSlope)
//         allowedOrientations.emplace_back(Angle::fromRad(0), 2*M_PI);
//     else {
//         limitRad = interpolate(slope, inclineLimittingMinSlope, M_PI_2,
//                                maxSlope, inclineLimittingLimit);
//         startRad = slopeDirectionAtan2 - limitRad;
//         width    = 2 * limitRad;
//         allowedOrientations.emplace_back(Angle::fromRad(startRad), width);
//         if (allowForwardDownhill)                                // mirrored wedge
//             allowedOrientations.emplace_back(Angle::fromRad(startRad - M_PI), width);
//     }
// and computeAllowedOrientations is only invoked when enableInclineLimitting is set
// (TraversabilityGenerator3d.cpp:1736); disabled means no attitude restriction.
// ---------------------------------------------------------------------------

struct Wedge
{
    double start; //!< radians, any range
    double width; //!< radians, >= 0
};

struct Oracle
{
    bool obstacle = false;     //!< legacy returned false (slope >= maxSlope, limiting on)
    bool unrestricted = false; //!< limiting disabled -> no wedges emitted at all
    std::vector<Wedge> wedges; //!< legacy base::AngleSegment list (start/width pairs)
};

double legacyInterpolate(double x, double x0, double y0, double x1, double y1)
{
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

Oracle legacyAllowedOrientations(double slope, double slopeDirAtan2, const RobotModel& c)
{
    Oracle o;
    if (!c.enableInclineLimitting)
    {
        o.unrestricted = true;
        return o;
    }
    if (slope >= c.maxSlope)
    {
        o.obstacle = true;
        return o;
    }
    if (slope < c.inclineLimittingMinSlope)
    {
        o.wedges.push_back(Wedge{0.0, TWO_PI});
        return o;
    }
    const double limitRad = legacyInterpolate(slope, c.inclineLimittingMinSlope, M_PI_2,
                                              c.maxSlope, c.inclineLimittingLimit);
    const double startRad = slopeDirAtan2 - limitRad;
    const double width = 2.0 * limitRad;
    o.wedges.push_back(Wedge{startRad, width});
    if (c.allowForwardDownhill)
        o.wedges.push_back(Wedge{startRad - M_PI, width});
    return o;
}

/** base::AngleSegment::isInside semantics: theta in [start, start + width] (closed),
 *  evaluated circularly. */
bool wedgeContains(const Wedge& w, double theta)
{
    return norm02pi(theta - w.start) <= w.width;
}

bool oracleContains(const Oracle& o, double theta)
{
    if (o.obstacle)
        return false;
    if (o.unrestricted)
        return true;
    for (const auto& w : o.wedges)
        if (wedgeContains(w, theta))
            return true;
    return false;
}

bool nearWedgeBoundary(const Oracle& o, double theta, double tol)
{
    for (const auto& w : o.wedges)
    {
        if (circDist(theta, w.start) < tol)
            return true;
        if (circDist(theta, w.start + w.width) < tol)
            return true;
    }
    return false;
}

/** z-up unit normal of a plane with the given slope whose steepest-ascent azimuth
 *  is slopeDir. */
Eigen::Vector3f normalFrom(double slope, double slopeDir)
{
    return Eigen::Vector3f(-(float)(std::sin(slope) * std::cos(slopeDir)),
                           -(float)(std::sin(slope) * std::sin(slopeDir)),
                           (float)std::cos(slope));
}
}

int main()
{
    std::mt19937 rng(20260707);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    int failures = 0;

    // ------------------------------------------------------------------
    // 1) Membership equality vs the legacy oracle on random configurations
    // ------------------------------------------------------------------
    const int CONFIGS = 400;
    const int ANGLES = 4000;
    const double GUARD = 1e-3;

    long long checked = 0, skipped = 0, mismatches = 0;

    for (int c = 0; c < CONFIGS; ++c)
    {
        RobotModel robot;
        robot.enableInclineLimitting = (uni01(rng) < 0.5);
        robot.allowForwardDownhill = (uni01(rng) < 0.5);
        robot.inclineLimittingLimit = uni01(rng) * 1.0;
        do
        {
            robot.inclineLimittingMinSlope = uni01(rng) * 0.4;
            robot.maxSlope = 0.3 + uni01(rng) * 0.5;
        } while (!(robot.inclineLimittingMinSlope < robot.maxSlope));

        double slope = uni01(rng) * 0.8;
        double slopeDir = uni01(rng) * TWO_PI;
        // sprinkle in exact branch-threshold slopes to pin the >=/< semantics
        if (c % 17 == 0)
            slope = robot.maxSlope;
        else if (c % 17 == 1)
            slope = robot.inclineLimittingMinSlope;

        const Oracle oracle = legacyAllowedOrientations(slope, slopeDir, robot);
        const AngleIntervalSet mine = attitudeHeadings(slope, slopeDir, robot);

        for (int k = 0; k < ANGLES; ++k)
        {
            const double theta = uni01(rng) * TWO_PI;
            if (nearWedgeBoundary(oracle, theta, GUARD))
            {
                ++skipped;
                continue;
            }
            ++checked;
            const bool expect = oracleContains(oracle, theta);
            const bool got = mine.contains(theta);
            if (expect != got)
            {
                if (mismatches < 10)
                    std::printf("MISMATCH cfg=%d theta=%.9f expect=%d got=%d "
                                "(slope=%.6f dir=%.6f enable=%d fwdDown=%d "
                                "minSlope=%.6f limit=%.6f maxSlope=%.6f)\n",
                                c, theta, (int)expect, (int)got, slope, slopeDir,
                                (int)robot.enableInclineLimitting,
                                (int)robot.allowForwardDownhill,
                                robot.inclineLimittingMinSlope,
                                robot.inclineLimittingLimit, robot.maxSlope);
                ++mismatches;
            }
        }
    }
    std::printf("oracle membership: %d configs, %lld angles checked, %lld near-boundary "
                "skipped, %lld mismatches\n",
                CONFIGS, checked, skipped, mismatches);
    if (mismatches != 0)
        ++failures;

    // ------------------------------------------------------------------
    // 2) Normal-overload consistency with the analytic overload
    // ------------------------------------------------------------------
    long long normalMismatches = 0;
    for (int c = 0; c < 200; ++c)
    {
        RobotModel robot;
        robot.enableInclineLimitting = true;
        robot.allowForwardDownhill = (uni01(rng) < 0.5);
        robot.inclineLimittingLimit = uni01(rng);
        do
        {
            robot.inclineLimittingMinSlope = uni01(rng) * 0.4;
            robot.maxSlope = 0.3 + uni01(rng) * 0.5;
        } while (!(robot.inclineLimittingMinSlope < robot.maxSlope));

        const double slope = 0.05 + uni01(rng) * 0.7;
        const double slopeDir = uni01(rng) * TWO_PI;

        const Oracle oracle = legacyAllowedOrientations(slope, slopeDir, robot);
        const AngleIntervalSet fromNormal =
            attitudeHeadings(normalFrom(slope, slopeDir), robot);

        for (int k = 0; k < 500; ++k)
        {
            const double theta = uni01(rng) * TWO_PI;
            // float normal + trig round-trip: keep clear of wedge edges
            if (nearWedgeBoundary(oracle, theta, 1e-3))
                continue;
            if (oracleContains(oracle, theta) != fromNormal.contains(theta))
            {
                if (normalMismatches < 5)
                    std::printf("NORMAL MISMATCH cfg=%d theta=%.9f slope=%.6f dir=%.6f\n",
                                c, theta, slope, slopeDir);
                ++normalMismatches;
            }
        }
    }
    std::printf("normal overload: %lld mismatches\n", normalMismatches);
    if (normalMismatches != 0)
        ++failures;

    // ------------------------------------------------------------------
    // 3) Sanity checks
    // ------------------------------------------------------------------
    {
        RobotModel robot;
        robot.enableInclineLimitting = true;
        robot.inclineLimittingMinSlope = 0.2;
        robot.inclineLimittingLimit = 0.1;
        robot.maxSlope = 0.45;

        if (!attitudeHeadings(0.1, 1.0, robot).isFull())
        {
            std::printf("FAIL: slope < minSlope must yield the full circle\n");
            ++failures;
        }
        if (!attitudeHeadings(0.5, 1.0, robot).isEmpty())
        {
            std::printf("FAIL: slope > maxSlope must yield the empty set\n");
            ++failures;
        }
        if (!attitudeHeadings(0.45, 1.0, robot).isEmpty())
        {
            std::printf("FAIL: slope == maxSlope must yield the empty set (legacy >=)\n");
            ++failures;
        }

        robot.enableInclineLimitting = false;
        if (!attitudeHeadings(0.7, 1.0, robot).isFull())
        {
            std::printf("FAIL: limiting disabled must yield the full circle\n");
            ++failures;
        }

        // allowForwardDownhill gating: uphill heading always allowed in the wedge
        // branch, downhill only when the flag is set
        robot.enableInclineLimitting = true;
        robot.allowForwardDownhill = false;
        const double dir = 2.3;
        AngleIntervalSet uphillOnly = attitudeHeadings(0.4, dir, robot);
        if (!uphillOnly.contains(dir) || uphillOnly.contains(dir - M_PI))
        {
            std::printf("FAIL: allowForwardDownhill=false must allow uphill only\n");
            ++failures;
        }
        robot.allowForwardDownhill = true;
        AngleIntervalSet both = attitudeHeadings(0.4, dir, robot);
        if (!both.contains(dir) || !both.contains(dir - M_PI))
        {
            std::printf("FAIL: allowForwardDownhill=true must allow the mirrored wedge\n");
            ++failures;
        }
        // sideways heading must be blocked in the wedge branch (limit < pi/2)
        if (both.contains(dir + M_PI_2))
        {
            std::printf("FAIL: sideways heading must be blocked above minSlope\n");
            ++failures;
        }
    }

    // ------------------------------------------------------------------
    // 4) terrainCost: nonnegative + monotone in each degradation direction
    // ------------------------------------------------------------------
    {
        long long costFails = 0;
        for (int c = 0; c < 2000; ++c)
        {
            CostWeights w;
            w.wSlope = uni01(rng) * 2.0;
            w.wRough = uni01(rng) * 2.0;
            w.wClear = uni01(rng) * 2.0;
            w.wConf = uni01(rng) * 2.0;
            w.costFunctionDist = (uni01(rng) < 0.3) ? 0.0 : 0.5 + uni01(rng) * 2.5;

            const double slope = uni01(rng) * 0.8;
            TerrainCell cell;
            cell.normal = normalFrom(slope, uni01(rng) * TWO_PI);
            cell.roughness = (float)(uni01(rng) * 0.3);
            cell.confidence = (float)uni01(rng);
            const float esdf = (float)(uni01(rng) * 4.0);

            const double base = terrainCost(cell, esdf, w);
            if (!(base >= 0.0))
            {
                if (costFails < 5)
                    std::printf("FAIL: terrainCost < 0 (%.9f)\n", base);
                ++costFails;
            }

            // increase slope
            TerrainCell worse = cell;
            const double slope2 = slope + uni01(rng) * (0.8 - slope);
            worse.normal = normalFrom(slope2, uni01(rng) * TWO_PI);
            if (terrainCost(worse, esdf, w) < base - 1e-9)
            {
                if (costFails < 5)
                    std::printf("FAIL: cost decreased with increasing slope\n");
                ++costFails;
            }
            // increase roughness
            worse = cell;
            worse.roughness = cell.roughness + (float)(uni01(rng) * 0.3);
            if (terrainCost(worse, esdf, w) < base - 1e-9)
            {
                if (costFails < 5)
                    std::printf("FAIL: cost decreased with increasing roughness\n");
                ++costFails;
            }
            // decrease confidence
            worse = cell;
            worse.confidence = cell.confidence * (float)uni01(rng);
            if (terrainCost(worse, esdf, w) < base - 1e-9)
            {
                if (costFails < 5)
                    std::printf("FAIL: cost decreased with decreasing confidence\n");
                ++costFails;
            }
            // decrease esdf
            const float esdf2 = esdf * (float)uni01(rng);
            if (terrainCost(cell, esdf2, w) < base - 1e-9)
            {
                if (costFails < 5)
                    std::printf("FAIL: cost decreased with decreasing esdf\n");
                ++costFails;
            }
        }
        std::printf("terrainCost checks: %lld failures\n", costFails);
        if (costFails != 0)
            ++failures;
    }

    if (failures == 0)
    {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL\n");
    return 1;
}
