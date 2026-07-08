// ADVERSARIAL verification of AttitudeIntervals + Cost. Attacks what the main
// test may have missed: exact branch thresholds (slope == minSlope, == maxSlope),
// limits wider than pi/2 and wider than 2*pi (overlapping / full wedges),
// degenerate zero-width limits, slopeDir wraparound (large positive / negative),
// near-vertical and degenerate normals (NaN safety), mirror symmetry of the
// normal overload, and terrainCost with infinite/zero esdf and out-of-range
// normal.z / confidence.
// Build: g++ -O2 -Wall -std=c++14 -I ../../src -I /usr/include/eigen3
//            verify_attitude.cpp
//            ../../src/terrain_field/AttitudeIntervals.cpp
//            ../../src/terrain_field/AngleIntervalSet.cpp -o verify_attitude

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

double circDist(double a, double b)
{
    const double d = norm02pi(a - b);
    return std::min(d, TWO_PI - d);
}

// Independent oracle: literal port of the legacy computeAllowedOrientations /
// interpolate / base::AngleSegment closed-membership semantics.
struct Wedge
{
    double start;
    double width;
};

struct Oracle
{
    bool obstacle = false;
    bool unrestricted = false;
    std::vector<Wedge> wedges;
};

Oracle legacyOracle(double slope, double slopeDirAtan2, const RobotModel& c)
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
    const double limitRad = M_PI_2 + (slope - c.inclineLimittingMinSlope)
                            * (c.inclineLimittingLimit - M_PI_2)
                            / (c.maxSlope - c.inclineLimittingMinSlope);
    const double startRad = slopeDirAtan2 - limitRad;
    const double width = 2.0 * limitRad;
    o.wedges.push_back(Wedge{startRad, width});
    if (c.allowForwardDownhill)
        o.wedges.push_back(Wedge{startRad - M_PI, width});
    return o;
}

bool oracleContains(const Oracle& o, double theta)
{
    if (o.obstacle)
        return false;
    if (o.unrestricted)
        return true;
    for (const auto& w : o.wedges)
        if (norm02pi(theta - w.start) <= w.width)
            return true;
    return false;
}

bool nearBoundary(const Oracle& o, double theta, double tol)
{
    for (const auto& w : o.wedges)
        if (circDist(theta, w.start) < tol || circDist(theta, w.start + w.width) < tol)
            return true;
    return false;
}

int gFailures = 0;

/** Dense membership sweep of the module against the oracle. Guard band only at
 *  oracle wedge boundaries (tight: analytic double arithmetic on both sides). */
void sweepCompare(const char* what, double slope, double slopeDir,
                  const RobotModel& robot, int steps = 20011, double guard = 1e-7)
{
    const Oracle o = legacyOracle(slope, slopeDir, robot);
    const AngleIntervalSet s = attitudeHeadings(slope, slopeDir, robot);
    long long bad = 0;
    for (int i = 0; i < steps; ++i)
    {
        const double theta = TWO_PI * (double)i / (double)steps + 1.2345e-4;
        if (nearBoundary(o, theta, guard))
            continue;
        if (oracleContains(o, theta) != s.contains(theta))
        {
            if (bad < 3)
                std::printf("FAIL [%s]: theta=%.12f expect=%d got=%d\n", what, theta,
                            (int)oracleContains(o, theta), (int)s.contains(theta));
            ++bad;
        }
    }
    if (bad != 0)
    {
        std::printf("FAIL [%s]: %lld sweep mismatches (slope=%.9f dir=%.9f)\n",
                    what, bad, slope, slopeDir);
        ++gFailures;
    }
}

bool arcsFinite(const AngleIntervalSet& s)
{
    for (const auto& a : s.arcs())
        if (!std::isfinite(a.first) || !std::isfinite(a.second))
            return false;
    return true;
}

void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++gFailures;
    }
}
}

int main()
{
    std::mt19937 rng(424242);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    RobotModel base;
    base.enableInclineLimitting = true;
    base.inclineLimittingMinSlope = 0.2;
    base.inclineLimittingLimit = 0.1;
    base.maxSlope = 0.45;

    // ---- 1) exact branch thresholds -------------------------------------
    for (int fwd = 0; fwd <= 1; ++fwd)
    {
        RobotModel r = base;
        r.allowForwardDownhill = (fwd != 0);
        // slope exactly == minSlope: limitRad == pi/2 exactly, width == pi
        sweepCompare("slope==minSlope", r.inclineLimittingMinSlope, 1.7, r);
        // with the mirrored wedge the two half circles tile the whole circle
        if (r.allowForwardDownhill)
        {
            const AngleIntervalSet s =
                attitudeHeadings(r.inclineLimittingMinSlope, 1.7, r);
            check(s.totalWidth() > TWO_PI - 1e-9,
                  "slope==minSlope + fwdDownhill must cover (almost) the full circle");
        }
        // slope just below maxSlope (narrowest wedge)
        sweepCompare("slope=maxSlope-eps", std::nextafter(r.maxSlope, 0.0), 0.3, r);
        // slope exactly == maxSlope -> empty, and above
        check(attitudeHeadings(r.maxSlope, 0.3, r).isEmpty(),
              "slope==maxSlope must be empty");
        check(attitudeHeadings(r.maxSlope + 1e-12, 0.3, r).isEmpty(),
              "slope>maxSlope must be empty");
        // slope just below minSlope -> full
        check(attitudeHeadings(std::nextafter(r.inclineLimittingMinSlope, 0.0), 0.3,
                               r).isFull(),
              "slope just below minSlope must be full");
    }

    // ---- 2) minSlope == 0 with slope == 0 (wedge branch at zero slope) ---
    {
        RobotModel r = base;
        r.inclineLimittingMinSlope = 0.0;
        sweepCompare("minSlope==0, slope==0", 0.0, 2.9, r);
        sweepCompare("minSlope==0, slope==0, dir=0", 0.0, 0.0, r);
    }

    // ---- 3) limit wider than pi/2 (overlapping wedges) and > 2*pi -------
    {
        RobotModel r = base;
        r.inclineLimittingLimit = 2.5; // > pi/2: width > pi, wedges overlap
        r.allowForwardDownhill = true;
        for (double slope : {0.2, 0.3, 0.4, 0.4499})
            sweepCompare("limit=2.5", slope, 5.9, r);
        r.allowForwardDownhill = false;
        sweepCompare("limit=2.5 noDown", 0.44, 5.9, r);

        r.inclineLimittingLimit = 7.0; // width up to 14 rad > 2*pi near maxSlope
        r.allowForwardDownhill = false;
        sweepCompare("limit=7 (width>2pi)", 0.44, 1.0, r);
        check(attitudeHeadings(0.4499, 1.0, r).isFull(),
              "width > 2*pi must yield the full circle");
    }

    // ---- 4) degenerate limit == 0 (near-zero-width wedge) ----------------
    {
        RobotModel r = base;
        r.inclineLimittingLimit = 0.0;
        r.allowForwardDownhill = true;
        const double slope = std::nextafter(r.maxSlope, 0.0);
        sweepCompare("limit==0 near maxSlope", slope, 2.2, r, 20011, 1e-6);
        const AngleIntervalSet s = attitudeHeadings(slope, 2.2, r);
        check(s.totalWidth() < 1e-6, "limit==0 near maxSlope must be near-degenerate");
        check(!s.contains(2.2 + 0.1), "limit==0: off-axis heading must be blocked");
    }

    // ---- 5) slopeDir wraparound / far outside [0, 2pi) -------------------
    for (double dir : {-100.0, -TWO_PI, -1e-14, 0.0, TWO_PI, TWO_PI - 1e-14, 1000.0})
    {
        RobotModel r = base;
        r.allowForwardDownhill = true;
        sweepCompare("slopeDir wraparound", 0.35, dir, r);
        r.allowForwardDownhill = false;
        sweepCompare("slopeDir wraparound noDown", 0.35, dir, r);
    }

    // ---- 6) randomized adversarial configs, tight guard ------------------
    for (int c = 0; c < 300; ++c)
    {
        RobotModel r;
        r.enableInclineLimitting = true;
        r.allowForwardDownhill = (uni01(rng) < 0.5);
        r.inclineLimittingLimit = uni01(rng) * 3.5; // deliberately beyond pi/2
        do
        {
            r.inclineLimittingMinSlope = uni01(rng) * 0.4;
            r.maxSlope = 0.3 + uni01(rng) * 0.5;
        } while (!(r.inclineLimittingMinSlope < r.maxSlope));
        const double slope = r.inclineLimittingMinSlope
                             + uni01(rng) * (r.maxSlope - r.inclineLimittingMinSlope);
        sweepCompare("random adversarial", slope, uni01(rng) * 400.0 - 200.0, r, 3001);
    }

    // ---- 7) normals: vertical / degenerate / non-normalized / mirror -----
    {
        RobotModel r = base;
        r.allowForwardDownhill = false;

        // perfectly vertical: slope == 0 < minSlope -> full, and never NaN
        for (auto n : {Eigen::Vector3f(0.f, 0.f, 1.f), Eigen::Vector3f(0.f, 0.f, -1.f),
                       Eigen::Vector3f(0.f, 0.f, 0.5f), Eigen::Vector3f(0.f, 0.f, 42.f)})
        {
            const AngleIntervalSet s = attitudeHeadings(n, r);
            check(arcsFinite(s), "vertical normal must not produce NaN arcs");
            check(s.isFull(), "vertical normal (slope 0 < minSlope) must be full");
        }
        // near-vertical, denormal horizontal part: slopeDir ill-conditioned but the
        // slope is ~0 < minSlope so the result must still be the full circle, no NaN
        {
            const AngleIntervalSet s =
                attitudeHeadings(Eigen::Vector3f(1e-30f, -1e-30f, 1.f), r);
            check(arcsFinite(s) && s.isFull(), "near-vertical normal must be full/finite");
        }
        // vertical normal with minSlope == 0: wedge branch with undefined azimuth.
        // Any full-circle-consistent wedge is acceptable; it must just not NaN and
        // must have the correct width.
        {
            RobotModel r0 = r;
            r0.inclineLimittingMinSlope = 0.0;
            const AngleIntervalSet s = attitudeHeadings(Eigen::Vector3f(0, 0, 1), r0);
            check(arcsFinite(s), "vertical normal + minSlope==0 must not NaN");
            check(std::fabs(s.totalWidth() - M_PI) < 1e-6,
                  "vertical normal + minSlope==0: wedge width must be pi");
        }
        // zero normal: degenerate input; must not NaN or crash (slope = acos(0) =
        // pi/2 >= maxSlope -> empty is the documented behavior)
        {
            const AngleIntervalSet s = attitudeHeadings(Eigen::Vector3f(0, 0, 0), r);
            check(arcsFinite(s), "zero normal must not produce NaN arcs");
            check(s.isEmpty(), "zero normal (slope pi/2 >= maxSlope) must be empty");
        }
        // non-normalized normal must equal its normalized twin
        // and n vs -n must be identical (z-up flip)
        long long mirrorBad = 0;
        for (int c = 0; c < 200; ++c)
        {
            const double slope = 0.05 + uni01(rng) * 0.7;
            const double dir = uni01(rng) * TWO_PI;
            Eigen::Vector3f n(-(float)(std::sin(slope) * std::cos(dir)),
                              -(float)(std::sin(slope) * std::sin(dir)),
                              (float)std::cos(slope));
            const float scale = 0.01f + (float)uni01(rng) * 50.f;
            RobotModel rr = base;
            rr.allowForwardDownhill = (uni01(rng) < 0.5);
            const AngleIntervalSet a = attitudeHeadings(n, rr);
            const AngleIntervalSet b = attitudeHeadings((n * scale).eval(), rr);
            const AngleIntervalSet m = attitudeHeadings((-n).eval(), rr);
            for (int k = 0; k < 200; ++k)
            {
                const double theta = uni01(rng) * TWO_PI;
                // stay off the boundaries of a's arcs
                bool nearEdge = false;
                for (const auto& arc : a.arcs())
                    if (circDist(theta, arc.first) < 1e-5
                        || circDist(theta, arc.second) < 1e-5)
                        nearEdge = true;
                if (nearEdge)
                    continue;
                if (a.contains(theta) != b.contains(theta)
                    || a.contains(theta) != m.contains(theta))
                    ++mirrorBad;
            }
        }
        check(mirrorBad == 0, "scaled / negated normal must give the identical set");
    }

    // ---- 8) disabled limiting overrides everything ------------------------
    {
        RobotModel r = base;
        r.enableInclineLimitting = false;
        check(attitudeHeadings(10.0, 0.0, r).isFull(),
              "limiting disabled: even absurd slope must be full");
        check(attitudeHeadings(r.maxSlope, 0.0, r).isFull(),
              "limiting disabled: slope==maxSlope must be full");
    }

    // ---- 9) terrainCost adversarial ---------------------------------------
    {
        CostWeights w;
        w.wSlope = 1.0; w.wRough = 1.0; w.wClear = 1.0; w.wConf = 1.0;
        w.costFunctionDist = 2.0;

        TerrainCell cell;
        cell.normal = Eigen::Vector3f(0.f, 0.f, 1.f);
        cell.roughness = 0.f;
        cell.confidence = 1.f;

        const float INF = std::numeric_limits<float>::infinity();
        // esdf = inf: clearance term must vanish, cost finite
        const double cInf = terrainCost(cell, INF, w);
        check(std::isfinite(cInf) && std::fabs(cInf) < 1e-12,
              "flat/confident cell at esdf=inf must cost ~0");
        // esdf = 0: full clearance penalty
        check(std::fabs(terrainCost(cell, 0.f, w) - w.wClear) < 1e-12,
              "esdf=0 must add exactly wClear");
        // esdf beyond falloff: same as inf
        check(std::fabs(terrainCost(cell, 5.f, w) - cInf) < 1e-12,
              "esdf > costFunctionDist must not add clearance cost");
        // costFunctionDist <= 0 disables the clearance term even at esdf=0
        CostWeights w0 = w;
        w0.costFunctionDist = 0.0;
        check(std::fabs(terrainCost(cell, 0.f, w0)) < 1e-12,
              "costFunctionDist==0 must disable the clearance term");
        w0.costFunctionDist = -1.0;
        check(std::fabs(terrainCost(cell, 0.f, w0)) < 1e-12,
              "costFunctionDist<0 must disable the clearance term");

        // normal.z slightly above 1 (float noise) must not NaN via acos
        TerrainCell noisy = cell;
        noisy.normal = Eigen::Vector3f(0.f, 0.f, 1.0000002f);
        check(std::isfinite(terrainCost(noisy, 1.f, w)),
              "normal.z > 1 must not produce NaN cost");
        noisy.normal = Eigen::Vector3f(0.f, 0.f, -1.f);
        check(std::isfinite(terrainCost(noisy, 1.f, w)),
              "normal.z = -1 must not produce NaN cost");

        // out-of-range confidence must still respect the >= 0 contract
        TerrainCell over = cell;
        over.confidence = 2.f;
        check(terrainCost(over, INF, w) >= 0.0,
              "confidence > 1 must still yield cost >= 0");

        // all-zero weights -> exactly 0
        CostWeights z;
        z.wSlope = z.wRough = z.wClear = z.wConf = 0.0;
        z.costFunctionDist = 2.0;
        TerrainCell bad;
        bad.normal = Eigen::Vector3f(0.6f, 0.f, 0.8f);
        bad.roughness = 0.5f;
        bad.confidence = 0.f;
        check(terrainCost(bad, 0.f, z) == 0.0, "zero weights must give zero cost");
    }

    if (gFailures == 0)
    {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL (%d failures)\n", gFailures);
    return 1;
}
