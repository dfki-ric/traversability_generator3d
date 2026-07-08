// Adversarial verification of feasibleHeadings (TerrainField M2, §5.4).
// Build: g++ -O2 -std=c++14 -I ../../src -I /usr/include/eigen3
//            verify_headings.cpp ../../src/terrain_field/HeadingIntervals.cpp
//            ../../src/terrain_field/AngleIntervalSet.cpp -o verify_hi && ./verify_hi
//
// Attacks what the builder's random test may have missed:
//  - obstacles placed EXACTLY on the critical annuli rho = |d|+r and rho = ||d|-r|
//    (exact axis-aligned arithmetic plus random-angle placements)
//  - near-center obstacles with radius larger than every |d| (wide, near-full arcs)
//  - collinear obstacle stacks along a single ray through p (max arc overlap/merging)
//  - duplicate offsets and offsets straddling the 1e-12 center-disk epsilon
//  - grazing scenes rho = (r - |d|) + eps (blocked arc width -> 2*pi)
//  - huge obstacle counts (1500 points)
//  - properties: symmetric offset sets must give pi-periodic allowed sets; mirroring
//    all obstacles through p (with symmetric offsets) must leave the set unchanged;
//    duplicated offsets must not change the result.
//
// Brute force sweeps theta in 0.02 degree steps. A heading is free iff every disk
// center keeps distance >= radius to every obstacle. Skipped: thetas within 1e-3 rad
// of a returned arc endpoint (float band) and thetas whose minimum clearance is within
// 1e-9 of radius (exact-boundary placements make these genuinely ambiguous).

#include "terrain_field/HeadingIntervals.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

using traversability_generator3d::terrain_field::AngleIntervalSet;
using traversability_generator3d::terrain_field::feasibleHeadings;

namespace
{
const double TWO_PI = 2.0 * M_PI;
const int N_THETA = 18000;                     // 360 / 0.02
const double THETA_STEP = TWO_PI / N_THETA;    // 0.02 degrees
const double ENDPOINT_BAND = 1e-3;             // [rad] around returned arc endpoints
const double DIST_BAND = 1e-9;                 // [m] clearance band around radius

long long gChecked = 0;
long long gMismatches = 0;
int gDeterministicFails = 0;

enum class Truth
{
    Free,
    Blocked,
    Boundary // min clearance within DIST_BAND of radius: do not judge
};

Truth truthAt(const Eigen::Vector2d& p,
              const std::vector<double>& offsets,
              double radius,
              const std::vector<Eigen::Vector2d>& obstacles,
              double theta)
{
    const double ct = std::cos(theta), st = std::sin(theta);
    double minDist = std::numeric_limits<double>::infinity();
    for (double d : offsets)
    {
        const Eigen::Vector2d c(p.x() + d * ct, p.y() + d * st);
        for (const auto& o : obstacles)
            minDist = std::min(minDist, (o - c).norm());
    }
    if (std::fabs(minDist - radius) < DIST_BAND)
        return Truth::Boundary;
    return minDist >= radius ? Truth::Free : Truth::Blocked;
}

/** Mark theta indices within ENDPOINT_BAND (circular) of any returned arc endpoint. */
std::vector<char> endpointMask(const AngleIntervalSet& s)
{
    std::vector<char> mask(N_THETA, 0);
    const int halo = (int)std::ceil(ENDPOINT_BAND / THETA_STEP) + 1;
    for (const auto& arc : s.arcs())
    {
        for (double e : {arc.first, arc.second})
        {
            const int center = (int)std::lround(e / THETA_STEP);
            for (int k = -halo; k <= halo; ++k)
            {
                const int i = ((center + k) % N_THETA + N_THETA) % N_THETA;
                double dist = std::fabs(i * THETA_STEP - e);
                dist = std::min(dist, TWO_PI - dist);
                if (dist < ENDPOINT_BAND)
                    mask[i] = 1;
            }
        }
    }
    return mask;
}

void bruteCompare(const char* tag,
                  int scene,
                  const Eigen::Vector2d& p,
                  const std::vector<double>& offsets,
                  double radius,
                  const std::vector<Eigen::Vector2d>& obstacles)
{
    const AngleIntervalSet allowed = feasibleHeadings(p, offsets, radius, obstacles);
    const std::vector<char> mask = endpointMask(allowed);
    for (int i = 0; i < N_THETA; ++i)
    {
        if (mask[i])
            continue;
        const double theta = i * THETA_STEP;
        const Truth t = truthAt(p, offsets, radius, obstacles, theta);
        if (t == Truth::Boundary)
            continue;
        const bool expected = (t == Truth::Free);
        const bool got = allowed.contains(theta);
        ++gChecked;
        if (expected != got)
        {
            if (gMismatches < 10)
                std::printf("MISMATCH [%s] scene %d theta %.6f expected %d got %d\n",
                            tag, scene, theta, (int)expected, (int)got);
            ++gMismatches;
        }
    }
}

void expect(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("DETERMINISTIC FAIL: %s\n", what);
        ++gDeterministicFails;
    }
}

/** Dense set-equality outside both endpoint bands. */
void expectSameSet(const AngleIntervalSet& a, const AngleIntervalSet& b,
                   const char* what)
{
    const std::vector<char> ma = endpointMask(a);
    const std::vector<char> mb = endpointMask(b);
    for (int i = 0; i < N_THETA; ++i)
    {
        if (ma[i] || mb[i])
            continue;
        const double theta = i * THETA_STEP;
        if (a.contains(theta) != b.contains(theta))
        {
            std::printf("PROPERTY FAIL: %s (theta %.6f)\n", what, theta);
            ++gDeterministicFails;
            return;
        }
    }
}
}

int main()
{
    std::mt19937 rng(424242);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::uniform_real_distribution<double> posDist(-5.0, 5.0);
    std::uniform_real_distribution<double> angDist(0.0, TWO_PI);

    // ---------------- deterministic exact-boundary cases (exact arithmetic) --------
    {
        const Eigen::Vector2d p(0.25, -0.5);
        // obstacle exactly on the outer annulus rho = d + r: tangent -> full circle
        {
            const AngleIntervalSet s = feasibleHeadings(
                p, {2.0}, 0.5, {p + Eigen::Vector2d(2.5, 0.0)});
            expect(s.isFull(), "rho == d + r exactly must yield the full circle");
        }
        // obstacle exactly on the inner annulus rho = d - r (d > r): tangent -> full
        {
            const AngleIntervalSet s = feasibleHeadings(
                p, {2.0}, 0.5, {p + Eigen::Vector2d(0.0, 1.5)});
            expect(s.isFull(), "rho == d - r exactly must yield the full circle");
        }
        // obstacle exactly at p, single off-center disk with |d| > r: never reached
        {
            const AngleIntervalSet s = feasibleHeadings(p, {-2.0}, 0.5, {p});
            expect(s.isFull(), "obstacle at p, |d| > r must yield the full circle");
        }
        // obstacle exactly at p, |d| < r: inside the disk at every heading -> empty
        {
            const AngleIntervalSet s = feasibleHeadings(p, {0.3}, 0.5, {p});
            expect(s.isEmpty(), "obstacle at p, |d| < r must yield the empty set");
        }
        // obstacle exactly at p, |d| == r: rho + |d| <= r boundary -> empty (spec)
        {
            const AngleIntervalSet s = feasibleHeadings(p, {0.5}, 0.5, {p});
            expect(s.isEmpty(), "obstacle at p, |d| == r must yield the empty set");
        }
        // center disk, obstacle exactly at distance radius: rho <= r -> empty (spec)
        {
            const AngleIntervalSet s = feasibleHeadings(
                p, {0.0}, 0.5, {p + Eigen::Vector2d(0.5, 0.0)});
            expect(s.isEmpty(), "d == 0, rho == r exactly must yield the empty set");
        }
        // no disks at all: nothing can collide -> full even with obstacles present
        {
            const AngleIntervalSet s = feasibleHeadings(
                p, {}, 0.5, {p, p + Eigen::Vector2d(0.1, 0.1)});
            expect(s.isFull(), "empty offset list must yield the full circle");
        }
        // offset exactly at the center-disk epsilon boundary behaves like d == 0
        {
            const AngleIntervalSet s = feasibleHeadings(
                p, {1e-13}, 0.5, {p + Eigen::Vector2d(0.2, 0.0)});
            expect(s.isEmpty(), "|d| < 1e-12 with rho < r must yield the empty set");
        }
    }

    // ---------------- F0: near-center obstacles, radius > every |d| ----------------
    for (int scene = 0; scene < 40; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double radius = 2.5 + uni01(rng) * 1.5;
        std::vector<double> offsets;
        const int n = 1 + (int)(uni01(rng) * 4.0);
        for (int k = 0; k < n; ++k)
            offsets.push_back((uni01(rng) * 2.0 - 1.0) * 2.0); // |d| <= 2 < radius
        double dMax = 0.0;
        for (double d : offsets)
            dMax = std::max(dMax, std::fabs(d));
        std::vector<Eigen::Vector2d> obstacles;
        const int nObs = 1 + (int)(uni01(rng) * 12.0);
        for (int k = 0; k < nObs; ++k)
        {
            // bias rho into (r - dMax, r + dMax): the arc-generating window
            const double rho = std::max(
                0.0, radius + (uni01(rng) * 2.0 - 1.0) * (dMax + 0.3));
            const double a = angDist(rng);
            obstacles.emplace_back(p.x() + rho * std::cos(a),
                                   p.y() + rho * std::sin(a));
        }
        bruteCompare("F0 near-center", scene, p, offsets, radius, obstacles);
    }

    // ---------------- F1: collinear obstacle stacks along one ray ------------------
    for (int scene = 0; scene < 40; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double radius = 0.3 + uni01(rng) * 1.2;
        const std::vector<double> offsets{-2.25, -0.75, 0.75, 2.25};
        const double a = angDist(rng);
        const Eigen::Vector2d u(std::cos(a), std::sin(a));
        std::vector<Eigen::Vector2d> obstacles;
        const int nObs = 3 + (int)(uni01(rng) * 15.0);
        for (int k = 0; k < nObs; ++k)
        {
            const double rho = uni01(rng) * (2.25 + radius + 0.5);
            const double sign = (uni01(rng) < 0.5) ? 1.0 : -1.0; // both ray directions
            obstacles.emplace_back(p + sign * rho * u);
        }
        bruteCompare("F1 collinear", scene, p, offsets, radius, obstacles);
    }

    // ---------------- F2: duplicate and epsilon-straddling offsets -----------------
    for (int scene = 0; scene < 40; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double radius = 0.3 + uni01(rng) * 1.5;
        const double d = 0.2 + uni01(rng) * 2.5;
        const std::vector<double> offsets{d, d, -d, 1e-13, 1e-10, -d, d};
        std::vector<Eigen::Vector2d> obstacles;
        const int nObs = 1 + (int)(uni01(rng) * 10.0);
        for (int k = 0; k < nObs; ++k)
        {
            const double rho = uni01(rng) * (d + radius + 0.5);
            const double a = angDist(rng);
            obstacles.emplace_back(p.x() + rho * std::cos(a),
                                   p.y() + rho * std::sin(a));
        }
        bruteCompare("F2 dup-offsets", scene, p, offsets, radius, obstacles);

        // duplicates must be a no-op on the result
        const AngleIntervalSet full = feasibleHeadings(p, offsets, radius, obstacles);
        const AngleIntervalSet dedup = feasibleHeadings(
            p, {d, -d, 1e-13, 1e-10}, radius, obstacles);
        expectSameSet(full, dedup, "duplicate offsets changed the allowed set");
    }

    // ---------------- F3: obstacles exactly on the critical annuli -----------------
    for (int scene = 0; scene < 40; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double radius = 0.3 + uni01(rng) * 1.5;
        std::vector<double> offsets;
        const int n = 1 + (int)(uni01(rng) * 3.0);
        for (int k = 0; k < n; ++k)
            offsets.push_back((uni01(rng) * 2.0 - 1.0) * 3.0);
        std::vector<Eigen::Vector2d> obstacles;
        for (double d : offsets)
        {
            const double ad = std::fabs(d);
            for (double rho : {ad + radius, std::fabs(ad - radius)})
            {
                const double a = angDist(rng);
                obstacles.emplace_back(p.x() + rho * std::cos(a),
                                       p.y() + rho * std::sin(a));
            }
        }
        bruteCompare("F3 exact-annulus", scene, p, offsets, radius, obstacles);
    }

    // ---------------- F4: grazing rho = (r - |d|) + eps: near-full arcs ------------
    for (int scene = 0; scene < 40; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double d = 0.3 + uni01(rng) * 1.0;
        const double radius = d + 0.5 + uni01(rng); // radius > |d|
        const std::vector<double> offsets{d, -d};
        std::vector<Eigen::Vector2d> obstacles;
        const int nObs = 1 + (int)(uni01(rng) * 4.0);
        for (int k = 0; k < nObs; ++k)
        {
            const double eps = std::pow(10.0, -1.0 - uni01(rng) * 5.0); // 1e-1..1e-6
            const double rho = (radius - d) + eps;
            const double a = angDist(rng);
            obstacles.emplace_back(p.x() + rho * std::cos(a),
                                   p.y() + rho * std::sin(a));
        }
        bruteCompare("F4 grazing", scene, p, offsets, radius, obstacles);
    }

    // ---------------- F5: huge obstacle counts --------------------------------------
    for (int scene = 0; scene < 4; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double radius = 0.4 + uni01(rng);
        const std::vector<double> offsets{-2.25, -0.75, 0.0, 0.75, 2.25};
        std::vector<Eigen::Vector2d> obstacles;
        obstacles.reserve(1500);
        for (int k = 0; k < 1500; ++k)
        {
            const double rho = radius + 0.05 + uni01(rng) * (2.25 + 1.0);
            const double a = angDist(rng);
            obstacles.emplace_back(p.x() + rho * std::cos(a),
                                   p.y() + rho * std::sin(a));
        }
        bruteCompare("F5 huge", scene, p, offsets, radius, obstacles);
    }

    // ---------------- properties: pi-periodicity and mirror invariance -------------
    for (int scene = 0; scene < 40; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double radius = 0.3 + uni01(rng) * 1.5;
        std::vector<double> offsets;
        const int pairs = 1 + (int)(uni01(rng) * 3.0);
        for (int k = 0; k < pairs; ++k)
        {
            const double d = 0.2 + uni01(rng) * 3.0;
            offsets.push_back(d);
            offsets.push_back(-d);
        }
        if (uni01(rng) < 0.5)
            offsets.push_back(0.0);
        std::vector<Eigen::Vector2d> obstacles, mirrored;
        const int nObs = 1 + (int)(uni01(rng) * 15.0);
        for (int k = 0; k < nObs; ++k)
        {
            const double rho = uni01(rng) * (3.0 + radius + 0.5);
            const double a = angDist(rng);
            const Eigen::Vector2d o(p.x() + rho * std::cos(a),
                                    p.y() + rho * std::sin(a));
            obstacles.push_back(o);
            mirrored.push_back(2.0 * p - o);
        }

        const AngleIntervalSet s = feasibleHeadings(p, offsets, radius, obstacles);
        const std::vector<char> mask = endpointMask(s);
        // symmetric offsets: theta and theta + pi are equivalent placements
        for (int i = 0; i < N_THETA / 2; ++i)
        {
            const int j = i + N_THETA / 2; // exactly + pi on this grid
            if (mask[i] || mask[j])
                continue;
            if (s.contains(i * THETA_STEP) != s.contains(j * THETA_STEP))
            {
                std::printf("PROPERTY FAIL: pi-periodicity, scene %d theta %.6f\n",
                            scene, i * THETA_STEP);
                ++gDeterministicFails;
                break;
            }
        }
        // mirroring every obstacle through p must not change the set
        const AngleIntervalSet sm = feasibleHeadings(p, offsets, radius, mirrored);
        expectSameSet(s, sm, "mirror invariance under symmetric offsets");
    }

    std::printf("point checks:       %lld\n", gChecked);
    std::printf("mismatches:         %lld\n", gMismatches);
    std::printf("deterministic/prop: %d\n", gDeterministicFails);
    const bool pass = gMismatches == 0 && gDeterministicFails == 0 &&
                      gChecked > 1000000;
    std::printf("RESULT: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
