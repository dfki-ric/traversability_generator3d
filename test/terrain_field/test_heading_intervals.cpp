// Standalone brute-force verification of feasibleHeadings (TerrainField M2, §5.4).
// Build: g++ -O2 -std=c++14 -I ../../src -I /usr/include/eigen3
//            test_heading_intervals.cpp ../../src/terrain_field/HeadingIntervals.cpp
//            ../../src/terrain_field/AngleIntervalSet.cpp -o test_hi && ./test_hi
//
// Sweeps theta in 0.02 degree steps and checks geometric collision (a heading is free
// iff every disk center keeps distance >= radius to every obstacle; >= so the boundary
// counts as free, matching closed intervals). Membership must match the returned set
// everywhere except within 1e-3 rad of a returned arc endpoint (floating point band).

#include "terrain_field/HeadingIntervals.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using traversability_generator3d::terrain_field::AngleIntervalSet;
using traversability_generator3d::terrain_field::feasibleHeadings;

namespace
{
const double TWO_PI = 2.0 * M_PI;
const double THETA_STEP = 0.02 * M_PI / 180.0; // 0.02 degrees
const int N_THETA = 18000;                     // 360 / 0.02
const double ENDPOINT_BAND = 1e-3;             // [rad] tolerance around arc endpoints

/** Free at heading theta iff no disk center is strictly within radius of an obstacle. */
bool bruteForceFree(const Eigen::Vector2d& p,
                    const std::vector<double>& offsets,
                    double radius,
                    const std::vector<Eigen::Vector2d>& obstacles,
                    double theta)
{
    const double ct = std::cos(theta), st = std::sin(theta);
    const double r2 = radius * radius;
    for (double d : offsets)
    {
        const Eigen::Vector2d c(p.x() + d * ct, p.y() + d * st);
        for (const auto& o : obstacles)
        {
            if ((o - c).squaredNorm() < r2) // >= radius counts as free
                return false;
        }
    }
    return true;
}

/** Mark all theta indices within ENDPOINT_BAND (circular) of any returned arc endpoint. */
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
}

int main()
{
    std::mt19937 rng(20260707);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::uniform_real_distribution<double> posDist(-5.0, 5.0);
    std::uniform_real_distribution<double> radDist(0.2, 3.0);
    std::uniform_real_distribution<double> offDist(-4.0, 4.0);
    std::uniform_real_distribution<double> angDist(0.0, TWO_PI);
    std::uniform_int_distribution<int> nObsDist(0, 40);

    const int SCENES = 320;
    long long checked = 0, skipped = 0, mismatches = 0;
    int specialFails = 0;

    for (int scene = 0; scene < SCENES; ++scene)
    {
        const Eigen::Vector2d p(posDist(rng), posDist(rng));
        const double radius = radDist(rng);

        // disk offset sets: robot-like symmetric, random symmetric, and fully random
        std::vector<double> offsets;
        switch (scene % 3)
        {
            case 0:
                offsets = {-2.25, -0.75, 0.75, 2.25};
                break;
            case 1:
            {
                const int pairs = 1 + (int)(uni01(rng) * 3.0); // 1..3 pairs
                for (int k = 0; k < pairs; ++k)
                {
                    const double d = 0.1 + uni01(rng) * 3.9;
                    offsets.push_back(-d);
                    offsets.push_back(d);
                }
                if (uni01(rng) < 0.5)
                    offsets.push_back(0.0);
                break;
            }
            default:
            {
                const int n = 1 + (int)(uni01(rng) * 6.0); // 1..6 offsets
                for (int k = 0; k < n; ++k)
                    offsets.push_back(offDist(rng));
                if (uni01(rng) < 0.3)
                    offsets.push_back(0.0);
                break;
            }
        }

        double dMax = 0.0;
        for (double d : offsets)
            dMax = std::max(dMax, std::fabs(d));

        // obstacles: half uniform in [0, dMax+r+1], half biased near critical annuli
        const int nObs = nObsDist(rng);
        std::vector<Eigen::Vector2d> obstacles;
        obstacles.reserve(nObs);
        for (int k = 0; k < nObs; ++k)
        {
            double rho;
            if (uni01(rng) < 0.5 || offsets.empty())
            {
                rho = uni01(rng) * (dMax + radius + 1.0);
            }
            else
            {
                const double d = offsets[(size_t)(uni01(rng) * offsets.size()) %
                                         offsets.size()];
                const double boundary = (uni01(rng) < 0.5)
                                            ? std::fabs(d) + radius
                                            : std::fabs(std::fabs(d) - radius);
                rho = std::max(0.0, boundary + (uni01(rng) - 0.5) * 0.1);
            }
            const double a = angDist(rng);
            obstacles.emplace_back(p.x() + rho * std::cos(a),
                                   p.y() + rho * std::sin(a));
        }

        const AngleIntervalSet allowed = feasibleHeadings(p, offsets, radius, obstacles);
        const std::vector<char> mask = endpointMask(allowed);

        for (int i = 0; i < N_THETA; ++i)
        {
            if (mask[i])
            {
                ++skipped;
                continue;
            }
            const double theta = i * THETA_STEP;
            const bool expected = bruteForceFree(p, offsets, radius, obstacles, theta);
            const bool got = allowed.contains(theta);
            ++checked;
            if (expected != got)
            {
                if (mismatches < 10)
                    std::printf("MISMATCH scene %d theta %.6f expected %d got %d\n",
                                scene, theta, (int)expected, (int)got);
                ++mismatches;
            }
        }
    }

    // special case: no obstacles -> full circle allowed
    {
        const std::vector<double> offsets{-2.25, -0.75, 0.75, 2.25};
        const AngleIntervalSet allowed =
            feasibleHeadings(Eigen::Vector2d(1.0, -2.0), offsets, 0.6, {});
        if (!allowed.isFull())
        {
            std::printf("SPECIAL FAIL: no obstacles must yield the full circle\n");
            ++specialFails;
        }
    }
    // special case: obstacle at p with rho < radius and a d=0 disk -> empty
    {
        const std::vector<double> offsets{-1.5, 0.0, 1.5};
        const Eigen::Vector2d p(0.5, 0.5);
        const std::vector<Eigen::Vector2d> obstacles{p + Eigen::Vector2d(0.05, -0.03)};
        const AngleIntervalSet allowed = feasibleHeadings(p, offsets, 0.8, obstacles);
        if (!allowed.isEmpty())
        {
            std::printf("SPECIAL FAIL: obstacle under the center disk must block all\n");
            ++specialFails;
        }
    }

    std::printf("scenes:         %d\n", SCENES);
    std::printf("point checks:   %lld\n", checked);
    std::printf("skipped (band): %lld\n", skipped);
    std::printf("mismatches:     %lld\n", mismatches);
    std::printf("special fails:  %d\n", specialFails);
    const bool pass = mismatches == 0 && specialFails == 0 && checked > 1000000;
    std::printf("RESULT: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
