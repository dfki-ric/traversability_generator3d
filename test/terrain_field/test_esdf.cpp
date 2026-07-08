// Brute-force verification of the exact 2D Euclidean distance transform (M1).
//
// computeEsdfSquared is compared against an O(n*m) brute force on >= 250 random
// grids (sizes 1..48 x 1..48, blocked densities 0, tiny, 0.1..0.9, 1.0) plus
// deterministic edge cases. Squared distances between integer grid points are
// integers, so equality must be EXACT (compared as double, tolerance 1e-6).
// computeEsdf is additionally checked to equal sqrt(squared) * resolution.
//
// Build:
//   g++ -O2 -Wall -std=c++14 -I src -I /usr/include/eigen3
//       test/terrain_field/test_esdf.cpp src/terrain_field/Esdf.cpp -o test_esdf

#include "terrain_field/Esdf.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

using traversability_generator3d::terrain_field::computeEsdf;
using traversability_generator3d::terrain_field::computeEsdfSquared;

namespace
{

int failures = 0;

/** O(n*m) reference: per cell, minimum squared distance over all blocked cells. */
void bruteForceSquared(std::size_t w, std::size_t h,
                       const std::vector<std::uint8_t>& blocked,
                       std::vector<double>& out)
{
    std::vector<std::pair<long, long>> sources;
    for (std::size_t y = 0; y < h; ++y)
        for (std::size_t x = 0; x < w; ++x)
            if (blocked[y * w + x])
                sources.emplace_back((long)x, (long)y);

    out.assign(w * h, std::numeric_limits<double>::infinity());
    for (std::size_t y = 0; y < h; ++y)
    {
        for (std::size_t x = 0; x < w; ++x)
        {
            double best = std::numeric_limits<double>::infinity();
            for (const auto& s : sources)
            {
                const double dx = (double)((long)x - s.first);
                const double dy = (double)((long)y - s.second);
                const double d2 = dx * dx + dy * dy;
                if (d2 < best)
                    best = d2;
            }
            out[y * w + x] = best;
        }
    }
}

/** Compare computeEsdfSquared to the brute force; exact equality required. */
bool checkGrid(std::size_t w, std::size_t h,
               const std::vector<std::uint8_t>& blocked, const char* label)
{
    std::vector<float> got;
    computeEsdfSquared(w, h, blocked, got);

    if (got.size() != w * h)
    {
        std::printf("FAIL [%s] %zux%zu: output size %zu != %zu\n",
                    label, w, h, got.size(), w * h);
        ++failures;
        return false;
    }

    std::vector<double> expected;
    bruteForceSquared(w, h, blocked, expected);

    for (std::size_t i = 0; i < w * h; ++i)
    {
        const bool expInf = std::isinf(expected[i]);
        const bool gotInf = std::isinf((double)got[i]);
        if (expInf != gotInf ||
            (!expInf && std::fabs((double)got[i] - expected[i]) > 1e-6))
        {
            std::printf("FAIL [%s] %zux%zu cell (%zu,%zu): got %g expected %g\n",
                        label, w, h, i % w, i / w, (double)got[i], expected[i]);
            ++failures;
            return false;
        }
    }
    return true;
}

/** Check computeEsdf == sqrt(squared) * resolution cell-by-cell. */
bool checkMeters(std::size_t w, std::size_t h,
                 const std::vector<std::uint8_t>& blocked, double resolution)
{
    std::vector<float> squared;
    std::vector<float> meters;
    computeEsdfSquared(w, h, blocked, squared);
    computeEsdf(w, h, blocked, resolution, meters);

    if (meters.size() != w * h)
    {
        std::printf("FAIL [meters] %zux%zu: output size %zu != %zu\n",
                    w, h, meters.size(), w * h);
        ++failures;
        return false;
    }

    for (std::size_t i = 0; i < w * h; ++i)
    {
        const bool sqInf = std::isinf((double)squared[i]);
        const bool mInf = std::isinf((double)meters[i]);
        if (sqInf != mInf)
        {
            std::printf("FAIL [meters res=%g] %zux%zu cell %zu: infinity "
                        "mismatch (squared %g, meters %g)\n",
                        resolution, w, h, i, (double)squared[i],
                        (double)meters[i]);
            ++failures;
            return false;
        }
        if (sqInf)
            continue;
        const double want = std::sqrt((double)squared[i]) * resolution;
        if (std::fabs((double)meters[i] - want) > 1e-6)
        {
            std::printf("FAIL [meters res=%g] %zux%zu cell %zu: got %.9g "
                        "expected %.9g\n",
                        resolution, w, h, i, (double)meters[i], want);
            ++failures;
            return false;
        }
    }
    return true;
}

}

int main()
{
    std::mt19937 rng(20260707u);
    std::uniform_int_distribution<std::size_t> sizeDist(1, 48);

    // blocked densities: 0, "tiny" (exactly one blocked cell), 0.1..0.9, 1.0
    const double TINY = -1.0; // marker
    const double densities[] = {0.0, TINY,  0.1, 0.2, 0.3, 0.4,
                                0.5, 0.6,   0.7, 0.8, 0.9, 1.0};
    const std::size_t numDensities = sizeof(densities) / sizeof(densities[0]);

    // deterministic edge cases: 1x1, single row, single column; all-free/all-blocked
    {
        std::vector<std::uint8_t> b;
        b = {0};
        checkGrid(1, 1, b, "edge 1x1 free");
        b = {1};
        checkGrid(1, 1, b, "edge 1x1 blocked");
        b = {0, 0, 0, 1, 0, 0, 0};
        checkGrid(7, 1, b, "edge 7x1");
        checkGrid(1, 7, b, "edge 1x7");
        b.assign(48 * 48, 0);
        checkGrid(48, 48, b, "edge 48x48 all-free");
        b.assign(48 * 48, 1);
        checkGrid(48, 48, b, "edge 48x48 all-blocked");
        b.assign(48 * 48, 0);
        b[0] = 1; // single corner source
        checkGrid(48, 48, b, "edge 48x48 corner source");
    }

    // randomized brute-force comparison
    const int numRandomGrids = 300;
    int checked = 0;
    for (int t = 0; t < numRandomGrids; ++t)
    {
        const std::size_t w = sizeDist(rng);
        const std::size_t h = sizeDist(rng);
        const double density = densities[t % numDensities];

        std::vector<std::uint8_t> blocked(w * h, 0);
        if (density == 1.0)
        {
            blocked.assign(w * h, 1);
        }
        else if (density == TINY)
        {
            std::uniform_int_distribution<std::size_t> cell(0, w * h - 1);
            blocked[cell(rng)] = 1;
        }
        else if (density > 0.0)
        {
            std::bernoulli_distribution occ(density);
            for (std::size_t i = 0; i < w * h; ++i)
                blocked[i] = occ(rng) ? 1 : 0;
        }

        checkGrid(w, h, blocked, "random");
        ++checked;

        // spot-check the metric variant on a subset, two resolutions
        if (t % 25 == 0)
        {
            checkMeters(w, h, blocked, 0.5);
            checkMeters(w, h, blocked, 0.13);
        }
    }

    std::printf("checked %d random grids + edge cases, %d failure(s)\n",
                checked, failures);
    if (failures == 0)
    {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL\n");
    return 1;
}
