// Adversarial verification of the exact 2D Euclidean distance transform (M1).
//
// Attacks beyond test_esdf.cpp:
//   - EXHAUSTIVE enumeration of every blocked mask for all grid sizes with
//     w*h <= 16 (covers every 1xN/Nx1 strip up to 16 and every grid up to 4x4,
//     including all equidistant-obstacle and envelope-pop corner cases)
//   - long 1xN / Nx1 strips (N up to 401) with sparse, dense and end-only sources
//   - single blocked cell in each of the four corners of a 64x64 grid
//   - two exactly equidistant obstacles (odd and even midpoints)
//   - 64x64 grids: checkerboard, border ring, single diagonal, random densities
//   - larger 257x131 grid with one far-corner source (float exactness stress)
//   - resolution scaling of computeEsdf at several resolutions, including the
//     infinity-preserving behaviour on an all-free grid
//
// Build:
//   g++ -O2 -Wall -Wextra -std=c++14 -I src -I /usr/include/eigen3
//       test/terrain_field/verify_esdf.cpp src/terrain_field/Esdf.cpp -o verify_esdf

#include "terrain_field/Esdf.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

using traversability_generator3d::terrain_field::computeEsdf;
using traversability_generator3d::terrain_field::computeEsdfSquared;

namespace
{

int failures = 0;

/** O(n*m) reference computed entirely in exact integer arithmetic. */
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
            long best = -1;
            for (const auto& s : sources)
            {
                const long dx = (long)x - s.first;
                const long dy = (long)y - s.second;
                const long d2 = dx * dx + dy * dy;
                if (best < 0 || d2 < best)
                    best = d2;
            }
            if (best >= 0)
                out[y * w + x] = (double)best;
        }
    }
}

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

/** Every possible blocked mask for a fixed w x h grid (w*h <= 20 or so). */
void exhaustive(std::size_t w, std::size_t h)
{
    const std::size_t n = w * h;
    std::vector<std::uint8_t> blocked(n);
    const unsigned long masks = 1ul << n;
    for (unsigned long m = 0; m < masks; ++m)
    {
        for (std::size_t i = 0; i < n; ++i)
            blocked[i] = (m >> i) & 1u;
        if (!checkGrid(w, h, blocked, "exhaustive"))
        {
            std::printf("  (mask 0x%lx on %zux%zu)\n", m, w, h);
            return; // one failure per size is enough diagnostics
        }
    }
}

void checkMeters(std::size_t w, std::size_t h,
                 const std::vector<std::uint8_t>& blocked, double resolution)
{
    std::vector<float> squared;
    std::vector<float> meters;
    computeEsdfSquared(w, h, blocked, squared);
    computeEsdf(w, h, blocked, resolution, meters);
    if (meters.size() != w * h)
    {
        std::printf("FAIL [meters] %zux%zu: bad output size %zu\n",
                    w, h, meters.size());
        ++failures;
        return;
    }
    for (std::size_t i = 0; i < w * h; ++i)
    {
        if (std::isinf((double)squared[i]))
        {
            if (!std::isinf((double)meters[i]))
            {
                std::printf("FAIL [meters res=%g] cell %zu: infinity lost "
                            "(got %g)\n", resolution, i, (double)meters[i]);
                ++failures;
                return;
            }
            continue;
        }
        const double want = std::sqrt((double)squared[i]) * resolution;
        if (std::fabs((double)meters[i] - want) > 1e-5)
        {
            std::printf("FAIL [meters res=%g] cell %zu: got %.9g want %.9g\n",
                        resolution, i, (double)meters[i], want);
            ++failures;
            return;
        }
    }
}

}

int main()
{
    // 1) exhaustive: every mask for every grid with w*h <= 16
    for (std::size_t w = 1; w <= 16; ++w)
        for (std::size_t h = 1; h <= 16; ++h)
            if (w * h <= 16)
                exhaustive(w, h);
    std::printf("exhaustive small grids done (%d failures so far)\n", failures);

    std::mt19937 rng(424242u);

    // 2) long strips 1xN and Nx1
    for (std::size_t n : {17u, 63u, 64u, 65u, 128u, 401u})
    {
        std::vector<std::uint8_t> b(n, 0);
        b[0] = 1; // one end only
        checkGrid(n, 1, b, "strip end");
        checkGrid(1, n, b, "strip end (col)");
        b.assign(n, 0);
        b[n - 1] = 1; // other end
        checkGrid(n, 1, b, "strip far end");
        checkGrid(1, n, b, "strip far end (col)");
        b.assign(n, 0);
        b[n / 2] = 1; // middle
        checkGrid(n, 1, b, "strip middle");
        checkGrid(1, n, b, "strip middle (col)");
        std::bernoulli_distribution occ(0.07);
        for (std::size_t i = 0; i < n; ++i)
            b[i] = occ(rng) ? 1 : 0;
        checkGrid(n, 1, b, "strip sparse random");
        checkGrid(1, n, b, "strip sparse random (col)");
    }

    // 3) 64x64: single source in each corner
    {
        const std::size_t W = 64, H = 64;
        const std::size_t corners[4] = {0, W - 1, (H - 1) * W, (H - 1) * W + W - 1};
        for (std::size_t c : corners)
        {
            std::vector<std::uint8_t> b(W * H, 0);
            b[c] = 1;
            checkGrid(W, H, b, "64x64 corner source");
        }
    }

    // 4) two exactly equidistant obstacles (even and odd separations)
    {
        std::vector<std::uint8_t> b(21 * 21, 0);
        b[10 * 21 + 0] = 1;
        b[10 * 21 + 20] = 1; // midpoint x=10 exactly equidistant
        checkGrid(21, 21, b, "equidistant horizontal");
        b.assign(20 * 20, 0);
        b[0] = 1;
        b[19 * 20 + 19] = 1; // diagonal pair, even grid
        checkGrid(20, 20, b, "equidistant diagonal");
    }

    // 5) 64x64 structured + random
    {
        const std::size_t W = 64, H = 64;
        std::vector<std::uint8_t> b(W * H, 0);
        for (std::size_t y = 0; y < H; ++y)
            for (std::size_t x = 0; x < W; ++x)
                b[y * W + x] = ((x + y) & 1) ? 1 : 0;
        checkGrid(W, H, b, "64x64 checkerboard");

        b.assign(W * H, 0);
        for (std::size_t x = 0; x < W; ++x)
        {
            b[x] = 1;
            b[(H - 1) * W + x] = 1;
        }
        for (std::size_t y = 0; y < H; ++y)
        {
            b[y * W] = 1;
            b[y * W + W - 1] = 1;
        }
        checkGrid(W, H, b, "64x64 border ring");

        b.assign(W * H, 0);
        for (std::size_t i = 0; i < W; ++i)
            b[i * W + i] = 1;
        checkGrid(W, H, b, "64x64 diagonal");

        for (double density : {0.001, 0.01, 0.05, 0.5, 0.98})
        {
            std::bernoulli_distribution occ(density);
            b.assign(W * H, 0);
            for (std::size_t i = 0; i < W * H; ++i)
                b[i] = occ(rng) ? 1 : 0;
            checkGrid(W, H, b, "64x64 random");
        }
    }

    // 6) larger non-square grid, single far-corner source (float exactness)
    {
        const std::size_t W = 257, H = 131;
        std::vector<std::uint8_t> b(W * H, 0);
        b[(H - 1) * W + (W - 1)] = 1;
        checkGrid(W, H, b, "257x131 far corner");
        std::bernoulli_distribution occ(0.003);
        b.assign(W * H, 0);
        for (std::size_t i = 0; i < W * H; ++i)
            b[i] = occ(rng) ? 1 : 0;
        checkGrid(W, H, b, "257x131 sparse random");
    }

    // 7) resolution scaling, incl. infinity preservation on all-free grid
    {
        std::vector<std::uint8_t> b(32 * 24, 0);
        std::bernoulli_distribution occ(0.1);
        for (std::size_t i = 0; i < b.size(); ++i)
            b[i] = occ(rng) ? 1 : 0;
        for (double res : {1.0, 0.05, 0.3, 2.5})
            checkMeters(32, 24, b, res);
        b.assign(32 * 24, 0); // all free: everything must stay infinity
        for (double res : {1.0, 0.05})
            checkMeters(32, 24, b, res);
    }

    if (failures == 0)
    {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("%d failure(s)\nRESULT: FAIL\n", failures);
    return 1;
}
