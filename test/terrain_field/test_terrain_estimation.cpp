// Synthetic-scene verification of L1 TerrainEstimation (module M3).
// Scenes: wall-pull regression, ramp continuity, bridge two-layers, single
// outlier patch robustness (TERRAIN_FIELD_ARCHITECTURE.md §5.1/§5.2, §7).
// Build: g++ -O2 -Wall -std=c++14 -I ../../src -I /usr/include/eigen3
//            test_terrain_estimation.cpp ../../src/terrain_field/TerrainEstimation.cpp
//            -o test_te && ./test_te

#include "terrain_field/TerrainEstimation.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <set>
#include <vector>

using namespace traversability_generator3d::terrain_field;

namespace
{

int failures = 0;

#define CHECK(cond, ...)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(cond))                                                            \
        {                                                                       \
            std::printf("FAIL(%s:%d): ", __FILE__, __LINE__);                   \
            std::printf(__VA_ARGS__);                                           \
            std::printf("\n");                                                  \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

PatchSample makeSupport(float z, float variance = 1e-4f, float halfThick = 0.01f,
                        const Eigen::Vector3f& normal = Eigen::Vector3f(0.f, 0.f, 1.f))
{
    PatchSample p;
    p.mean = z;
    p.zMin = z - halfThick;
    p.zMax = z + halfThick;
    p.normal = normal.normalized();
    p.variance = variance;
    return p;
}

PatchSample makeWall(float zMin, float zMax)
{
    PatchSample p;
    p.mean = 0.5f * (zMin + zMax);
    p.zMin = zMin;
    p.zMax = zMax;
    p.normal = Eigen::Vector3f(1.f, 0.f, 0.1f).normalized(); // near-horizontal
    p.variance = 0.01f;
    return p;
}

TerrainGridInput makeGrid(std::size_t w, std::size_t h)
{
    TerrainGridInput in;
    in.width = w;
    in.height = h;
    in.cells.resize(w * h);
    return in;
}

// ---------------------------------------------------------------------------
// A) WALL-PULL: flat ground at z = 1 plus a wall line (STRUCTURE patches with
//    near-horizontal normals, zMin = 1, zMax = 3). The fit must ignore the wall
//    entirely; a naive mean over all 3x3 patch means must NOT (test teeth).
// ---------------------------------------------------------------------------
void testWallPull()
{
    std::printf("A) wall-pull...\n");
    const std::size_t W = 20, H = 20, wallCol = 10;
    TerrainParams params; // res 0.5, maxStepHeight 0.25, structureNormalZ 0.5

    TerrainGridInput in = makeGrid(W, H);
    std::mt19937 rng(1234);
    std::normal_distribution<float> noise(0.f, 0.005f);
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            if (x == wallCol)
                in.cells[y * W + x].push_back(makeWall(1.f, 3.f));
            else
                in.cells[y * W + x].push_back(
                    makeSupport(1.f + noise(rng), 0.005f * 0.005f));
        }
    }

    TerrainGridOutput out;
    estimateTerrain(in, params, out);

    // every produced (cell, layer) height must sit on the ground plane
    double maxErr = 0.0;
    for (std::size_t ci = 0; ci < W * H; ++ci)
    {
        CHECK(!out.cells[ci].empty(), "cell %zu produced no terrain cell", ci);
        for (const TerrainCell& tc : out.cells[ci])
            maxErr = std::max(maxErr, std::abs((double)tc.height - 1.0));
    }
    std::printf("   robust max |height - 1| = %.5f\n", maxErr);
    CHECK(maxErr < 0.02, "wall pulled the ground fit: max err %.4f >= 0.02", maxErr);

    // wall cells have no SUPPORT patches -> interpolated from the neighborhood
    for (std::size_t y = 0; y < H; ++y)
    {
        const auto& cells = out.cells[y * W + wallCol];
        CHECK(!cells.empty(), "wall cell (10,%zu) not interpolated", y);
        for (const TerrainCell& tc : cells)
        {
            CHECK(tc.flags & TerrainCell::INTERPOLATED,
                  "wall cell (10,%zu) missing INTERPOLATED flag", y);
            CHECK(!(tc.flags & TerrainCell::GROUND),
                  "wall cell (10,%zu) wrongly flagged GROUND", y);
        }
    }

    // teeth: the naive unweighted mean of ALL patch means in the 3x3 (i.e.
    // including the wall patches) errs badly on wall-adjacent cells
    double minNaiveErr = 1e9;
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            if (x != wallCol - 1 && x != wallCol + 1)
                continue;
            double sum = 0.0;
            int n = 0;
            for (int jy = (int)y - 1; jy <= (int)y + 1; ++jy)
            {
                for (int jx = (int)x - 1; jx <= (int)x + 1; ++jx)
                {
                    if (jx < 0 || jy < 0 || jx >= (int)W || jy >= (int)H)
                        continue;
                    for (const PatchSample& p : in.cells[(std::size_t)jy * W + jx])
                    {
                        sum += p.mean;
                        ++n;
                    }
                }
            }
            minNaiveErr = std::min(minNaiveErr, std::abs(sum / n - 1.0));
        }
    }
    std::printf("   naive  min |height - 1| = %.5f on wall-adjacent cells\n",
                minNaiveErr);
    CHECK(minNaiveErr > 0.1,
          "naive fit err %.4f <= 0.1 -- the wall-pull test has no teeth",
          minNaiveErr);
}

// ---------------------------------------------------------------------------
// B) RAMP: 0.08 m rise per cell (< maxStepHeight) -> exactly one layer;
//    fitted normals tilt in the ramp direction; heights track the ramp.
// ---------------------------------------------------------------------------
void testRamp()
{
    std::printf("B) ramp...\n");
    const std::size_t W = 30, H = 10;
    TerrainParams params; // res 0.5 -> slope dz/dx = 0.08 / 0.5 = 0.16
    const double risePerCell = 0.08;
    const double slope = risePerCell / params.gridResolution;
    const Eigen::Vector3f expectedNormal =
        Eigen::Vector3f((float)-slope, 0.f, 1.f).normalized();

    TerrainGridInput in = makeGrid(W, H);
    for (std::size_t y = 0; y < H; ++y)
        for (std::size_t x = 0; x < W; ++x)
            in.cells[y * W + x].push_back(
                makeSupport((float)(risePerCell * (double)x), 1e-4f, 0.01f,
                            expectedNormal));

    TerrainGridOutput out;
    estimateTerrain(in, params, out);

    std::set<uint16_t> layerIds;
    double maxHeightErr = 0.0, minDot = 1.0;
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const auto& cells = out.cells[y * W + x];
            CHECK(cells.size() == 1, "ramp cell (%zu,%zu) has %zu layers, want 1",
                  x, y, cells.size());
            for (const TerrainCell& tc : cells)
            {
                layerIds.insert(tc.layerId);
                maxHeightErr = std::max(
                    maxHeightErr,
                    std::abs((double)tc.height - risePerCell * (double)x));
                minDot = std::min(minDot, (double)tc.normal.dot(expectedNormal));
            }
        }
    }
    std::printf("   layers = %zu, max height err = %.5f, min normal dot = %.5f\n",
                layerIds.size(), maxHeightErr, minDot);
    CHECK(layerIds.size() == 1, "ramp split into %zu layers", layerIds.size());
    CHECK(maxHeightErr < 0.02, "ramp height err %.4f >= 0.02", maxHeightErr);
    CHECK(minDot > 0.99, "ramp normal dot %.4f <= 0.99", minDot);
}

// ---------------------------------------------------------------------------
// C) BRIDGE: ground at z = 0 everywhere plus a 3-cell-wide strip at z = 2 ->
//    two layers on strip cells, one elsewhere; no layer mixes heights.
// ---------------------------------------------------------------------------
void testBridge()
{
    std::printf("C) bridge...\n");
    const std::size_t W = 21, H = 21;
    const std::size_t strip0 = 9, strip1 = 11; // rows of the bridge strip
    TerrainParams params;

    TerrainGridInput in = makeGrid(W, H);
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            in.cells[y * W + x].push_back(makeSupport(0.f));
            if (y >= strip0 && y <= strip1)
                in.cells[y * W + x].push_back(makeSupport(2.f));
        }
    }

    TerrainGridOutput out;
    estimateTerrain(in, params, out);

    std::set<uint16_t> layerIds;
    // layerId -> reference surface height (-1 = unseen)
    double layerSurface[65536];
    std::fill(layerSurface, layerSurface + 65536, -1.0);

    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const bool onStrip = (y >= strip0 && y <= strip1);
            const auto& cells = out.cells[y * W + x];
            CHECK(cells.size() == (onStrip ? 2u : 1u),
                  "bridge cell (%zu,%zu) has %zu layers, want %u", x, y,
                  cells.size(), onStrip ? 2u : 1u);
            for (const TerrainCell& tc : cells)
            {
                layerIds.insert(tc.layerId);
                const bool near0 = std::abs((double)tc.height - 0.0) < 0.05;
                const bool near2 = std::abs((double)tc.height - 2.0) < 0.05;
                CHECK(near0 || near2,
                      "cell (%zu,%zu) layer %u height %.3f off both surfaces",
                      x, y, tc.layerId, tc.height);
                const double surface = near0 ? 0.0 : 2.0;
                double& ref = layerSurface[tc.layerId];
                if (ref < 0.0)
                    ref = surface;
                CHECK(ref == surface,
                      "layer %u mixes heights: %.3f vs surface %.1f",
                      tc.layerId, tc.height, ref);
                CHECK(tc.flags & TerrainCell::GROUND,
                      "bridge cell (%zu,%zu) not GROUND", x, y);
            }
        }
    }
    std::printf("   distinct layers = %zu\n", layerIds.size());
    CHECK(layerIds.size() == 2, "bridge scene has %zu layers, want 2",
          layerIds.size());
}

// ---------------------------------------------------------------------------
// D) OUTLIER: flat ground, one cell carries an extra low-variance SUPPORT
//    patch at z = 1.6. It must form its own (sparse) layer and must not drag
//    the ground fit of the cell or its neighbors.
// ---------------------------------------------------------------------------
void testOutlier()
{
    std::printf("D) outlier...\n");
    const std::size_t W = 15, H = 15, ox = 7, oy = 7;
    TerrainParams params;

    TerrainGridInput in = makeGrid(W, H);
    for (std::size_t y = 0; y < H; ++y)
        for (std::size_t x = 0; x < W; ++x)
            in.cells[y * W + x].push_back(makeSupport(1.f));
    in.cells[oy * W + ox].push_back(makeSupport(1.6f, 1e-6f));

    TerrainGridOutput out;
    estimateTerrain(in, params, out);

    uint16_t groundLayer = out.cells[0].front().layerId;
    double maxGroundErr = 0.0;
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const std::size_t ci = y * W + x;
            const bool isOutlierCell = (x == ox && y == oy);
            CHECK(out.cells[ci].size() == (isOutlierCell ? 2u : 1u),
                  "outlier scene cell (%zu,%zu) has %zu layers", x, y,
                  out.cells[ci].size());
            bool haveGround = false;
            for (const TerrainCell& tc : out.cells[ci])
            {
                if (tc.layerId == groundLayer)
                {
                    haveGround = true;
                    maxGroundErr = std::max(
                        maxGroundErr, std::abs((double)tc.height - 1.0));
                }
                else
                {
                    // the outlier's own single-patch layer
                    CHECK(isOutlierCell, "unexpected extra layer at (%zu,%zu)",
                          x, y);
                    CHECK(std::abs((double)tc.height - 1.6) < 0.01,
                          "outlier layer height %.3f, want 1.6", tc.height);
                    CHECK(tc.flags & TerrainCell::SPARSE,
                          "outlier layer missing SPARSE flag");
                    CHECK(tc.flags & TerrainCell::GROUND,
                          "outlier layer missing GROUND flag");
                }
            }
            CHECK(haveGround, "cell (%zu,%zu) lost its ground layer", x, y);
        }
    }
    std::printf("   max ground |height - 1| = %.5f\n", maxGroundErr);
    CHECK(maxGroundErr < 0.03, "outlier dragged ground fit: err %.4f >= 0.03",
          maxGroundErr);
}

// ---------------------------------------------------------------------------
// E) IN-LAYER BUMP: exercises the Tukey reweight itself. A low-variance patch
//    at z = 1.2 stays within maxStepHeight of the z = 1 ground, so it JOINS
//    the ground layer -- the robust reweight (not layer building) must shed it.
// ---------------------------------------------------------------------------
void testInLayerBump()
{
    std::printf("E) in-layer bump (robust reweight)...\n");
    const std::size_t W = 15, H = 15, bx = 7, by = 7;
    TerrainParams params;

    TerrainGridInput in = makeGrid(W, H);
    std::mt19937 rng(77);
    std::normal_distribution<float> noise(0.f, 0.005f);
    for (std::size_t y = 0; y < H; ++y)
        for (std::size_t x = 0; x < W; ++x)
            in.cells[y * W + x].push_back(
                makeSupport(1.f + noise(rng), 0.005f * 0.005f));
    in.cells[by * W + bx].front() = makeSupport(1.2f, 1e-6f); // joins the layer

    TerrainGridOutput out;
    estimateTerrain(in, params, out);

    std::set<uint16_t> layerIds;
    double maxNeighborErr = 0.0;
    for (std::size_t y = by - 1; y <= by + 1; ++y)
    {
        for (std::size_t x = bx - 1; x <= bx + 1; ++x)
        {
            const auto& cells = out.cells[y * W + x];
            CHECK(cells.size() == 1, "bump scene cell (%zu,%zu) has %zu layers",
                  x, y, cells.size());
            layerIds.insert(cells.front().layerId);
            if (x == bx && y == by)
                continue;
            maxNeighborErr = std::max(
                maxNeighborErr, std::abs((double)cells.front().height - 1.0));
        }
    }
    std::printf("   max neighbor |height - 1| = %.5f\n", maxNeighborErr);
    CHECK(layerIds.size() == 1, "bump split the ground into %zu layers",
          layerIds.size());
    CHECK(maxNeighborErr < 0.03,
          "in-layer bump dragged neighbors: err %.4f >= 0.03", maxNeighborErr);
}

// generic sanity over all scenes' outputs is embedded above; a tiny extra:
void testEmpty()
{
    TerrainParams params;
    TerrainGridInput in; // 0 x 0
    TerrainGridOutput out;
    estimateTerrain(in, params, out);
    CHECK(out.cells.empty(), "empty input produced cells");
}

}

int main()
{
    testWallPull();
    testRamp();
    testBridge();
    testOutlier();
    testInLayerBump();
    testEmpty();

    if (failures == 0)
    {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL (%d failed checks)\n", failures);
    return 1;
}
