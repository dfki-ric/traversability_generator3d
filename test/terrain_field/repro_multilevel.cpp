// Offline reproduction of the multi-level (parking-deck) failure.
// Two flat decks (lower z=0 everywhere, upper z=3 over the right half) connected
// by a ramp, with the lower floor continuing UNDER the upper deck (as in a real
// deck). We inspect how estimateTerrain assigns layers and how TerrainField
// classifies the overlap region.
//
// Build: g++ -O2 -std=c++14 -I ../../src -I /usr/include/eigen3 repro_multilevel.cpp \
//   ../../src/terrain_field/*.cpp -o repro && ./repro

#include "terrain_field/TerrainField.hpp"

#include <cmath>
#include <cstdio>
#include <map>
#include <set>

using namespace traversability_generator3d::terrain_field;

static PatchSample flat(float z)
{
    PatchSample s;
    s.zMin = z - 0.02f;
    s.zMax = z + 0.02f;
    s.mean = z;
    s.normal = Eigen::Vector3f(0, 0, 1);
    s.variance = 0.0004f;
    return s;
}

static void report(const char* name, TerrainField& field, std::size_t W, std::size_t H,
                   const TerrainParams& params)
{
    std::map<uint16_t, std::pair<float, float>> layerZ;
    std::map<uint16_t, int> layerCount;
    for (std::size_t i = 0; i < W * H; ++i)
        for (const TerrainCell& tc : field.terrain().cells[i])
        {
            auto& z = layerZ[tc.layerId];
            if (layerCount[tc.layerId]++ == 0) z = {tc.height, tc.height};
            else { z.first = std::min(z.first, tc.height); z.second = std::max(z.second, tc.height); }
        }
    std::printf("[%s] layers=%zu\n", name, layerZ.size());
    for (const auto& kv : layerZ)
    {
        const float span = kv.second.second - kv.second.first;
        std::printf("   layer %u: cells=%d z=[%.2f,%.2f] span=%.2f%s\n",
                    kv.first, layerCount[kv.first], kv.second.first, kv.second.second, span,
                    span > 3 * params.maxStepHeight ? "  <-- MULTI-LEVEL MERGE" : "");
    }
}

// count how many distinct layer ids carry z near a target height (fragmentation of
// one physical flat level across ids)
static int layersNearHeight(TerrainField& field, std::size_t W, std::size_t H,
                            double z, double tol)
{
    std::set<uint16_t> ids;
    for (std::size_t i = 0; i < W * H; ++i)
        for (const TerrainCell& tc : field.terrain().cells[i])
            if (std::fabs(tc.height - z) < tol)
                ids.insert(tc.layerId);
    return (int)ids.size();
}

static void scenarioMultiStack()
{
    // 3 flat levels z=0,3,6 stacked over the right half, each connected UP by its own
    // ramp lane; lower floor continuous everywhere. Tests 3+ level fragmentation.
    const std::size_t W = 60, H = 20;
    const double RES = 0.5;
    TerrainParams params; params.gridResolution = RES; params.maxStepHeight = 0.45;
    TerrainGridInput in; in.width = W; in.height = H; in.cells.assign(W * H, {});
    const double zL1 = 3.0, zL2 = 6.0;
    for (std::size_t y = 0; y < H; ++y)
        for (std::size_t x = 0; x < W; ++x)
        {
            const std::size_t i = y * W + x;
            in.cells[i].push_back(flat(0.f));                    // lower floor
            if (x >= 20) in.cells[i].push_back(flat((float)zL1));// level 1 over right 2/3
            if (x >= 40) in.cells[i].push_back(flat((float)zL2));// level 2 over right 1/3
            // ramp 0->3 in y[8,11] x[6,19]
            if (y >= 8 && y <= 11 && x >= 6 && x <= 19)
                in.cells[i].push_back(flat((float)(zL1 * (x - 5) / 14.0)));
            // ramp 3->6 in y[8,11] x[26,39]
            if (y >= 8 && y <= 11 && x >= 26 && x <= 39)
                in.cells[i].push_back(flat((float)(zL1 + zL1 * (x - 25) / 14.0)));
        }
    TerrainField field; field.compute(in, params, params.maxStepHeight, 2.0);
    report("3-level stack", field, W, H, params);
    // Count layer ids over a FLAT, ramp-free patch of level 1 (x in [20,25], all y is
    // level1 z=3 with no ramp lane and no level2 above) with tight tolerance, so ramp
    // bases near z=3 don't create false positives.
    auto idsInFlatRegion = [&](std::size_t x0, std::size_t x1, double z) {
        std::set<uint16_t> ids;
        for (std::size_t y = 0; y < H; ++y)
            for (std::size_t x = x0; x <= x1; ++x)
                for (const TerrainCell& tc : field.terrain().cells[y * W + x])
                    if (std::fabs(tc.height - z) < 0.1) ids.insert(tc.layerId);
        return (int)ids.size();
    };
    const int l1ids = idsInFlatRegion(20, 25, 3.0);
    const int l2ids = idsInFlatRegion(44, 59, 6.0);
    std::printf("   flat level-1 (x20-25,z=3) layer ids (should be 1): %d %s\n", l1ids,
                l1ids > 1 ? "<-- LEVEL FRAGMENTED" : "(OK)");
    std::printf("   flat level-2 (x44-59,z=6) layer ids (should be 1): %d %s\n", l2ids,
                l2ids > 1 ? "<-- LEVEL FRAGMENTED" : "(OK)");
}

static void scenarioTightClearance(double clearance)
{
    // lower floor + upper slab directly above at `clearance` metres, no ramp.
    // With robotHeight=2.0 the overhead band is [0.45, 2.45]; a slab within that
    // band blocks the entire lower floor beneath it.
    const std::size_t W = 30, H = 20; const double RES = 0.5;
    TerrainParams params; params.gridResolution = RES; params.maxStepHeight = 0.45;
    TerrainGridInput in; in.width = W; in.height = H; in.cells.assign(W * H, {});
    for (std::size_t y = 0; y < H; ++y)
        for (std::size_t x = 0; x < W; ++x)
        {
            const std::size_t i = y * W + x;
            in.cells[i].push_back(flat(0.f));
            if (x >= 15) in.cells[i].push_back(flat((float)clearance)); // upper slab, right half
        }
    TerrainField field; field.compute(in, params, params.maxStepHeight, 2.0);
    // count lower-floor cells (z~0) blocked under the slab (x>=15)
    int blockedUnder = 0, totalUnder = 0;
    for (std::size_t y = 0; y < H; ++y)
        for (std::size_t x = 15; x < W; ++x)
        {
            for (const TerrainCell& tc : field.terrain().cells[y * W + x])
                if (std::fabs(tc.height) < 0.3)
                {
                    ++totalUnder;
                    const TerrainField::LayerField* lf = field.layerField(tc.layerId);
                    if (lf && lf->blocked[y * W + x]) ++blockedUnder;
                }
        }
    std::printf("[tight clearance %.2f m, band top = robot body height 2.0] lower-floor cells under slab "
                "blocked: %d/%d %s\n", clearance, blockedUnder, totalUnder,
                blockedUnder > 0 ? "<-- lower level wiped by overhead" : "(clear)");
}

int main()
{
    const std::size_t W = 48, H = 20;
    const double RES = 0.5;
    TerrainParams params;
    params.gridResolution = RES;
    params.maxStepHeight = 0.45;

    TerrainGridInput in;
    in.width = W;
    in.height = H;
    in.cells.assign(W * H, {});

    const double zUpper = 3.0;
    const std::size_t upperX0 = 30; // upper deck covers x in [30, 47]
    const std::size_t rampX0 = 16, rampX1 = 29; // ramp lane rises here, y in [8,11]

    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const std::size_t i = y * W + x;

            // lower floor z=0 everywhere (continuous, incl. under the upper deck)
            in.cells[i].push_back(flat(0.f));

            // ramp lane y in [8,11], x in [rampX0, rampX1], rising 0 -> 3.0
            const bool rampLane = (y >= 8 && y <= 11 && x >= rampX0 && x <= rampX1);
            if (rampLane)
            {
                const double t = (double)(x - rampX0 + 1) / (double)(rampX1 - rampX0 + 1);
                in.cells[i].push_back(flat((float)(zUpper * t)));
            }

            // upper deck z=3 over the right region
            if (x >= upperX0)
                in.cells[i].push_back(flat((float)zUpper));
        }
    }

    TerrainField field;
    field.compute(in, params, params.maxStepHeight, 2.0);

    // --- layer statistics ---
    std::map<uint16_t, std::pair<float, float>> layerZ; // id -> (min,max)
    std::map<uint16_t, int> layerCount;
    int multiLayerCells = 0;
    for (std::size_t i = 0; i < W * H; ++i)
    {
        const auto& cl = field.terrain().cells[i];
        if (cl.size() > 1)
            ++multiLayerCells;
        for (const TerrainCell& tc : cl)
        {
            auto& z = layerZ[tc.layerId];
            if (layerCount[tc.layerId]++ == 0)
                z = {tc.height, tc.height};
            else
            {
                z.first = std::min(z.first, tc.height);
                z.second = std::max(z.second, tc.height);
            }
        }
    }

    std::printf("grid %zux%zu, layers=%zu, cells-with->1-layer=%d\n",
                W, H, layerZ.size(), multiLayerCells);
    for (const auto& kv : layerZ)
    {
        const float span = kv.second.second - kv.second.first;
        std::printf("  layer %u: cells=%d  z=[%.2f, %.2f]  span=%.2f %s\n",
                    kv.first, layerCount[kv.first], kv.second.first, kv.second.second,
                    span, span > 3 * params.maxStepHeight ? "  <-- SPANS MULTIPLE LEVELS" : "");
    }

    // --- is the lower floor ONE layer, or fragmented? ---
    // sample the lower (z~0) layer id at the far-left and far-right of the deck
    auto lowerLayerAt = [&](std::size_t x, std::size_t y) -> int
    {
        for (const TerrainCell& tc : field.terrain().cells[y * W + x])
            if (std::fabs(tc.height) < 0.3)
                return (int)tc.layerId;
        return -1;
    };
    const int lowerLeft = lowerLayerAt(2, 2);
    const int lowerRightUnderUpper = lowerLayerAt(W - 3, 2);
    std::printf("\nlower-floor layer id  left(2,2)=%d   right-under-upper(%zu,2)=%d  %s\n",
                lowerLeft, W - 3, lowerRightUnderUpper,
                (lowerLeft == lowerRightUnderUpper && lowerLeft >= 0)
                    ? "(same layer - OK)"
                    : "<-- FRAGMENTED: same physical floor split across layers");

    // --- overlap column: what layers exist directly under the upper deck? ---
    const std::size_t ox = W - 3, oy = 2;
    std::printf("\noverlap column (%zu,%zu) heights present:", ox, oy);
    for (const TerrainCell& tc : field.terrain().cells[oy * W + ox])
        std::printf(" [layer %u z=%.2f]", tc.layerId, tc.height);
    std::printf("\n");

    // --- ESDF sanity on the lower floor far from any real edge ---
    // (10,10) is deep inside the lower floor; should have large clearance
    if (lowerLeft >= 0)
    {
        const TerrainField::LayerField* lf = field.layerField((uint16_t)lowerLeft);
        if (lf)
            std::printf("\nlower-floor ESDF at deep-interior (10,10) = %.2f m "
                        "(large expected; small => void treated as obstacle)\n",
                        lf->esdf[10 * W + 10]);
    }

    std::printf("\n=== scenario B: 3-level stack (ramp-merge fragmentation) ===\n");
    scenarioMultiStack();

    std::printf("\n=== scenario C: tight floor-to-ceiling clearance ===\n");
    scenarioTightClearance(3.0);
    scenarioTightClearance(2.4);
    scenarioTightClearance(2.2);
    scenarioTightClearance(1.8);

    return 0;
}
