// Adversarial verification of L1 TerrainEstimation (module M3), complementing
// test_terrain_estimation.cpp. Attacks: triage boundaries (exact thresholds),
// layer-step boundary (exactly maxStepHeight vs just above), layer id
// stability/determinism (including per-cell patch insertion order), all-
// STRUCTURE input, 1x1 / 1xN / Nx1 grids, INTERPOLATED bounded to one cell
// (3-cell hole must not be fully filled), single-carrier holes (no
// interpolation below 2 carrier cells), same-cell stacked patches never
// sharing a layer, checkerboard isolation (connectivity defines layers, not
// height equality), and generic per-cell output invariants.
// Build: g++ -O2 -Wall -std=c++14 -I ../../src -I /usr/include/eigen3
//            verify_terrain.cpp ../../src/terrain_field/TerrainEstimation.cpp
//            -o verify_terrain && ./verify_terrain

#include "terrain_field/TerrainEstimation.hpp"

#include <cmath>
#include <cstdio>
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

TerrainGridInput makeGrid(std::size_t w, std::size_t h)
{
    TerrainGridInput in;
    in.width = w;
    in.height = h;
    in.cells.resize(w * h);
    return in;
}

/** Invariants every output must satisfy, regardless of scene. */
void checkInvariants(const TerrainGridOutput& out, const char* scene)
{
    CHECK(out.cells.size() == out.width * out.height,
          "%s: cells size %zu != %zu*%zu", scene, out.cells.size(), out.width,
          out.height);
    for (std::size_t ci = 0; ci < out.cells.size(); ++ci)
    {
        int prevLayer = -1;
        for (const TerrainCell& tc : out.cells[ci])
        {
            CHECK((int)tc.layerId > prevLayer,
                  "%s: cell %zu layers not strictly increasing (%d then %u)",
                  scene, ci, prevLayer, tc.layerId);
            prevLayer = tc.layerId;
            const bool g = (tc.flags & TerrainCell::GROUND) != 0;
            const bool ip = (tc.flags & TerrainCell::INTERPOLATED) != 0;
            CHECK(g != ip, "%s: cell %zu layer %u flags %u not GROUND xor "
                  "INTERPOLATED", scene, ci, tc.layerId, tc.flags);
            CHECK(std::isfinite(tc.height) && std::isfinite(tc.heightSigma) &&
                  std::isfinite(tc.roughness) && std::isfinite(tc.confidence),
                  "%s: cell %zu layer %u non-finite output", scene, ci,
                  tc.layerId);
            CHECK(tc.confidence >= 0.f && tc.confidence <= 1.f,
                  "%s: cell %zu confidence %.4f out of [0,1]", scene, ci,
                  tc.confidence);
            CHECK(std::abs(tc.normal.norm() - 1.f) < 1e-4f,
                  "%s: cell %zu normal not unit (%.6f)", scene, ci,
                  tc.normal.norm());
            CHECK(tc.normal.z() > 0.f, "%s: cell %zu normal not z-up (%.4f)",
                  scene, ci, tc.normal.z());
            CHECK(tc.roughness >= 0.f && tc.heightSigma >= 0.f,
                  "%s: cell %zu negative roughness/sigma", scene, ci);
        }
    }
}

// ---------------------------------------------------------------------------
// 1) TRIAGE BOUNDARIES: thickness exactly 2*maxStepHeight is SUPPORT (strict
//    '>'); slightly thicker is STRUCTURE. |normal.z| exactly structureNormalZ
//    is SUPPORT (strict '<'); slightly below is STRUCTURE.
// ---------------------------------------------------------------------------
void testTriageBoundaries()
{
    std::printf("1) triage boundaries...\n");
    TerrainParams params; // maxStepHeight 0.25 -> thickness limit 0.5;
                          // structureNormalZ 0.5

    struct Case
    {
        PatchSample p;
        bool support;
        const char* name;
    };
    PatchSample thickOk = makeSupport(1.f);
    thickOk.zMin = 0.75f;
    thickOk.zMax = 1.25f; // thickness == 0.5 exactly
    PatchSample thickBad = thickOk;
    thickBad.zMax = 1.26f; // thickness 0.51 > 0.5
    PatchSample nzOk = makeSupport(1.f);
    nzOk.normal = Eigen::Vector3f(std::sqrt(0.75f), 0.f, 0.5f); // z exactly 0.5
    PatchSample nzBad = makeSupport(1.f);
    nzBad.normal = Eigen::Vector3f(0.f, std::sqrt(1.f - 0.49f * 0.49f), 0.49f);

    const Case cases[] = {
        {thickOk, true, "thickness == 2*maxStepHeight"},
        {thickBad, false, "thickness > 2*maxStepHeight"},
        {nzOk, true, "|nz| == structureNormalZ"},
        {nzBad, false, "|nz| < structureNormalZ"},
    };
    for (const Case& c : cases)
    {
        TerrainGridInput in = makeGrid(1, 1);
        in.cells[0].push_back(c.p);
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, c.name);
        CHECK(out.cells[0].size() == (c.support ? 1u : 0u),
              "%s: got %zu cells, want %u", c.name, out.cells[0].size(),
              c.support ? 1u : 0u);
        if (c.support && !out.cells[0].empty())
        {
            const TerrainCell& tc = out.cells[0].front();
            CHECK(std::abs(tc.height - 1.f) < 1e-5f,
                  "%s: height %.4f, want 1", c.name, tc.height);
            CHECK((tc.flags & TerrainCell::GROUND) &&
                  (tc.flags & TerrainCell::SPARSE),
                  "%s: single patch not GROUND|SPARSE (flags %u)", c.name,
                  tc.flags);
        }
    }
}

// ---------------------------------------------------------------------------
// 2) ALL-STRUCTURE input: nothing may be fitted or interpolated anywhere.
// ---------------------------------------------------------------------------
void testAllStructure()
{
    std::printf("2) all-STRUCTURE input...\n");
    TerrainParams params;
    TerrainGridInput in = makeGrid(8, 8);
    for (std::size_t ci = 0; ci < 64; ++ci)
    {
        PatchSample wall = makeSupport(1.f);
        wall.normal = Eigen::Vector3f(1.f, 0.f, 0.1f).normalized();
        PatchSample thick = makeSupport(1.f);
        thick.zMin = 0.f;
        thick.zMax = 2.f;
        in.cells[ci].push_back(wall);
        in.cells[ci].push_back(thick);
    }
    TerrainGridOutput out;
    estimateTerrain(in, params, out);
    checkInvariants(out, "all-structure");
    for (std::size_t ci = 0; ci < 64; ++ci)
        CHECK(out.cells[ci].empty(),
              "all-structure: cell %zu produced %zu terrain cells", ci,
              out.cells[ci].size());
}

// ---------------------------------------------------------------------------
// 3) STEP BOUNDARY: adjacent means differing by exactly maxStepHeight connect
//    (one layer); differing by maxStepHeight + eps split (one layer/column,
//    and cross-column patches must not leak into each other's fits).
// ---------------------------------------------------------------------------
void testStepBoundary()
{
    std::printf("3) step boundary...\n");
    const std::size_t W = 6, H = 3;
    TerrainParams params; // maxStepHeight 0.25

    // exactly at the limit: 0.25 per column -> ONE layer
    {
        TerrainGridInput in = makeGrid(W, H);
        for (std::size_t y = 0; y < H; ++y)
            for (std::size_t x = 0; x < W; ++x)
                in.cells[y * W + x].push_back(makeSupport(0.25f * (float)x));
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, "step==limit");
        std::set<uint16_t> ids;
        for (const auto& cell : out.cells)
        {
            CHECK(cell.size() == 1, "step==limit: %zu layers in a cell",
                  cell.size());
            for (const TerrainCell& tc : cell)
                ids.insert(tc.layerId);
        }
        CHECK(ids.size() == 1, "step==limit: %zu layers, want 1", ids.size());
    }

    // just above the limit: every column its own layer, heights uncontaminated
    {
        TerrainGridInput in = makeGrid(W, H);
        for (std::size_t y = 0; y < H; ++y)
            for (std::size_t x = 0; x < W; ++x)
                in.cells[y * W + x].push_back(makeSupport(0.2501f * (float)x));
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, "step>limit");
        std::set<uint16_t> ids;
        for (std::size_t y = 0; y < H; ++y)
        {
            for (std::size_t x = 0; x < W; ++x)
            {
                const auto& cell = out.cells[y * W + x];
                CHECK(cell.size() == 1,
                      "step>limit: cell (%zu,%zu) has %zu layers", x, y,
                      cell.size());
                for (const TerrainCell& tc : cell)
                {
                    ids.insert(tc.layerId);
                    CHECK(std::abs(tc.height - 0.2501f * (float)x) < 1e-3f,
                          "step>limit: cell (%zu,%zu) height %.4f leaked "
                          "across columns", x, y, tc.height);
                }
            }
        }
        CHECK(ids.size() == W, "step>limit: %zu layers, want %zu", ids.size(),
              W);
    }
}

// ---------------------------------------------------------------------------
// 4) DETERMINISM & LAYER ID STABILITY: identical input twice -> identical
//    output; permuting per-cell patch insertion order -> identical output.
// ---------------------------------------------------------------------------
bool sameOutput(const TerrainGridOutput& a, const TerrainGridOutput& b)
{
    if (a.width != b.width || a.height != b.height ||
        a.cells.size() != b.cells.size())
        return false;
    for (std::size_t ci = 0; ci < a.cells.size(); ++ci)
    {
        if (a.cells[ci].size() != b.cells[ci].size())
            return false;
        for (std::size_t k = 0; k < a.cells[ci].size(); ++k)
        {
            const TerrainCell& x = a.cells[ci][k];
            const TerrainCell& y = b.cells[ci][k];
            if (x.height != y.height || x.heightSigma != y.heightSigma ||
                x.roughness != y.roughness || x.confidence != y.confidence ||
                x.layerId != y.layerId || x.flags != y.flags ||
                x.normal != y.normal)
                return false;
        }
    }
    return true;
}

void testDeterminism()
{
    std::printf("4) determinism / layer id stability...\n");
    const std::size_t W = 12, H = 12;
    TerrainParams params;

    // bridge-like scene with two patches per strip cell
    TerrainGridInput in = makeGrid(W, H);
    TerrainGridInput inPermuted = makeGrid(W, H);
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const PatchSample lo = makeSupport(0.01f * (float)((x + y) % 3));
            in.cells[y * W + x].push_back(lo);
            if (y >= 5 && y <= 7)
            {
                const PatchSample hi = makeSupport(2.f + 0.01f * (float)(x % 2));
                in.cells[y * W + x].push_back(hi);
                // reversed insertion order in the permuted copy
                inPermuted.cells[y * W + x].push_back(hi);
                inPermuted.cells[y * W + x].push_back(lo);
            }
            else
            {
                inPermuted.cells[y * W + x].push_back(lo);
            }
        }
    }

    TerrainGridOutput out1, out2, out3;
    estimateTerrain(in, params, out1);
    estimateTerrain(in, params, out2);
    estimateTerrain(inPermuted, params, out3);
    checkInvariants(out1, "determinism");
    CHECK(sameOutput(out1, out2), "same input produced different outputs");
    CHECK(sameOutput(out1, out3),
          "per-cell patch insertion order changed the output");
}

// ---------------------------------------------------------------------------
// 5) INTERPOLATION BOUNDS: a 3-cell-wide hole is only bridged one cell deep
//    from each side; the middle column must stay empty. A hole with a single
//    carrier cell must NOT be interpolated (needs >= 2 carrier cells).
// ---------------------------------------------------------------------------
void testInterpolationBounds()
{
    std::printf("5) interpolation bounds...\n");
    TerrainParams params;

    // 9x5 ground at z = 1 with columns 3..5 empty
    {
        const std::size_t W = 9, H = 5;
        TerrainGridInput in = makeGrid(W, H);
        for (std::size_t y = 0; y < H; ++y)
            for (std::size_t x = 0; x < W; ++x)
                if (x < 3 || x > 5)
                    in.cells[y * W + x].push_back(makeSupport(1.f));
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, "3-cell hole");
        for (std::size_t y = 0; y < H; ++y)
        {
            for (std::size_t x = 0; x < W; ++x)
            {
                const auto& cell = out.cells[y * W + x];
                if (x == 4)
                {
                    CHECK(cell.empty(),
                          "hole center (%zu,%zu) was filled (%zu cells) -- "
                          "interpolation not bounded to one cell", x, y,
                          cell.size());
                }
                else if (x == 3 || x == 5)
                {
                    CHECK(cell.size() == 1,
                          "hole edge (%zu,%zu) has %zu cells, want 1", x, y,
                          cell.size());
                    for (const TerrainCell& tc : cell)
                    {
                        CHECK(tc.flags & TerrainCell::INTERPOLATED,
                              "hole edge (%zu,%zu) not INTERPOLATED", x, y);
                        CHECK(std::abs(tc.height - 1.f) < 0.02f,
                              "hole edge (%zu,%zu) height %.4f, want 1", x, y,
                              tc.height);
                    }
                }
                else
                {
                    CHECK(cell.size() == 1 &&
                          (cell.front().flags & TerrainCell::GROUND),
                          "ground cell (%zu,%zu) wrong (%zu cells)", x, y,
                          cell.size());
                }
            }
        }
    }

    // 1x2 grid: the empty cell has exactly ONE carrier cell -> no interpolation
    {
        TerrainGridInput in = makeGrid(2, 1);
        in.cells[0].push_back(makeSupport(1.f));
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, "single-carrier hole");
        CHECK(out.cells[0].size() == 1, "carrier cell missing its layer");
        CHECK(out.cells[1].empty(),
              "hole with 1 carrier cell was interpolated (%zu cells)",
              out.cells[1].size());
    }
}

// ---------------------------------------------------------------------------
// 6) DEGENERATE GRIDS: 1x1, 1xN ramp, Nx1 ramp; width/height set but cells
//    vector left empty (robustness of the input guard).
// ---------------------------------------------------------------------------
void testDegenerateGrids()
{
    std::printf("6) degenerate grids...\n");
    TerrainParams params;

    { // 1x1
        TerrainGridInput in = makeGrid(1, 1);
        in.cells[0].push_back(makeSupport(2.5f));
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, "1x1");
        CHECK(out.cells.size() == 1 && out.cells[0].size() == 1,
              "1x1 grid wrong output");
        if (!out.cells[0].empty())
            CHECK(std::abs(out.cells[0].front().height - 2.5f) < 1e-5f,
                  "1x1 height %.4f, want 2.5", out.cells[0].front().height);
    }

    // 1xN and Nx1 ramps at 0.2/cell (< limit): one layer. Interior cells have
    // 3 collinear points -> weighted-mean fallback, exact by symmetry. End
    // cells have n = 2 -> the SPEC-mandated weighted-mean fallback: with own
    // weight 1/(1e-4 + sigma0^2) and neighbor weight 0.6/(...), cell 0 gives
    // (0*1 + 0.2*0.6)/1.6 = 0.075 and cell N-1 gives 1.8 - 0.075 = 1.725.
    for (int orient = 0; orient < 2; ++orient)
    {
        const std::size_t N = 10;
        const std::size_t W = orient == 0 ? N : 1;
        const std::size_t H = orient == 0 ? 1 : N;
        TerrainGridInput in = makeGrid(W, H);
        for (std::size_t k = 0; k < N; ++k)
            in.cells[k].push_back(makeSupport(0.2f * (float)k));
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, orient == 0 ? "1xN ramp" : "Nx1 ramp");
        std::set<uint16_t> ids;
        const float endBias = 0.2f * 0.6f / 1.6f; // = 0.075, see above
        for (std::size_t k = 0; k < N; ++k)
        {
            CHECK(out.cells[k].size() == 1, "%s cell %zu has %zu layers",
                  orient == 0 ? "1xN" : "Nx1", k, out.cells[k].size());
            for (const TerrainCell& tc : out.cells[k])
            {
                ids.insert(tc.layerId);
                float want = 0.2f * (float)k;
                if (k == 0)
                    want += endBias;
                else if (k == N - 1)
                    want -= endBias;
                CHECK(std::abs(tc.height - want) < 1e-3f,
                      "%s cell %zu height %.4f, want %.4f",
                      orient == 0 ? "1xN" : "Nx1", k, tc.height, want);
                CHECK(k != 0 || (tc.flags & TerrainCell::SPARSE),
                      "line-ramp end cell (n = 2) missing SPARSE flag");
            }
        }
        CHECK(ids.size() == 1, "line ramp split into %zu layers", ids.size());
    }

    { // declared 3x3 but empty cells vector: must not crash, no output cells
        TerrainGridInput in;
        in.width = 3;
        in.height = 3;
        TerrainGridOutput out;
        estimateTerrain(in, params, out);
        checkInvariants(out, "missing cells vector");
        for (const auto& cell : out.cells)
            CHECK(cell.empty(), "missing-cells input produced terrain cells");
    }
}

// ---------------------------------------------------------------------------
// 7) SAME-CELL SEPARATION: two patches per cell only 0.1 m apart (well within
//    maxStepHeight) must still form two distinct layers everywhere.
// ---------------------------------------------------------------------------
void testSameCellSeparation()
{
    std::printf("7) same-cell stacked patches...\n");
    const std::size_t W = 5, H = 5;
    TerrainParams params;
    TerrainGridInput in = makeGrid(W, H);
    for (std::size_t ci = 0; ci < W * H; ++ci)
    {
        in.cells[ci].push_back(makeSupport(0.f));
        in.cells[ci].push_back(makeSupport(0.1f));
    }
    TerrainGridOutput out;
    estimateTerrain(in, params, out);
    checkInvariants(out, "same-cell");
    std::set<uint16_t> ids;
    for (std::size_t ci = 0; ci < W * H; ++ci)
    {
        CHECK(out.cells[ci].size() == 2,
              "same-cell: cell %zu has %zu layers, want 2", ci,
              out.cells[ci].size());
        bool near0 = false, near1 = false;
        for (const TerrainCell& tc : out.cells[ci])
        {
            ids.insert(tc.layerId);
            near0 = near0 || std::abs(tc.height - 0.0f) < 0.02f;
            near1 = near1 || std::abs(tc.height - 0.1f) < 0.02f;
        }
        CHECK(near0 && near1,
              "same-cell: cell %zu heights merged/mixed", ci);
    }
    CHECK(ids.size() == 2, "same-cell: %zu layers, want 2", ids.size());
}

// ---------------------------------------------------------------------------
// 8) CHECKERBOARD: equal heights but 4-disconnected cells -> one layer PER
//    support cell (connectivity defines layers, not height equality); empty
//    cells see 3-4 distinct single-carrier layers -> never interpolated.
// ---------------------------------------------------------------------------
void testCheckerboard()
{
    std::printf("8) checkerboard isolation...\n");
    const std::size_t W = 16, H = 16;
    TerrainParams params;
    TerrainGridInput in = makeGrid(W, H);
    std::size_t nSupport = 0;
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            if ((x + y) % 2 == 0)
            {
                in.cells[y * W + x].push_back(makeSupport(1.f));
                ++nSupport;
            }
        }
    }
    TerrainGridOutput out;
    estimateTerrain(in, params, out);
    checkInvariants(out, "checkerboard");
    std::set<uint16_t> ids;
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const auto& cell = out.cells[y * W + x];
            if ((x + y) % 2 == 0)
            {
                CHECK(cell.size() == 1,
                      "checkerboard support (%zu,%zu) has %zu layers", x, y,
                      cell.size());
                for (const TerrainCell& tc : cell)
                {
                    ids.insert(tc.layerId);
                    CHECK((tc.flags & TerrainCell::GROUND) &&
                          (tc.flags & TerrainCell::SPARSE),
                          "checkerboard support (%zu,%zu) flags %u", x, y,
                          tc.flags);
                }
            }
            else
            {
                CHECK(cell.empty(),
                      "checkerboard empty cell (%zu,%zu) interpolated from "
                      "single-carrier layers (%zu cells)", x, y, cell.size());
            }
        }
    }
    CHECK(ids.size() == nSupport,
          "checkerboard: %zu layers, want %zu (one per isolated cell)",
          ids.size(), nSupport);
}

}

int main()
{
    testTriageBoundaries();
    testAllStructure();
    testStepBoundary();
    testDeterminism();
    testInterpolationBounds();
    testDegenerateGrids();
    testSameCellSeparation();
    testCheckerboard();

    if (failures == 0)
    {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL (%d failed checks)\n", failures);
    return 1;
}
