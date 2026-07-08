// Cross-module integration test of the TerrainField orchestrator (M5):
// L1 estimation + L2 structure/ESDF + L3 lens queries working together.
// Build: g++ -O2 -Wall -std=c++14 -I ../../src -I /usr/include/eigen3 <this> <modules> -o test_tf
//   test_terrain_field.cpp ../../src/terrain_field/*.cpp -o test_tf && ./test_tf

#include "terrain_field/AttitudeIntervals.hpp"
#include "terrain_field/HeadingIntervals.hpp"
#include "terrain_field/TerrainField.hpp"

#include <cmath>
#include <cstdio>

using namespace traversability_generator3d::terrain_field;

namespace
{
int failures = 0;

#define CHECK(cond, msg)                                                     \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);      \
            ++failures;                                                      \
        }                                                                    \
    } while (0)
}

int main()
{
    const std::size_t W = 40, H = 40;
    const double RES = 0.5;

    TerrainParams params;
    params.gridResolution = RES;
    params.maxStepHeight = 0.25;

    // flat ground at z = 0 everywhere; a wall column at x = 20, y in [5, 34]
    TerrainGridInput in;
    in.width = W;
    in.height = H;
    in.cells.assign(W * H, {});
    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const bool wall = (x == 20 && y >= 5 && y < 35);
            if (wall)
            {
                PatchSample s;
                s.zMin = 0.f;
                s.zMax = 3.f; // tall vertical structure
                s.mean = 1.5f;
                s.normal = Eigen::Vector3f(1.f, 0.f, 0.f); // horizontal normal
                s.variance = 0.01f;
                in.cells[y * W + x].push_back(s);
            }
            else
            {
                PatchSample s;
                s.zMin = -0.01f;
                s.zMax = 0.01f;
                s.mean = 0.f;
                s.normal = Eigen::Vector3f(0.f, 0.f, 1.f);
                s.variance = 0.0001f;
                in.cells[y * W + x].push_back(s);
            }
        }
    }

    TerrainField field;
    field.compute(in, params, /*bandStart=*/params.maxStepHeight, /*bandHeight=*/0.5);

    // one layer on interior ground; grab its id from a far cell
    const auto& groundLayers = field.terrain().cells[20 * W + 5];
    CHECK(groundLayers.size() == 1, "ground cell has exactly one layer");
    const uint16_t layer = groundLayers[0].layerId;
    const TerrainField::LayerField* lf = field.layerField(layer);
    CHECK(lf != nullptr, "layer field exists");
    if (!lf)
        return 1;

    // ground height correct despite the wall (the wall-pull regression, end to end)
    for (std::size_t x = 15; x < 26; ++x)
    {
        const auto& cl = field.terrain().cells[20 * W + x];
        for (const TerrainCell& tc : cl)
            CHECK(std::fabs(tc.height) < 0.02, "ground height not pulled by wall");
    }

    // wall cells blocked; neighbors not structure-blocked
    CHECK(lf->blocked[20 * W + 20] == 1, "wall cell blocked");
    CHECK(lf->blocked[20 * W + 19] == 0, "cell next to wall not structure-blocked");
    CHECK(lf->blocked[20 * W + 5] == 0, "far cell not blocked");

    // exact ESDF: cell (10,20) center to wall column centers = (20-10)*0.5 = 5.0 m
    CHECK(std::fabs(lf->esdf[20 * W + 10] - 5.0) < 1e-3, "esdf exact at (10,20)");
    // adjacent to the wall: one cell pitch
    CHECK(std::fabs(lf->esdf[20 * W + 19] - 0.5) < 1e-3, "esdf exact at (19,20)");

    // obstacle collection
    std::vector<Eigen::Vector2d> obs;
    const Eigen::Vector2d p10((10 + 0.5) * RES, (20 + 0.5) * RES);
    field.collectObstaclesNear(layer, p10, 3.0, obs);
    CHECK(obs.empty(), "no obstacles within 3 m of (10,20)");
    field.collectObstaclesNear(layer, p10, 6.0, obs);
    CHECK(!obs.empty(), "obstacles within 6 m of (10,20)");
    for (const auto& o : obs)
        CHECK((o - p10).norm() <= 6.0 + 1e-9, "collected obstacle within radius");

    // --- lens sanity: 6x3 robot near the wall -------------------------------------
    RobotModel robot;
    robot.sizeX = 6.0;
    robot.sizeY = 3.0;
    const double r = robot.radius();          // 1.5
    const double halfDiag = robot.halfDiagonal();

    // cell (15,20): center x = 7.75, wall plane of centers at x = 10.25 -> 2.5 m
    const Eigen::Vector2d p15((15 + 0.5) * RES, (20 + 0.5) * RES);
    CHECK(std::fabs(lf->esdf[20 * W + 15] - 2.5) < 1e-3, "esdf exact at (15,20)");
    CHECK(lf->esdf[20 * W + 15] >= r && lf->esdf[20 * W + 15] < halfDiag,
          "(15,20) lies in the maybe band");

    field.collectObstaclesNear(layer, p15, halfDiag + r, obs);
    const AngleIntervalSet allowed =
        feasibleHeadings(p15, robot.diskOffsets(), r, obs);
    CHECK(!allowed.isEmpty(), "near-wall cell has some allowed heading");
    CHECK(!allowed.isFull(), "near-wall cell is restricted");
    CHECK(allowed.contains(M_PI / 2), "parallel-to-wall heading allowed");
    CHECK(!allowed.contains(0.0), "toward-wall heading blocked");

    // attitude: flat ground, limiting disabled -> full circle
    CHECK(attitudeHeadings(Eigen::Vector3f(0, 0, 1), robot).isFull(),
          "flat ground attitude is unrestricted");

    std::printf("failures: %d\nRESULT: %s\n", failures, failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
