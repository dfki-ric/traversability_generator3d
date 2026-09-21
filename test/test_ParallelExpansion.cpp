#define BOOST_TEST_MODULE ParallelExpansionTestModule
#include <boost/test/included/unit_test.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "traversability_generator3d/TraversabilityGenerator3d.hpp"

using namespace maps::grid;
using traversability_generator3d::NodeType;
using traversability_generator3d::TravGenNode;
using traversability_generator3d::TraversabilityConfig;
using traversability_generator3d::TraversabilityGenerator3d;

namespace
{

/** Terrain with everything the expansion has to handle: flat ground, a tall
 *  obstacle block (obstacle typing + inflation + yaw sampling), a small step
 *  (step-height checks) and an unmeasured hole (unmeasured cells stay
 *  OBSTACLE — no pocket refilling by design). */
MLSMapSloped buildTestMls()
{
    const Vector2d res(0.3, 0.3);
    const Vector2ui numCells(20, 20);

    MLSConfig cfg;
    cfg.gapSize = 0.1;
    cfg.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls(numCells, res, cfg);
    mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

    const Eigen::Vector2d max = 0.5 * mls.getSize();
    const Eigen::Vector2d min = -0.5 * mls.getSize();

    for (double x = min.x(); x < max.x(); x += 0.05)
    {
        for (double y = min.y(); y < max.y(); y += 0.05)
        {
            // unmeasured hole
            if (x >= -1.5 && x < -0.9 && y >= -1.5 && y < -0.9)
                continue;

            double z = 0.0;
            // tall obstacle block
            if (x >= 0.9 && x < 1.8 && y >= 0.9 && y < 1.8)
                z = 0.6;
            // small step, below default maxStepHeight
            if (x >= -1.8 && x < -1.2 && y >= 0.9 && y < 1.5)
                z = 0.15;
            mls.mergePoint(Eigen::Vector3d(x, y, z));
        }
    }
    return mls;
}

TraversabilityConfig testConfig()
{
    TraversabilityConfig cfg;
    cfg.gridResolution = 0.3;
    cfg.robotSizeX = 0.6;
    cfg.robotSizeY = 0.6;
    return cfg;
}

/** Canonical, order-independent description of a generated map: one tuple per
 *  node (cell x, cell y, quantized height, base type, user node type), sorted. */
typedef std::tuple<int, int, std::int64_t, int, int> NodeSignature;

std::vector<NodeSignature> mapSignature(const TraversabilityGenerator3d& gen)
{
    std::vector<NodeSignature> sig;
    for (const LevelList<TravGenNode*>& level : gen.getTraversabilityMap())
    {
        for (const TravGenNode* n : level)
        {
            sig.emplace_back(n->getIndex().x(), n->getIndex().y(),
                             static_cast<std::int64_t>(std::llround(n->getHeight() * 1e4)),
                             static_cast<int>(n->getType()),
                             static_cast<int>(n->getUserData().nodeType));
        }
    }
    std::sort(sig.begin(), sig.end());
    return sig;
}

/** Expand the shared test terrain with the given thread count (via the
 *  TraversabilityConfig::numThreads parameter; expandAll() applies it and
 *  overrides any ambient OpenMP setting, so the config is the only knob). */
std::unique_ptr<TraversabilityGenerator3d> expandWithThreads(
    const std::shared_ptr<MLSMapSloped>& mlsPtr, int numThreads)
{
    TraversabilityConfig cfg = testConfig();
    cfg.numThreads = numThreads;
    std::unique_ptr<TraversabilityGenerator3d> gen(
        new TraversabilityGenerator3d(cfg));
    gen->setMLSGrid(const_cast<std::shared_ptr<MLSMapSloped>&>(mlsPtr));
    gen->expandAll(Eigen::Vector3d(0.0, 0.0, 0.0));
    return gen;
}

int countNodeType(const TraversabilityGenerator3d& gen, NodeType t)
{
    int count = 0;
    for (const LevelList<TravGenNode*>& level : gen.getTraversabilityMap())
        for (const TravGenNode* n : level)
            if (n->getUserData().nodeType == t)
                count++;
    return count;
}

}

/** The core guarantee of the wave-parallel expansion: the produced map does not
 *  depend on the number of threads. */
BOOST_AUTO_TEST_CASE(parallel_map_equals_single_threaded_map)
{
    auto mlsPtr = std::make_shared<MLSMapSloped>(buildTestMls());

    auto genSerial = expandWithThreads(mlsPtr, 1);
    const std::vector<NodeSignature> serialSig = mapSignature(*genSerial);
    const int serialNodes = genSerial->getNumNodes();

    // numThreads = 0 means "do not parallelize" — it must behave exactly like
    // an explicit single-thread run.
    auto genOff = expandWithThreads(mlsPtr, 0);
    BOOST_CHECK(mapSignature(*genOff) == serialSig);
    genOff.reset();

#ifdef _OPENMP
    const int hwThreads = std::max(2, omp_get_num_procs());
#else
    const int hwThreads = 2;
#endif
    auto genParallel = expandWithThreads(mlsPtr, hwThreads);
    const std::vector<NodeSignature> parallelSig = mapSignature(*genParallel);

    BOOST_CHECK_EQUAL(serialNodes, genParallel->getNumNodes());
    BOOST_REQUIRE_EQUAL(serialSig.size(), parallelSig.size());
    size_t mismatches = 0;
    for (size_t i = 0; i < serialSig.size(); i++)
    {
        if (serialSig[i] != parallelSig[i])
            mismatches++;
    }
    BOOST_CHECK_MESSAGE(mismatches == 0,
        std::to_string(mismatches) + " of " + std::to_string(serialSig.size()) +
        " nodes differ between 1-thread and " + std::to_string(hwThreads) +
        "-thread expansion");
}

/** Repeated multi-threaded runs must be identical to each other (no scheduling-
 *  dependent results, no data races surfacing as flaky types). */
BOOST_AUTO_TEST_CASE(parallel_expansion_is_repeatable)
{
    auto mlsPtr = std::make_shared<MLSMapSloped>(buildTestMls());

#ifdef _OPENMP
    const int hwThreads = std::max(2, omp_get_num_procs());
#else
    const int hwThreads = 2;
#endif

    auto genA = expandWithThreads(mlsPtr, hwThreads);
    const std::vector<NodeSignature> sigA = mapSignature(*genA);
    genA.reset();

    for (int run = 0; run < 3; run++)
    {
        auto genB = expandWithThreads(mlsPtr, hwThreads);
        const std::vector<NodeSignature> sigB = mapSignature(*genB);
        BOOST_REQUIRE_EQUAL(sigA.size(), sigB.size());
        BOOST_CHECK(sigA == sigB);
    }
}

/** The parallel expansion must preserve the map semantics: full coverage of the
 *  measured grid, correct typing of the synthetic features, and the global
 *  no-UNKNOWN/FRONTIER invariants of the unmeasured->OBSTACLE design. */
BOOST_AUTO_TEST_CASE(parallel_expansion_semantics)
{
    auto mlsPtr = std::make_shared<MLSMapSloped>(buildTestMls());
#ifdef _OPENMP
    const int hwThreads = std::max(2, omp_get_num_procs());
#else
    const int hwThreads = 2;
#endif
    auto gen = expandWithThreads(mlsPtr, hwThreads);

    // Near-full coverage of the 20x20 grid (the point sampling leaves border
    // cells without support, so the exact count is scene-dependent; bitwise
    // equality across thread counts is asserted by the tests above).
    BOOST_CHECK_GE(gen->getNumNodes(), 20 * 20 - 5);
    BOOST_CHECK_LE(gen->getNumNodes(), 20 * 20);

    // Flat ground far from all features is traversable.
    Index idx;
    gen->getTraversabilityMap().toGrid(Eigen::Vector3d(0.0, -1.2, 0.0), idx);
    const TravGenNode* flat = gen->findMatchingTraversabilityPatchAt(idx, 0.0);
    BOOST_REQUIRE(flat);
    BOOST_CHECK_EQUAL(static_cast<int>(flat->getUserData().nodeType),
                      static_cast<int>(NodeType::TRAVERSABLE));

    // The edge cell of the tall block is an obstacle. (The block INTERIOR gets
    // no nodes at all: expansion stops at the obstacle ring, so only cells
    // requested by a traversable neighbor exist. The node may also sit at the
    // interpolated ground height rather than on top of the block, so check
    // every node of the cell instead of matching a height band.)
    gen->getTraversabilityMap().toGrid(Eigen::Vector3d(1.05, 1.05, 0.0), idx);
    size_t blockNodes = 0;
    for (const TravGenNode* n : gen->getTraversabilityMap().at(idx))
    {
        blockNodes++;
        BOOST_CHECK_EQUAL(static_cast<int>(n->getType()),
                          static_cast<int>(TraversabilityNodeBase::OBSTACLE));
    }
    BOOST_CHECK_GT(blockNodes, 0);

    // The scene has obstacles, so the inflation/yaw pass must have produced at
    // least one evaluated ring node (either partially traversable or obstacle).
    BOOST_CHECK_GT(countNodeType(*gen, NodeType::OBSTACLE) +
                   countNodeType(*gen, NodeType::PARTIALLY_TRAVERSABLE), 0);

    // Global invariants: no UNSET/UNKNOWN/FRONTIER-family nodes survive, and no
    // node was left unexpanded by the wave bookkeeping.
    for (const LevelList<TravGenNode*>& level : gen->getTraversabilityMap())
    {
        for (const TravGenNode* n : level)
        {
            BOOST_CHECK(n->getUserData().nodeType != NodeType::UNKNOWN);
            BOOST_CHECK(n->getUserData().nodeType != NodeType::UNSET);
            BOOST_CHECK(n->getUserData().nodeType != NodeType::FRONTIER);
            BOOST_CHECK(n->getType() != TraversabilityNodeBase::UNSET);
            BOOST_CHECK(n->isExpanded());
        }
    }
}
