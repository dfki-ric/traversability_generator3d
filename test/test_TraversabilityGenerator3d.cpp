#define BOOST_TEST_MODULE TraversabilityGenerator3dTestModule
#include <boost/test/included/unit_test.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "traversability_generator3d/TraversabilityGenerator3d.hpp"

using namespace maps::grid;

std::vector<Eigen::Vector3d> startPositions;
std::vector<Eigen::Vector3d> goalPositions;

struct TraversabilityGenerator3dTest {
    TraversabilityGenerator3dTest(){}
    ~TraversabilityGenerator3dTest(){}

    traversability_generator3d::TraversabilityConfig traversabilityConfig;
    traversability_generator3d::TraversabilityGenerator3d* travGen;
};

BOOST_FIXTURE_TEST_CASE(travmap_resolution_equal_to_mls_resolution, TraversabilityGenerator3dTest){
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls.getLocalFrame().translation() << 0.5*mls.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls.getSize();
    Eigen::Vector2d min = -0.5 * mls.getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        //double cs = std::cos(x * M_PI/2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            //double sn = std::sin(y * M_PI/2.5);
            mls.mergePoint(Eigen::Vector3d(x, y, 0));
        }
    }

    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    traversabilityConfig.gridResolution = 0.3;
    travGen->setMLSGrid(mlsPtr);            
    travGen->expandAll(Eigen::Vector3d(0.0, 0.0, 0.0));
    BOOST_CHECK_EQUAL(travGen->getNumNodes(), numCells.x()* numCells.y());
    delete travGen;
}

BOOST_FIXTURE_TEST_CASE(check_travmap, TraversabilityGenerator3dTest){
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls.getLocalFrame().translation() << 0.5*mls.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls.getSize();
    Eigen::Vector2d min = -0.5 * mls.getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double z = 0;
            if ((x >= 0.6 && x < 0.9) && (y >= 0.6 && y < 0.9)){
                z = 0.3;
            }
            mls.mergePoint(Eigen::Vector3d(x, y, z));
        }
    }

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    traversabilityConfig.gridResolution = 0.3;

    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    travGen->setMLSGrid(mlsPtr);            

    std::vector<Eigen::Vector3d> startPositions;
    startPositions.emplace_back(Eigen::Vector3d(0,0,0));
    travGen->expandAll(startPositions);

    Eigen::Vector3d positionUnknown{1.2, 0.0, 0};
    maps::grid::Index idxUnknown;
    travGen->getTraversabilityMap().toGrid(positionUnknown, idxUnknown);
    auto unknown = travGen->findMatchingTraversabilityPatchAt(idxUnknown,0);
    BOOST_CHECK_EQUAL(unknown->getUserData().nodeType, ::traversability_generator3d::NodeType::UNKNOWN);

    Eigen::Vector3d positionFrontier{0.9, 0.0, 0};
    maps::grid::Index idxFrontier;
    travGen->getTraversabilityMap().toGrid(positionFrontier, idxFrontier);
    auto frontier = travGen->findMatchingTraversabilityPatchAt(idxFrontier,0);
    BOOST_CHECK_EQUAL(frontier->getUserData().nodeType, ::traversability_generator3d::NodeType::FRONTIER);

    Eigen::Vector3d positionInflatedFrontier{0.6, 0.0, 0};
    maps::grid::Index idxInflatedFrontier;
    travGen->getTraversabilityMap().toGrid(positionInflatedFrontier, idxInflatedFrontier);
    auto inflatedFrontier = travGen->findMatchingTraversabilityPatchAt(idxInflatedFrontier,0);
    BOOST_CHECK_EQUAL(inflatedFrontier->getUserData().nodeType, ::traversability_generator3d::NodeType::INFLATED_FRONTIER);

    Eigen::Vector3d positionObs{0.65, 0.65, 0};
    maps::grid::Index idxObstacleNode;
    travGen->getTraversabilityMap().toGrid(positionObs, idxObstacleNode);
    auto &trList(travGen->getTraversabilityMap().at(idxObstacleNode));
    auto obstacle = travGen->findMatchingTraversabilityPatchAt(idxObstacleNode,0);
    BOOST_CHECK_EQUAL(obstacle->getUserData().nodeType, ::maps::grid::TraversabilityNodeBase::OBSTACLE);

    Eigen::Vector3d positionInfObst{0.9, 0.3, 0};
    maps::grid::Index idxInfObstNode;
    travGen->getTraversabilityMap().toGrid(positionInfObst, idxInfObstNode);
    auto inflatedObstacle = travGen->findMatchingTraversabilityPatchAt(idxInfObstNode,0);
    BOOST_CHECK_EQUAL(inflatedObstacle->getUserData().nodeType, ::traversability_generator3d::NodeType::INFLATED_OBSTACLE);

    Eigen::Vector3d positionTrav{0.3, 0.3, 0};
    maps::grid::Index idxTraversableNode;
    travGen->getTraversabilityMap().toGrid(positionTrav, idxTraversableNode);
    auto traversable = travGen->findMatchingTraversabilityPatchAt(idxTraversableNode,0);
    BOOST_CHECK_EQUAL(traversable->getType(), ::maps::grid::TraversabilityNodeBase::TRAVERSABLE);

    delete travGen;
}

BOOST_FIXTURE_TEST_CASE(check_stepheight, TraversabilityGenerator3dTest){
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls.getLocalFrame().translation() << 0.5*mls.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls.getSize();
    Eigen::Vector2d min = -0.5 * mls.getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double z = 0;
            if ((x >= 0 && x < 0.3) && (y >= 0 && y < 0.3)){
                z = 0.1;
            }
            mls.mergePoint(Eigen::Vector3d(x, y, z));
        }
    }

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    traversabilityConfig.gridResolution = 0.3;
    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    travGen->setMLSGrid(mlsPtr);       
    
    std::vector<Eigen::Vector3d> startPositions;
    startPositions.emplace_back(Eigen::Vector3d(0,0,0));
    travGen->expandAll(startPositions);

    Eigen::Vector3d positionObs{0.25, 0.25, 0};
    maps::grid::Index idxObstacleNode;

    travGen->getTraversabilityMap().toGrid(positionObs, idxObstacleNode);
    for(auto *snode : travGen->getTraversabilityMap().at(idxObstacleNode))
    {
        BOOST_CHECK_EQUAL(snode->getType(), ::maps::grid::TraversabilityNodeBase::OBSTACLE);
    }
    delete travGen;

    traversabilityConfig.maxStepHeight = 0.2;
    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    travGen->setMLSGrid(mlsPtr);       
    travGen->expandAll(startPositions);

    travGen->getTraversabilityMap().toGrid(positionObs, idxObstacleNode);
    for(auto *snode : travGen->getTraversabilityMap().at(idxObstacleNode))
    {
        BOOST_CHECK_EQUAL(snode->getType(), ::maps::grid::TraversabilityNodeBase::TRAVERSABLE);
    }
    delete travGen;
}

BOOST_FIXTURE_TEST_CASE(check_plane_fitting_and_slope_flat, TraversabilityGenerator3dTest){
    Vector2d res(0.2, 0.2);
    Vector2ui numCells(8, 8);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls(numCells, res, mls_config);

    mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

    // perfectly flat ground
    for (double x = -0.8; x <= 0.8; x += 0.05)
        for (double y = -0.8; y <= 0.8; y += 0.05)
            mls.mergePoint({x, y, 0.0});

    traversabilityConfig.gridResolution = 0.2;
    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    travGen->setMLSGrid(mlsPtr);
    travGen->expandAll(Eigen::Vector3d(0, 0, 0));

    Index idx;
    travGen->getTraversabilityMap().toGrid({0,0,0}, idx);
    auto node = travGen->findMatchingTraversabilityPatchAt(idx, 0);

    BOOST_REQUIRE(node);
    BOOST_CHECK_SMALL(node->getUserData().slope, 1e-3);
    BOOST_CHECK_SMALL(node->getUserData().slopeDirection.norm(), 1e-3);

    delete travGen;
}

BOOST_FIXTURE_TEST_CASE(check_incline_limitting_via_expand, TraversabilityGenerator3dTest){
    traversabilityConfig.enableInclineLimitting = true;
    traversabilityConfig.maxSlope = M_PI / 6.0;
    traversabilityConfig.inclineLimittingMinSlope = M_PI / 18.0;

    Vector2d res(0.3,0.3);
    Vector2ui numCells(6,6);

    MLSConfig cfg;
    cfg.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls(numCells,res,cfg);
    mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

    // create an inclined plane
    for(double x=-0.8;x<=0.8;x+=0.05)
        for(double y=-0.8;y<=0.8;y+=0.05)
            mls.mergePoint({x,y,0.1*x});

    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);

    travGen->setMLSGrid(mlsPtr);
    travGen->expandAll({0,0,0});

    Index idx;
    travGen->getTraversabilityMap().toGrid({0,0,0}, idx);
    auto node = travGen->findMatchingTraversabilityPatchAt(idx, 0);

    BOOST_REQUIRE(node);
    BOOST_CHECK(!node->getUserData().allowedOrientations.empty());

    delete travGen;
}


BOOST_FIXTURE_TEST_CASE(check_frontier_inflation, TraversabilityGenerator3dTest){
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig cfg;
    cfg.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls(numCells, res, cfg);
    mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

    // Only populate lower half → force frontier
    for (double x = -1.0; x < 1.0; x += 0.05)
        for (double y = -1.0; y <= 1.0; y += 0.05)
            mls.mergePoint({x,y,0});

    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    auto mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    travGen->setMLSGrid(mlsPtr);
    travGen->expandAll({0,0,0});

    size_t inflatedFrontierCount = 0;
    for (auto& level : travGen->getTraversabilityMap())
        for (auto* n : level){
            if (n->getUserData().nodeType == traversability_generator3d::NodeType::INFLATED_FRONTIER)
                inflatedFrontierCount++;
        }

    BOOST_CHECK_GT(inflatedFrontierCount, 0);
    delete travGen;
}

BOOST_FIXTURE_TEST_CASE(check_obstacle_inflation_via_expand, TraversabilityGenerator3dTest){
    traversabilityConfig.robotSizeX = 0.6;
    traversabilityConfig.robotSizeY = 0.6;

    Vector2d res(0.3,0.3);
    Vector2ui numCells(6,6);

    MLSConfig cfg;
    cfg.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls(numCells,res,cfg);
    mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

    // create a tall obstacle
    for(double x=-0.8;x<=0.8;x+=0.05)
        for(double y=-0.8;y<=0.8;y+=0.05)
            mls.mergePoint({x,y,(x>0.2 && y>0.2) ? 0.5 : 0.0});

    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    travGen->setMLSGrid(mlsPtr);       
    travGen->expandAll({0,0,0});

    Index idx;
    travGen->getTraversabilityMap().toGrid({0.3,0.3,0}, idx);

    for(auto* n : travGen->getTraversabilityMap().at(idx))
    {
        BOOST_CHECK(n->getType() == TraversabilityNodeBase::OBSTACLE ||
                    n->getUserData().nodeType == traversability_generator3d::NodeType::INFLATED_OBSTACLE);
    }
    delete travGen;
}


BOOST_FIXTURE_TEST_CASE(check_gaussian2D_properties, TraversabilityGenerator3dTest){
    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);

    double g0 = travGen->gaussian2D(
        0, 0, 0, 0, 1, 1);

    double g1 = travGen->gaussian2D(
        1, 0, 0, 0, 1, 1);

    BOOST_CHECK_CLOSE(g0, 1.0, 1e-6);
    BOOST_CHECK_LT(g1, g0);

    delete travGen;
}

BOOST_FIXTURE_TEST_CASE(check_soil_propagation, TraversabilityGenerator3dTest){
    Vector2d res(0.3,0.3);
    Vector2ui numCells(6,6);

    MLSConfig cfg;
    cfg.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls(numCells,res,cfg);
    mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

    for(double x=-0.8;x<=0.8;x+=0.05)
        for(double y=-0.8;y<=0.8;y+=0.05)
            mls.mergePoint({x,y,0});

    travGen = new traversability_generator3d::TraversabilityGenerator3d(traversabilityConfig);
    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    travGen->setMLSGrid(mlsPtr);

    traversability_generator3d::SoilSample s;
    s.location = {0,0,0};
    s.type = traversability_generator3d::SoilType::SAND;
    s.sigmaX = s.sigmaY = 0.5;
    s.uncertainty = 0.2;

    BOOST_CHECK(travGen->addSoilNode(s));

    Index idx;
    travGen->getSoilMap().toGrid({0,0,0}, idx);
    auto node = travGen->findMatchingSoilPatchAt(idx,0);

    BOOST_REQUIRE(node);
    BOOST_CHECK_GT(node->getUserData().probSand, 0.5);

    delete travGen;
}