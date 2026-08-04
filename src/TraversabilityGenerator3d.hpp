#pragma once

#include <maps/grid/MLSMap.hpp>
#include <memory>
#include <cstdint>
#include <unordered_map>
#include "TraversabilityConfig.hpp"
#include "TravGenNode.hpp"
#include "SoilNode.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <cmath> // for trigonometric functions

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                                Polyhedron_3;
typedef K::Point_3                                           Point_3;

namespace traversability_generator3d
{

class TraversabilityGenerator3d
{
public:
    // TODO use MLSMapPrecalculated and actually use slope information?
//    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > MLGrid;
    typedef maps::grid::MLSMapSloped MLGrid;
    
protected:
    
    typedef MLGrid::CellType Cell;
    typedef MLGrid::View View;
    typedef View::CellType ViewCell;
    typedef MLGrid::PatchType Patch;

    Polyhedron_3 generatePolyhedron(const std::vector<Eigen::Vector3d>& points);
    Polyhedron_3 createPolyhedronFromSurfacePatch(const maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE> *p, const Eigen::Vector3d& position);
    std::shared_ptr<MLGrid > mlsGrid;
    bool addInitialPatch;
    Eigen::Affine3d initialPatch2Mls;
    double patchRadius;

    std::vector<TravGenNode*> obstacleNodesGrowList;
    
    maps::grid::TraversabilityMap3d<TravGenNode*> trMap;
    maps::grid::TraversabilityMap3d<SoilNode*> soilMap;

    int currentNodeId = 0; //used while expanding
    int currentSoilNodeId = 0; //used while expanding

    std::vector<TravGenNode *> frontierNodesGrowList;
    
    bool computePlaneRansac(TravGenNode &node);
    double computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const;
    Eigen::Vector3d computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const;
    
    bool checkStepHeightAABB(TravGenNode* node);
    bool checkStepHeightOBB(TravGenNode* node);

    /** The robot body volume shared by the step-height OBB check and the per-yaw
     *  collision checks: the yaw-rotated, offset footprint box resting on the
     *  node's fitted plane with maxStepHeight clearance, extruded by robotHeight
     *  along the (sanitized) plane normal. */
    struct BodyHull
    {
        Polyhedron_3 poly;
        Eigen::Vector3d planeNormal;
        std::vector<Eigen::Vector3d> bottomCorners;
        Eigen::Vector3d heightOffset;
    };
    BodyHull buildBodyPolyhedron(const Eigen::Vector3d& nodePos,
                                 const Eigen::Hyperplane<double, 3>& plane, double yaw);

    /** The rotation-safe MLS search window around a node (covers the footprint at
     *  every yaw), shared by collectYawCheckPatches() and checkCollisionForYaw(). */
    Eigen::AlignedBox3d rotationSafeSearchBox(const Eigen::Vector3d& nodePos) const;

#ifdef ENABLE_DEBUG_DRAWINGS
    /** Wireframe drawing of a body/patch collision, shared by the OBB and yaw
     *  checks. Must never run from an OpenMP worker (blocking invoke to the Qt
     *  GUI thread). */
    void drawCollisionDebug(const std::string& tag, int counter, const BodyHull& body,
                            const maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE>* p,
                            const Eigen::Vector3d& pos);
#endif
    
    /** One MLS patch prepared for the per-yaw collision checks of one node: the
     *  CGAL polyhedron (yaw-INdependent, so built once per node instead of once
     *  per yaw sample), its cell centre and top height for the cheap per-yaw
     *  prefilters, and the raw patch for the debug drawings. */
    struct YawCheckPatch
    {
        Polyhedron_3 poly;
        Eigen::Vector2d cellXY;
        double zMax;
        const maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE>* patch;
        Eigen::Vector3d pos;
    };

    /** Build the patch polyhedra of the node's rotation-safe search window once;
     *  checkCollisionForYaw() reuses them for every sampled yaw. */
    std::vector<YawCheckPatch> collectYawCheckPatches(TravGenNode* node);

    /** Check if the robot at a specific yaw orientation collides with MLS patches
     *  (@p patches from collectYawCheckPatches() of the same node).
     *  @return true if the yaw is collision-free (safe). */
    bool checkCollisionForYaw(TravGenNode* node, double yaw,
                              const std::vector<YawCheckPatch>& patches);

    /** Sample exactly config.numYawSamples yaws over the full circle [0,360deg)
     *  (footprint offset applied to every check), and if any are collision-free,
     *  fill node->allowedOrientations with a wedge (width = sampling step) per
     *  safe yaw. @return true if at least one safe orientation was found. */
    bool computeSafeOrientations(TravGenNode* node);
    
    /** @return false if no allowed orientation was found (e.g. due to extreme slope)*/
    bool computeAllowedOrientations(TravGenNode* node);
    
    bool checkForFrontier(const TravGenNode* node);

    /** Outcome of the read-only expansion checks; see classifyNode(). */
    enum class NodeClassification : uint8_t
    {
        Unknown,              ///< node type is UNKNOWN: skip
        PreexistingObstacle,  ///< node was already OBSTACLE: seed inflation only
        Obstacle,             ///< failed slope / step-height / incline checks
        Traversable           ///< passed all checks
    };

    /** The read-only half of expandNode(): slope, step-height (AABB with OBB
     *  fallback) and incline-limit checks. Reads only the MLS/trMap geometry and
     *  writes only the node's OWN userData (allowedOrientations), so DISTINCT
     *  nodes may be classified concurrently from multiple threads. */
    NodeClassification classifyNode(TravGenNode* node);


    /** A node fitted by buildPatchNodeAt() but not yet part of the map: no id,
     *  not inserted into trMap. */
    struct PrefitPatch
    {
        TravGenNode* node = nullptr;
    };

    /** The read-only (fitting) half of createTraversabilityPatchAt(): candidate
     *  heights from the MLS plus the RANSAC plane fits. Safe to run concurrently
     *  for distinct cells. */
    PrefitPatch buildPatchNodeAt(const maps::grid::Index& idx, const double curHeight);

    /** Serial completion of buildPatchNodeAt(): assigns the node id and inserts
     *  the node into trMap. Must run single-threaded. */
    TravGenNode* finishPatchNode(const PrefitPatch& prefit, const maps::grid::Index& idx);

    /** One cell pre-fitted during the parallel phase of an expansion wave. */
    struct PrefitEntry
    {
        maps::grid::Index idx;
        double requestHeight;
        PrefitPatch patch;
    };
    typedef std::unordered_map<uint64_t, PrefitEntry> PrefitCache;

    /** The mutation half of expandNode(): applies a classification (soil node,
     *  expanded flag, type, grow lists, neighbor growth, frontier check).
     *  expandNode() = classifyNode() + finalizeNode(); the wave loop calls it
     *  with the pre-computed classification and the wave's pre-fit cache.
     *  Must run single-threaded. @return true if the node became TRAVERSABLE. */
    bool finalizeNode(TravGenNode* node, NodeClassification classification,
                      PrefitCache* prefits);

    enum class NeighborRequest : uint8_t { Ok, Skip, Abort };

    /** The neighbor projection addConnectedPatches() performs before matching or
     *  creating a neighbor node in direction @p idxS: output cell and height.
     *  Abort mirrors the historical early-return of addConnectedPatches() on an
     *  inconsistent plane intersection (remaining neighbors are not processed). */
    NeighborRequest computeNeighborRequest(const TravGenNode* node, const maps::grid::Index& idxS,
                                           maps::grid::Index& outIdx, double& outHeight) const;

    void addConnectedPatches(TravGenNode* node);

    /** addConnectedPatches() variant that consumes nodes pre-fitted by the
     *  parallel wave phase instead of fitting them inline. */
    void addConnectedPatches(TravGenNode* node, PrefitCache* prefits);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
    
    static double interpolate(double x, double x0, double y0, double x1, double y1);
    
    TravGenNode *createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight);
    SoilNode *createSoilPatchAt(maps::grid::Index idx, const double curHeight);

    void inflateFrontiers();

    void inflateObstacles();
    
    TraversabilityConfig config;
    
    void addInitialPatchToMLS();

    SoilNode* generateStartSoilNode(const Eigen::Vector3d& startPos);

public:
    TraversabilityGenerator3d(const TraversabilityConfig &config);

    virtual ~TraversabilityGenerator3d();

    void clearTrMap();
    void clearSoilMap();

    void setInitialPatch(const Eigen::Affine3d &ground2Mls, double patchRadius);

    void setSoilType(SoilNode * node, SoilType soilType);
    bool addSoilNode(const SoilSample& sample);

    double gaussian2D(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY);

    virtual TravGenNode *generateStartNode(const Eigen::Vector3d &startPos);
    TravGenNode *findMatchingTraversabilityPatchAt(maps::grid::Index idx, const double curHeight) const;
    SoilNode* findMatchingSoilPatchAt(maps::grid::Index idx, const double curHeight) const;
    void updateSoilInformation();

    /**Expand the map starting from all given @p positions */
    void expandAll(const std::vector<Eigen::Vector3d>& positions);
    
    void expandAll(const Eigen::Vector3d &startPos);
    
    
    /**Expands the map starting at @p startPos.
     * Expansion will stop if a distance of @p expandDist is reached. I.e. this will expand all nodes
     * in a circle of radius @p expandDist around @p startPos.*/
    void expandAll(const Eigen::Vector3d &startPos, const double expandDist);
    
    void expandAll(TravGenNode *startNode);
    
    /** @param expandDist How far should the map be expanded? If negative the whole map will be expanded. */
    void expandAll(TravGenNode *startNode, const double expandDist);

    virtual bool expandNode(TravGenNode *node);
    
    void setMLSGrid(std::shared_ptr<MLGrid> &grid);
    
    /**Returns the number of nodes after expansion*/
    int getNumNodes() const;
    
    const maps::grid::TraversabilityMap3d<TravGenNode *> &getTraversabilityMap() const;
    const maps::grid::TraversabilityMap3d<SoilNode *> &getSoilMap() const;
    void addConnectedPatches(SoilNode *  node);
        
    void setConfig(const TraversabilityConfig &config);


};

}
