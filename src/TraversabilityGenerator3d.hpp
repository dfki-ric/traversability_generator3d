#pragma once

#include <maps/grid/MLSMap.hpp>
#include <memory>
#include "TraversabilityConfig.hpp"
#include "TravGenNode.hpp"
#include "SoilNode.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Aff_transformation_3.h>
#include <cmath> // for trigonometric functions

#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz RT;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float RT;
#endif

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                                Polyhedron_3;
typedef K::Point_3                                           Point_3;
typedef CGAL::Surface_mesh<Point_3>                          Surface_mesh;
typedef Polyhedron_3::Vertex_const_iterator Vertex_const_iterator;
typedef CGAL::Homogeneous<RT>::Segment_3                     Segment_3;
typedef CGAL::Aff_transformation_3<K> Transformation;
typedef K::Vector_3 Vector_3;

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

    std::vector<Eigen::Vector3d> robotEdges;
    Polyhedron_3 robotPolyhedron;

    std::vector<Eigen::Vector3d> patchEdges;
    Polyhedron_3 patchPolyhedron;
    double patchHeight;

    Polyhedron_3 generatePolyhedron(const std::vector<Eigen::Vector3d>& points);
    void transformPolyhedron(Polyhedron_3& polyhedron, const Transformation& transform);
    Transformation generateTransform(const Eigen::Vector3d& normal, const Eigen::Vector3d& translation);
    Polyhedron_3 createPolyhedronFromSurfacePatch(const maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE> *p, const Eigen::Vector3d& position);
    void drawWireFrameBox(const Eigen::Vector3d& normal, const Eigen::Vector3d& position, const Eigen::Vector3d& size, const Eigen::Vector4d& colorRGBA);
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
    
    /** @return false if no allowed orientation was found (e.g. due to extreme slope)*/
    bool computeAllowedOrientations(TravGenNode* node);
    
    static bool checkForFrontier(const TravGenNode* node);
    
    void addConnectedPatches(TravGenNode* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
    
    static double interpolate(double x, double x0, double y0, double x1, double y1);
    
    bool isNodeFreeOfObstacles(const traversability_generator3d::TravGenNode* node) const;
    TravGenNode *createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight);
    SoilNode *createSoilPatchAt(maps::grid::Index idx, const double curHeight);

    void inflateFrontiers();

    void inflateObstacles();
    
    TraversabilityConfig config;
    
    void addInitialPatchToMLS();
    
    int intersections();

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
