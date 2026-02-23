/** 
 * @file travgen.cpp
 * @brief Example test program to generate a traversability map from a PLY point cloud.
 *
 * Requirements:
 *  - A point cloud (.ply) of the terrain
 *  - This will be inserted into an MLS (Multi-Level Surface) map
 *  - TraversabilityGenerator3d computes traversability nodes from the MLS
 *
 * Notes:
 *  - The traversability start is assumed to be ON the MLS surface, 
 *    so the z-value given is interpreted as distance to the ground (distToGround).
 */

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <maps/grid/MLSMap.hpp>
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>

using namespace traversability_generator3d;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " cloud.ply" << std::endl;
        return -1;
    }

    /** Step 1: Load input point cloud
     * The input PLY must contain a terrain point cloud (x,y,z).
     * This will be used to build the MLS map as a grid representation.
     */
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPLYFile(argv[1], cloud) < 0) {
        std::cerr << "Error: could not load " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud.size() << " points." << std::endl;

    /** Step 2: Configure traversability parameters
     * These parameters describe robot size and terrain constraints.
     * - gridResolution: resolution of MLS/traversability grid
     * - maxSlope: maximum slope (rad) the robot can handle
     * - maxStepHeight: vertical step height tolerance
     * - robotSizeX/Y/Height: robot footprint and height
     * - distToGround: starting offset from MLS surface (z-direction)
     * - See further parameters in TraversabilityConfig.hpp.
     */
    TraversabilityConfig config;
    config.gridResolution = 0.3;
    config.maxSlope = 0.5;
    config.maxStepHeight = 0.2;
    config.robotSizeX = 0.8;
    config.robotSizeY = 0.8;
    config.robotHeight = 1.5;
    config.distToGround = 0.47;

    TraversabilityGenerator3d travGen(config);

    /** Step 3: Build MLS map
     * - Use bounding box of the cloud to size the grid
     * - MLS stores multiple height levels per cell (for sloped or layered terrain)
     */
    pcl::PointXYZ mi, ma; 
    pcl::getMinMax3D (cloud, mi, ma); 

    const double mls_res = config.gridResolution;
    const double size_x = ma.x - mi.x;
    const double size_y = ma.y - mi.y;

    const maps::grid::Vector2ui numCells(size_x / mls_res + 2, size_y / mls_res + 2);

    maps::grid::MLSConfig cfg;
    cfg.gapSize = 0.1; // small gaps between layers
    const maps::grid::Vector2d mapSize(numCells[0]*mls_res, numCells[1]*mls_res);
    const maps::grid::Vector3d offset(mi.x-0.5*mls_res, mi.y-0.5*mls_res, 0);

    maps::grid::MLSMapSloped mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
    mlsMap.translate(offset);
    mlsMap.mergePointCloud(cloud, base::Transform3d::Identity());

    /** Step 4: Run traversability generation
     * - Provide MLS to the TraversabilityGenerator3d
     * - expandAll() runs the full analysis from MLS cells
     * - The robot start is assumed to be on the MLS (z = -distToGround)
     */
    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
    travGen.setMLSGrid(mlsPtr);            
    travGen.expandAll(Eigen::Vector3d(0.0, 0.0, -config.distToGround));

    auto& travmap = travGen.getTraversabilityMap();
    int numNodes = travGen.getNumNodes();

    std::cout << "Generated TraversabilityMap3d with " << numNodes << " nodes." << std::endl;

    /** Step 5: Count node types
     * Traversability map nodes are categorized as:
     * - TRAVERSABLE: robot can move here
     * - OBSTACLE: blocked
     * - FRONTIER: boundary between known and unknown space
     */
    int numTrav = 0, numObs = 0, numFrontier = 0;
    for (auto& level : travmap) {
        for (auto* node : level) {
            switch (node->getUserData().nodeType) {
                case TRAVERSABLE: numTrav++; break;
                case OBSTACLE: numObs++; break;
                case FRONTIER: numFrontier++; break;
                default: break;
            }
        }
    }
    std::cout << "Nodes: TRAVERSABLE=" << numTrav 
              << ", OBSTACLE=" << numObs 
              << ", FRONTIER=" << numFrontier << std::endl;
    return 0;
}
