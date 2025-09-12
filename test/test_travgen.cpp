#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <maps/grid/MLSMap.hpp>
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>

using namespace traversability_generator3d;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " cloud.ply" << std::endl;
        return -1;
    }

    // Load PLY file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPLYFile(argv[1], cloud) < 0) {
        std::cerr << "Error: could not load " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud.size() << " points." << std::endl;

    // Build MLS
    maps::grid::MLSConfig cfg;
    cfg.gapSize = 0.1;
    maps::grid::MLSMapSloped mls(
        maps::grid::Vector2ui(200, 200),
        maps::grid::Vector2d(0.3, 0.3),
        cfg
    );

    base::Transform3d identity; identity.setIdentity();
    mls.mergePointCloud(cloud, identity);

    // Configure Traversability
    TraversabilityConfig config;
    config.gridResolution = 0.3;
    config.maxSlope = 0.5;
    config.maxStepHeight = 0.2;
    config.robotSizeX = 0.8;
    config.robotSizeY = 0.8;
    config.robotHeight = 1.5;

    TraversabilityGenerator3d travgen(config);

    auto mls_ptr = std::make_shared<maps::grid::MLSMapSloped>(mls);
    travgen.setMLSGrid(mls_ptr);
    travgen.expandAll(Eigen::Vector3d(5.0, 5.0, 0.0));

    auto& travmap = travgen.getTraversabilityMap();
    int num_nodes = travgen.getNumNodes();

    std::cout << "Generated TraversabilityMap3d with " << num_nodes << " nodes." << std::endl;

    int num_trav = 0, num_obs = 0, num_front = 0;
    for (auto& level : travmap) {
        for (auto* node : level) {
            switch (node->getUserData().nodeType) {
                case TRAVERSABLE: num_trav++; break;
                case OBSTACLE: num_obs++; break;
                case FRONTIER: num_front++; break;
                default: break;
            }
        }
    }
    std::cout << "Nodes: TRAV=" << num_trav 
              << ", OBS=" << num_obs 
              << ", FRONTIER=" << num_front << std::endl;
    return 0;
}
