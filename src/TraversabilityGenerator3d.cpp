#include "TraversabilityGenerator3d.hpp"
#include <numeric/PlaneFitting.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>

#include <deque>
using namespace maps::grid;

namespace traversability_generator3d
{

TraversabilityGenerator3d::TraversabilityGenerator3d(const TraversabilityConfig& config) : addInitialPatch(false), config(config)
{
    trMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));

    // Assume center of robot at (0,0,0) for simplicity
    double robothalflength = config.robotSizeX / 2.0;
    double robothalfwidth = config.robotSizeY / 2.0;
    double robothalfheight = config.robotHeight / 2.0;

    robotEdges = {
        {robothalflength, robothalfwidth, robothalfheight},   // Top right front
        {robothalflength, robothalfwidth, -robothalfheight},  // Top right back
        {robothalflength, -robothalfwidth, robothalfheight},  // Bottom right front
        {robothalflength, -robothalfwidth, -robothalfheight}, // Bottom right back
        {-robothalflength, robothalfwidth, robothalfheight},  // Top left front
        {-robothalflength, robothalfwidth, -robothalfheight}, // Top left back
        {-robothalflength, -robothalfwidth, robothalfheight}, // Bottom left front
        {-robothalflength, -robothalfwidth, -robothalfheight} // Bottom left back
    };

    robotPolyhedron = generatePolyhedron(robotEdges);


    // Assume center of robot at (0,0,0) for simplicity
    double patchhalflength = config.gridResolution/2;
    double patchhalfwidth = config.gridResolution/2;

    //TODO: Hardcoded for now!
    patchheight = 0.02;

    double patchhalfheight = patchheight/2;

    patchEdges = {
        {patchhalflength, patchhalfwidth, patchhalfheight},   // Top right front
        {patchhalflength, patchhalfwidth, -patchhalfheight},  // Top right back
        {patchhalflength, -patchhalfwidth, patchhalfheight},  // Bottom right front
        {patchhalflength, -patchhalfwidth, -patchhalfheight}, // Bottom right back
        {-patchhalflength, patchhalfwidth, patchhalfheight},  // Top left front
        {-patchhalflength, patchhalfwidth, -patchhalfheight}, // Top left back
        {-patchhalflength, -patchhalfwidth, patchhalfheight}, // Bottom left front
        {-patchhalflength, -patchhalfwidth, -patchhalfheight} // Bottom left back
    };

    patchPolyhedron = generatePolyhedron(patchEdges);
}

Polyhedron_3 TraversabilityGenerator3d::generatePolyhedron(std::vector<Eigen::Vector3d> points){
    std::vector<Point_3> cgal_p3;   

    for (auto it = points.cbegin(); it != points.cend(); ++it){
        Point_3 p(it->x(), it->y(), it->z());
        cgal_p3.push_back(p);
    }
    
    CGAL::Object ch_object;
    CGAL::convex_hull_3(cgal_p3.begin(), cgal_p3.end(), ch_object);
    Polyhedron_3 polyhedron;
    CGAL::assign (polyhedron, ch_object);

    return polyhedron;
}

void TraversabilityGenerator3d::transformPolyhedron(Polyhedron_3& polyhedron, const Transformation& transform){
    // Apply the combined transformation to each point in the polyhedron
    std::transform(polyhedron.points_begin(), polyhedron.points_end(), polyhedron.points_begin(), transform);
}

Transformation TraversabilityGenerator3d::generateTransform(const Eigen::Vector3d& normal, const Eigen::Vector3d& translation){

    Eigen::Vector3d current_up(0, 0, 1);
    Eigen::Quaterniond rotation_quaternion = Eigen::Quaterniond::FromTwoVectors(current_up, normal);

    Eigen::Matrix3d rotation_matrix = rotation_quaternion.toRotationMatrix();

    Transformation rotate(
        rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2), 0,
        rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2), 0,
        rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2), 0
    );
    Vector_3 translation_vector(translation.x(), translation.y(), translation.z());
    Transformation translate(CGAL::TRANSLATION, translation_vector);

    Transformation combined = translate * rotate;
    return combined;    
}

TraversabilityGenerator3d::~TraversabilityGenerator3d()
{
    clearTrMap();
}

void TraversabilityGenerator3d::setInitialPatch(const Eigen::Affine3d& ground2Mls, double patchRadius)
{
    initialPatch2Mls = ground2Mls;
    addInitialPatch = true;

    this->patchRadius = patchRadius;

    if(mlsGrid)
        addInitialPatchToMLS();
}

const maps::grid::TraversabilityMap3d<TravGenNode *> & TraversabilityGenerator3d::getTraversabilityMap() const
{
    return trMap;
}

int TraversabilityGenerator3d::getNumNodes() const
{
    return currentNodeId;
}

bool TraversabilityGenerator3d::computePlaneRansac(TravGenNode& node)
{
    // Get node position from the grid
    Eigen::Vector3d nodePos;
    if (!trMap.fromGrid(node.getIndex(), nodePos, node.getHeight())) 
    {
        LOG_ERROR_S << "TraversabilityGenerator3d: Internal error node out of grid";
        return false;
    }

    // Calculate the search area dimensions
    const double growSize = std::min(config.robotSizeX, config.robotSizeY) / 2.0;
    const Eigen::Vector3d min(-growSize, -growSize, -config.maxStepHeight);
    const Eigen::Vector3d max(growSize, growSize, config.maxStepHeight);

    Eigen::AlignedBox3d searchArea(min + nodePos, max + nodePos);
    View area = mlsGrid->intersectCuboid(searchArea);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Compute the size and resolution of the area
    Eigen::Vector2d sizeHalf(area.getSize() / 2.0);
    const Eigen::Vector2d& res = mlsGrid->getResolution();

    // Count the number of patches in the area
    int patchCnt = 0;
    const int patchCntTotal = area.getNumCells().y() * area.getNumCells().x();

    for (size_t y = 0; y < area.getNumCells().y(); ++y) 
    {
        for (size_t x = 0; x < area.getNumCells().x(); ++x) 
        {
            Eigen::Vector2d pos = Eigen::Vector2d(x, y).cwiseProduct(res) - sizeHalf;
            bool hasPatch = false;
            for (const MLGrid::PatchType* p : area.at(x, y)) 
            {
                pcl::PointXYZ pclP(pos.x(), pos.y(), (p->getTop() + p->getBottom()) / 2.0);
                points->push_back(pclP);
                hasPatch = true;
            }

            if (hasPatch) 
            {
                ++patchCnt;
            }
        }
    }

    // Early exit if there are not enough patches
    if (patchCnt < 5) 
    {
        LOG_DEBUG_S << "Failed to fit plane using RANSAC because patchCnt < 5";
        return false;
    }

    if (patchCnt < patchCntTotal * config.minTraversablePercentage) 
    {
        LOG_DEBUG_S << "Not enough known patches: patchCnt " << patchCnt 
                    << " patchCntTotal " << patchCntTotal << " val " << patchCntTotal * config.minTraversablePercentage;
        return false;
    }

    // Perform RANSAC to fit a plane
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(points);
    seg.segment(inliers, coefficients);

    if (inliers.indices.size() <= 5) 
    {
        LOG_DEBUG_S << "RANSAC Failed: inliers->indices.size() <= 5";
        return false;
    }

    // Calculate the normal and distance from the origin for the plane
    Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    normal.normalize();
    double distToOrigin = coefficients.values[3];

    // Store the plane in the node's user data
    node.getUserData().plane = Eigen::Hyperplane<double, 3>(normal, distToOrigin);

    // Adjust the node's height based on the plane
    Eigen::ParametrizedLine<double, 3> line(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d newPos = line.intersectionPoint(node.getUserData().plane);

    if (newPos.x() > 0.0001 || newPos.y() > 0.0001) 
    {
        LOG_ERROR_S << "TraversabilityGenerator3d: Error, adjustment height calculation is weird";
        return false;
    }

    if (newPos.allFinite()) 
    {
        node.setHeight(newPos.z());
    }

    // Compute the slope and slope direction
    const Eigen::Vector3d slopeDir = computeSlopeDirection(node.getUserData().plane);
    node.getUserData().slope = computeSlope(node.getUserData().plane);
    node.getUserData().slopeDirection = slopeDir;
    node.getUserData().slopeDirectionAtan2 = std::atan2(slopeDir.y(), slopeDir.x());

    return true;
}

bool TraversabilityGenerator3d::computeAllowedOrientations(TravGenNode* node)
{

    if(node->getUserData().slope >= config.maxSlope)
        return false;


    if(node->getUserData().slope < config.inclineLimittingMinSlope)
    {
        //all orientations allowed below the slope limit
        node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(0), 2 * M_PI);
    }
    else
    {
        //only allow certail orientations because we are above the slope limit
        const double limitRad = interpolate(node->getUserData().slope, config.inclineLimittingMinSlope,
                                         M_PI_2, config.maxSlope, config.inclineLimittingLimit);
        const double startRad = node->getUserData().slopeDirectionAtan2 - limitRad;
        const double width = 2 * limitRad;
        assert(width >= 0);//this happens if the travmap was generated with a different maxSlope than config.maxSlope

        //add forward allowed angles
        node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad), width);

        //add backward allowed angles
        if(config.allowForwardDownhill)
        {
            node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad - M_PI), width);
        }
    }

    return true;
}



double TraversabilityGenerator3d::interpolate(double x, double x0, double y0, double x1, double y1)
{
    //linear interpolation
    return y0 + (x - x0) * (y1 - y0)/(x1-x0);
}


double TraversabilityGenerator3d::computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const
{
    const Eigen::Vector3d zNormal(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d planeNormal = plane.normal();
    planeNormal.normalize(); //just in case
    return acos(planeNormal.dot(zNormal));
}

Eigen::Vector3d TraversabilityGenerator3d::computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const
{
    /** The vector of maximum slope on a plane is the projection of (0,0,1) onto the plane.
     *  (0,0,1) is the steepest vector possible in the global frame, thus by projecting it onto
     *  the plane we get the steepest vector possible on that plane.
     */
    const Eigen::Vector3d zNormal(Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d planeNormal(plane.normal().normalized());
    const Eigen::Vector3d projection = zNormal - zNormal.dot(planeNormal) * planeNormal;
    return projection;
}


bool TraversabilityGenerator3d::checkForFrontier(const TravGenNode* node)
{
    //check direct neighborhood for missing connected patches. If
    //patches are missing, this patch is unknown
    using maps::grid::Index;
    const Index& index = node->getIndex();
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            if(x==0 && y==0) continue;
            const Index neighborIndex(index.x() + x, index.y() + y);
            const TravGenNode* neighbor = node->getConnectedNode(neighborIndex);
            if(neighbor == nullptr || neighbor->getType() == TraversabilityNodeBase::UNKNOWN)
            {
                return true;
            }
        }
    }
    return false;
}


void TraversabilityGenerator3d::drawWireFrameBox(const Eigen::Vector3d& normal, const Eigen::Vector3d& position, const Eigen::Vector3d& size, const Eigen::Vector4d& colorRGBA){
    Transformation transform = generateTransform(normal, position);

    Eigen::Matrix3d rotation_matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation_matrix(i, j) = transform.m(i, j); // Access rotation part of the matrix
        }
    }

    Eigen::Quaterniond orientation(rotation_matrix);
#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::COMPLEX_DRAWING([&]
    {
        V3DD::DRAW_WIREFRAME_BOX("traversability_generator3d_mls_patch_box", position, orientation, size, colorRGBA);
    });
#endif
}

bool TraversabilityGenerator3d::checkStepHeightAABB(TravGenNode *node)
{

    /** What this method does:
     * Check if any of the patches around @p node that the robot might stand on is higher than stepHeight.
     * I.e. if any of the patches is so high that it would be inside the robots body.
     */
    Eigen::Vector3d nodePos;
    if(!trMap.fromGrid(node->getIndex(), nodePos))
    {
        LOG_INFO_S << "TraversabilityGenerator3d: Internal error node out of grid";
        return false;
    }
    nodePos.z() += node->getHeight();

    const double smallerRobotSize = std::min(config.robotSizeX, config.robotSizeY);
    const double growSize = smallerRobotSize / 2.0 + 1e-5;

    Eigen::Vector3d min(-growSize, -growSize, 0);
    Eigen::Vector3d max(-min);
    max.z() = config.robotHeight;

    min += nodePos;
    max += nodePos;

    const Eigen::AlignedBox3d limitBox(min, max);
    View area = mlsGrid->intersectCuboid(limitBox);

    const Eigen::Hyperplane<double, 3> &plane(node->getUserData().plane);

    Index minIdx;
    Index maxIdx;


    if(!mlsGrid->toGrid(limitBox.min(), minIdx))
    {
        //when the robot bounding box leaves the map this patch cannot be traversable
        return false;
    }
    if(!mlsGrid->toGrid(limitBox.max(), maxIdx))
    {
        //when the robot bounding box leaves the map this patch cannot be traversable
        return false;
    }

    Index curIndex = minIdx;
    Index areaSize(maxIdx-minIdx);

    //the robot size has been set to a value smaller than one cell. Thus we cannot check anything.
    if(area.getNumCells().y() <= 0 || area.getNumCells().x() <= 0)
        return true;

    //intersectCuboid returns an area that is one patch bigger than requested. I.e.
    //it interpretes both min and max as inclusive. However, we consider max to be exclusive and
    //thus reduce the cell count by one
    for(size_t y = 0; y < area.getNumCells().y() - 1; y++, curIndex.y() += 1)
    {
        curIndex.x() = minIdx.x();
        for(size_t x = 0; x < area.getNumCells().x() - 1; x++, curIndex.x() += 1)
        {
            Eigen::Vector3d pos;
            if(!area.fromGrid(Index(x,y), pos))
            {
                LOG_ERROR_S << "Intersected area with MLS is invalid. This should never happen!";
            }

            for(const SurfacePatch<MLSConfig::SLOPE> *p : area.at(x, y))
            {
                pos.z() = (p->getTop()+p->getBottom())/2.;
                float dist = plane.absDistance(pos);
                //bounding box already checks height of robot
                if(dist > config.maxStepHeight)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

bool TraversabilityGenerator3d::checkStepHeightOBB(TravGenNode *node)
{
    /** What this method does:
     * Check if any of the patches within the robot OBB
     * @p node come into collison with the robot.
     */

    Eigen::Vector3d nodePos;
    if(!trMap.fromGrid(node->getIndex(), nodePos))
    {
        LOG_INFO_S << "TraversabilityGenerator3d: Internal error node out of grid";
        return false;
    }
    nodePos.z() += node->getHeight();

    Eigen::Vector3d robotCenterPos = nodePos;
    robotCenterPos.z() += config.maxStepHeight + config.robotHeight/2;

    Transformation transform = generateTransform(node->getUserData().plane.normal(), robotCenterPos);
    Polyhedron_3 robot = robotPolyhedron;
    transformPolyhedron(robot,transform);

    Eigen::Matrix3d rotation_matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation_matrix(i, j) = transform.m(i, j); // Access rotation part of the matrix
        }
    }

    Eigen::Quaterniond robotOrientation(rotation_matrix);
    Eigen::Vector3d robotSize{config.robotSizeX, config.robotSizeY, config.robotHeight};


    //TODO: Old code use for now to get the intersection with MLS
    const double smallerRobotSize = std::min(config.robotSizeX, config.robotSizeY);
    const double growSize = smallerRobotSize / 2.0 + 1e-5;

    Eigen::Vector3d min(-growSize, -growSize, 0);
    Eigen::Vector3d max(-min);
    max.z() = config.robotHeight;

    min += nodePos;
    max += nodePos;

    const Eigen::AlignedBox3d limitBox(min, max);
    View area = mlsGrid->intersectCuboid(limitBox);

    Index minIdx;
    Index maxIdx;

    if(!mlsGrid->toGrid(limitBox.min(), minIdx))
    {
        //when the robot bounding box leaves the map this patch cannot be traversable
        return false;
    }
    if(!mlsGrid->toGrid(limitBox.max(), maxIdx))
    {
        //when the robot bounding box leaves the map this patch cannot be traversable
        return false;
    }

    Index curIndex = minIdx;
    Index areaSize(maxIdx-minIdx);

    //the robot size has been set to a value smaller than one cell. Thus we cannot check anything.
    if(area.getNumCells().y() <= 0 || area.getNumCells().x() <= 0)
        return true;

    //intersectCuboid returns an area that is one patch bigger than requested. I.e.
    //it interpretes both min and max as inclusive. However, we consider max to be exclusive and
    //thus reduce the cell count by one
    for(size_t y = 0; y < area.getNumCells().y() - 1; y++, curIndex.y() += 1)
    {
        curIndex.x() = minIdx.x();
        for(size_t x = 0; x < area.getNumCells().x() - 1; x++, curIndex.x() += 1)
        {
            Eigen::Vector3d pos;
            if(!area.fromGrid(Index(x,y), pos))
            {
                LOG_INFO_S << "this should never happen";
            }

            //TODO: Fix the real cause of the offset!
            pos.x() += nodePos.x() - 1.5 * config.gridResolution;
            pos.y() += nodePos.y() - 1.5 * config.gridResolution;

            for(const SurfacePatch<MLSConfig::SLOPE> *p : area.at(x, y))
            {
                Eigen::Vector3f surface(0,0,0);
                pos.z() = p->getSurfacePos(surface);

                Polyhedron_3 patch = createPolyhedronFromSurfacePatch(p,pos);
                if(CGAL::Polygon_mesh_processing::do_intersect(patch,robot))
                {
#ifdef ENABLE_DEBUG_DRAWINGS
                    V3DD::COMPLEX_DRAWING([&]
                    {
                        V3DD::DRAW_WIREFRAME_BOX("traversability_generator3d_trav_obstacle_check_box", robotCenterPos, robotOrientation, robotSize, V3DD::Color::red);
                    });
                    Eigen::Vector3d patchSize{config.gridResolution, config.gridResolution, patchheight};
                    Eigen::Vector3f normalf = p->getNormal(); 
                    Eigen::Vector3d normal{normalf.x(), normalf.y(), normalf.z()};
                    drawWireFrameBox(normal,pos,patchSize,V3DD::Color::blue);
#endif
                    return false;
                }
            }
        }
    }

    return true;
    
}

Polyhedron_3 TraversabilityGenerator3d::createPolyhedronFromSurfacePatch(const SurfacePatch<MLSConfig::SLOPE> *p, const Eigen::Vector3d& position){

    // Extract the normal and center of the surface patch
    Eigen::Vector3f normalf = p->getNormal();  // Normal vector of the plane
    Eigen::Vector3d normal{normalf.x(), normalf.y(), normalf.z()};

    Polyhedron_3 patch = patchPolyhedron;
    Transformation transform = generateTransform(normal, position);
    transformPolyhedron(patch,transform);
    return patch;
}

void TraversabilityGenerator3d::growNodes()
{
    // Pre-compute the square of the grow radius once
    const double halfRobotSizeX = config.robotSizeX / 2.0;
    const double halfRobotSizeY = config.robotSizeY / 2.0;
    const double growRadiusSquared = (halfRobotSizeX * halfRobotSizeX) + (halfRobotSizeY * halfRobotSizeY);

    // Iterate over each node in the frontier grow list
    for (TravGenNode* node : frontierNodesGrowList)
    {
        const Eigen::Vector3d nodePos = node->getPosition(trMap);  // Cache the node's position

        node->eachConnectedNode([&](maps::grid::TraversabilityNodeBase* neighbor, bool& expandNode, bool& stop)
        {
            const Eigen::Vector3d neighborPos = neighbor->getPosition(trMap);  // Cache neighbor's position
            const double distanceSquared = (neighborPos - nodePos).squaredNorm();

            // Skip processing if the neighbor is outside the grow radius
            if (distanceSquared > growRadiusSquared)
            {
                stop = true;
            }

            expandNode = true;

            // Handle the different types of neighbors
            switch (neighbor->getType())
            {
                case TraversabilityNodeBase::FRONTIER:
                    if (node->getType() == TraversabilityNodeBase::OBSTACLE)
                    {
                        // If the current node is an obstacle, propagate its type to the frontier
                        neighbor->setType(node->getType());
                    }
                    break;

                case TraversabilityNodeBase::TRAVERSABLE:
                    // If the neighbor is traversable, propagate the current node's type
                    neighbor->setType(node->getType());
                    break;

                default:
                    break;
            }
        });
    }

    // Clear the list after processing all nodes
    frontierNodesGrowList.clear();
}

void TraversabilityGenerator3d::setConfig(const TraversabilityConfig &config)
{
    this->config = config;
}

void TraversabilityGenerator3d::expandAll(const Eigen::Vector3d& startPos)
{
    TravGenNode *startNode = generateStartNode(startPos);

    expandAll(startNode);
}

void TraversabilityGenerator3d::expandAll(const std::vector<Eigen::Vector3d>& positions)
{
    for(const Eigen::Vector3d& pos : positions)
    {
        expandAll(pos);
    }
}

void TraversabilityGenerator3d::expandAll(const Eigen::Vector3d& startPos, const double expandDist)
{
    TravGenNode *startNode = generateStartNode(startPos);
    expandAll(startNode, expandDist);
}

void TraversabilityGenerator3d::expandAll(TravGenNode* startNode)
{
    expandAll(startNode, -1.0);
}

void TraversabilityGenerator3d::expandAll(TravGenNode* startNode, const double expandDist)
{
    if (!startNode) 
        return;

    // Initialize a deque to act as the frontier for BFS.
    std::deque<TravGenNode *> candidates;
    candidates.push_back(startNode);
    int expandedNodeCount = 0;

    while (!candidates.empty()) 
    {
        TravGenNode *node = candidates.front();
        candidates.pop_front();

        // Skip nodes that have already been expanded
        if (node->isExpanded()) 
            continue;

        // Mark this node as expanded
        expandedNodeCount++;

        // Log progress every 1000 nodes
        if ((expandedNodeCount % 1000) == 0) 
        {
            LOG_DEBUG_S << "Expanded " << expandedNodeCount << " nodes";
        }

        // Attempt to expand the node
        if (!expandNode(node)) 
            continue;

        // Explore connected nodes
        for (auto* neighbor : node->getConnections()) 
        {
            // Only add non-expanded nodes to the candidates
            if (!neighbor->isExpanded()) 
            {
                // Apply distance constraint if provided
                if (expandDist > 0) 
                {
                    const double dist = (startNode->getPosition(trMap) - neighbor->getPosition(trMap)).norm();
                    if (dist <= expandDist) 
                    {
                        candidates.push_back(static_cast<TravGenNode *>(neighbor));  // Add to candidates if within range
                    }
                }
                else 
                {
                    candidates.push_back(static_cast<TravGenNode *>(neighbor));  // Add to candidates if no distance constraint
                }
            }
        }
    }

    // Grow frontier nodes and inflate obstacles after all nodes have been processed
    growNodes();
    inflateObstacles();

    LOG_DEBUG_S << "Expanded " << expandedNodeCount << " nodes";
}

void TraversabilityGenerator3d::inflateObstacles()
{
    // Calculate the inflation radius once using the robot's size
    const double halfRobotSizeX = config.robotSizeX / 2;
    const double halfRobotSizeY = config.robotSizeY / 2;
    const double inflationRadiusSquared = (halfRobotSizeX * halfRobotSizeX) + (halfRobotSizeY * halfRobotSizeY);

    // Iterate over each obstacle node in the grow list
    for (TravGenNode* node : obstacleNodesGrowList)
    {
        const Eigen::Vector3d nodePosition = node->getPosition(trMap);
        node->eachConnectedNode([&](maps::grid::TraversabilityNodeBase* neighbor, bool& expandNode, bool& stop)
        {

            const Eigen::Vector3d neighborPosition = neighbor->getPosition(trMap);
             // Compute the distance between the current node and the neighbor
            const double distanceSquared = (nodePosition - neighborPosition).squaredNorm();

            // If the neighbor is within the inflation radius, mark as an obstacle if it is traversable
            if (distanceSquared < inflationRadiusSquared)
            {
                if (neighbor->getType() == TraversabilityNodeBase::TRAVERSABLE)
                {
                    neighbor->setType(TraversabilityNodeBase::OBSTACLE);
                }
                expandNode = true;
            }
            else
            {
                // No need to continue processing once the neighbor is too far
                stop = true;
            }
        });
    }
    obstacleNodesGrowList.clear();
}

void TraversabilityGenerator3d::addInitialPatchToMLS()
{
    if(patchRadius == 0)
        return;


    LOG_DEBUG_S << "Adding Initial Patch";
    const Vector2d res = mlsGrid->getResolution();

    const double sizeHalfX = patchRadius;
    const double sizeHalfY = patchRadius;

    //we oversample by factor 2 to account for aliasing
    for(double x = -sizeHalfX; x <= sizeHalfX; x += res.x() / 2.0)
    {
        for(double y = -sizeHalfY; y <= sizeHalfY; y += res.y() / 2.0)
        {
            if(Vector2d(x,y).norm() > patchRadius)
                continue;

            Vector3d pos(x, y, 0);
            Vector3d posMLS = initialPatch2Mls * pos;

            Index idx;
            if(!mlsGrid->toGrid(posMLS, idx))
            {
                LOG_ERROR_S << "Something is wrong, initial position is outside of MLS !";
                continue;
            }

            auto &ll = mlsGrid->at(idx);

            bool hasPatch = false;
            for(const MLGrid::PatchType &p: ll)
            {
                if(p.isCovered(posMLS.z(), 0.05))
                {
                    hasPatch = true;
                    break;
                }
            }

            if(hasPatch)
                continue;

            MLGrid::PatchType newPatch(posMLS.cast<float>(), config.initialPatchVariance);
            ll.insert(newPatch);
        }
    }

}

void TraversabilityGenerator3d::setMLSGrid(std::shared_ptr< traversability_generator3d::TraversabilityGenerator3d::MLGrid >& grid)
{
    mlsGrid = grid;

    if(addInitialPatch)
    {
        addInitialPatchToMLS();
        addInitialPatch = false;
    }

    LOG_DEBUG_S << "Grid has size " << grid->getSize().transpose() << " resolution " << grid->getResolution().transpose();
    LOG_DEBUG_S << "Internal resolution is " << trMap.getResolution().transpose();
    Vector2d newSize = grid->getSize().array() / trMap.getResolution().array();
    LOG_DEBUG_S << "MLS was set " << grid->getResolution().transpose() << " " << mlsGrid->getResolution().transpose();

    LOG_DEBUG_S << "New Size is " << newSize.transpose();

    trMap.extend(Vector2ui(newSize.x(), newSize.y()));

    trMap.getLocalFrame() = mlsGrid->getLocalFrame();

    clearTrMap();
}

void TraversabilityGenerator3d::clearTrMap()
{
    // Iterate through each level in the trMap
    for (LevelList<TravGenNode*>& level : trMap)
    {
        // Iterate through each node in the level and delete it
        for (TravGenNode* node : level)
        {
            delete node;  // Safely delete each node
        }

        level.clear();  // Clear the list of nodes in this level after deletion
    }
}

TravGenNode* TraversabilityGenerator3d::generateStartNode(const Eigen::Vector3d& startPos)
{
    Index idx;
    if(!trMap.toGrid(startPos, idx))
    {
        LOG_ERROR_S << "TraversabilityGenerator3d::generateStartNode: Start position outside of map !";
        return nullptr;
    }

    //check if not already exists...
    TravGenNode *startNode = findMatchingTraversabilityPatchAt(idx, startPos.z());
    if(startNode)
    {
        LOG_DEBUG_S << "TraversabilityGenerator3d::generateStartNode: Using existing node ";
        return startNode;
    }

    startNode = createTraversabilityPatchAt(idx, startPos.z());
    if(!startNode)
    {
        LOG_ERROR_S << "TraversabilityGenerator3d::generateStartNode: Could not create travNode for given start position, no matching / not enough MLS patches";
        return startNode;
    }

    if(startNode->isExpanded() && startNode->getType() != TraversabilityNodeBase::TRAVERSABLE)
    {
        LOG_ERROR_S << "TraversabilityGenerator3d::generateStartNode: Position is on non traversable patch!";
        LOG_ERROR_S << "Type of start patch is: " << startNode->getType();
    }

    return startNode;
}


bool TraversabilityGenerator3d::expandNode(TravGenNode * node)
{
    node->setExpanded();

    if(node->getType() == TraversabilityNodeBase::UNKNOWN)
    {
        return false;
    }

    if(node->getType() == TraversabilityNodeBase::OBSTACLE)
    {
        obstacleNodesGrowList.push_back(node);
        return false;
    }

    if(node->getUserData().slope > config.maxSlope){
        return false;
    }

    if(!checkStepHeightAABB(node))
    {
        if(!checkStepHeightOBB(node))
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            obstacleNodesGrowList.push_back(node);
            return false;
        }
    } 
    
    if(config.enableInclineLimitting)
    {
        if(!computeAllowedOrientations(node))
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            obstacleNodesGrowList.push_back(node);
            return false;
        }
    }

    //add surrounding
    addConnectedPatches(node);

    if(checkForFrontier(node))
    {
        node->setType(TraversabilityNodeBase::FRONTIER);
        frontierNodesGrowList.push_back(node);
        return false;
    }

    node->setType(TraversabilityNodeBase::TRAVERSABLE);

    return true;
}

TravGenNode *TraversabilityGenerator3d::createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight)
{
    TravGenNode *ret = nullptr;

    maps::grid::Vector3d globalPos;
    trMap.fromGrid(idx, globalPos);
    Index mlsIdx;
    if(!mlsGrid->toGrid(globalPos, mlsIdx))
    {
        return nullptr;
    }

    const auto& patches = mlsGrid->at(mlsIdx);

    std::vector<double> candidates;

    for(const SurfacePatch<MLSConfig::SLOPE>& patch : patches)
    {
        //We use top, as we drive on the surface
        const double height = (patch.getTop()+patch.getBottom())/2.;

        if((height - config.maxStepHeight) <= curHeight && (height + config.maxStepHeight) >= curHeight)
        {
            candidates.push_back(height);
        }
        if(height > (curHeight + config.maxStepHeight))
            break;
    }

    //Also add the interpolated height, to fill in small holes
    //if there is no support, the ransac will filter the node out
    candidates.push_back(curHeight);

    ret = new TravGenNode(0.0, idx);
    ret->getUserData().id = currentNodeId++;

    for(double height: candidates)
    {
        ret->setHeight(height);
        ret->setNotExpanded();
        ret->setType(TraversabilityNodeBase::UNSET);

        //there is a neighboring patch in the mls that has a reachable hight
        if(!computePlaneRansac(*ret))
        {
            if(mlsIdx.x() == 1 || mlsIdx.y() == 1) {
                ret->setType(TraversabilityNodeBase::OBSTACLE);
            } else {
                ret->setType(TraversabilityNodeBase::UNKNOWN);
            }
        }

        if((ret->getHeight() - config.maxStepHeight) <= curHeight && (ret->getHeight() + config.maxStepHeight) >= curHeight)
        {
            trMap.at(idx).insert(ret);
            return ret;
        }
        else
        {
            //rare border case, ransac correction moved patch out of reachable height
        }
    }

    ret->setHeight(curHeight);
    ret->setNotExpanded();
    ret->setType(TraversabilityNodeBase::OBSTACLE);
    trMap.at(idx).insert(ret);
    return ret;
}

TravGenNode* TraversabilityGenerator3d::findMatchingTraversabilityPatchAt(Index idx, const double curHeight) const
{
    auto &trList(trMap.at(idx));

    //check if we got an existing node
    for(TravGenNode *snode : trList)
    {
        const double searchHeight = snode->getHeight();
        if((searchHeight - config.maxStepHeight) <= curHeight && (searchHeight + config.maxStepHeight) >= curHeight)
        {
            //found a connectable node
            return snode;
        }

        if(searchHeight > curHeight)
        {
            return nullptr;
        }
    }

    return nullptr;
}

void TraversabilityGenerator3d::addConnectedPatches(TravGenNode *  node)
{
    static std::vector<Index> surounding = {
        Index(1, 1),
        Index(1, 0),
        Index(1, -1),
        Index(0, 1),
        Index(0, -1),
        Index(-1, 1),
        Index(-1, 0),
        Index(-1, -1)};

    double curHeight = node->getHeight();
    for(const Index &idxS : surounding)
    {
        const Index idx(node->getIndex() + idxS);

        if(!trMap.inGrid(idx))
        {
            continue;
        }

        //compute height of cell in respect to plane
        const Vector3d patchPosPlane(idxS.x() * trMap.getResolution().x(), idxS.y() * trMap.getResolution().y(), 0);
        const Eigen::ParametrizedLine<double, 3> line(patchPosPlane, Eigen::Vector3d::UnitZ());
        const Vector3d newPos =  line.intersectionPoint(node->getUserData().plane);

        //this happend at some point because of a bug somewhere else.
        //not sure if this can still happen.
        if((patchPosPlane.head(2) - newPos.head(2)).norm() > 0.001)
        {
            LOG_INFO_S << "TraversabilityGenerator3d: FATAL ERROR, adjustment height calculation is weird";
            return;
        }

        const double localHeight = newPos.z();
        //The new patch is not reachable from the current patch
        if(fabs(localHeight - curHeight) > config.maxStepHeight)
        {
            continue;
        }


        TravGenNode *toAdd = nullptr;

        if(!newPos.allFinite())
        {
            LOG_ERROR_S << "newPos contains inf values";
            continue;
        }

        //check if we got an existing node
        toAdd = findMatchingTraversabilityPatchAt(idx, localHeight);

        //no existing node exists at that location.
        //try to create a new one at the position
        if(!toAdd)
        {
            toAdd = createTraversabilityPatchAt(idx, localHeight);
        }

        if(toAdd)
        {
            auto& connections = toAdd->getConnections();
            if(std::find(connections.begin(), connections.end(), node) == connections.end())
                toAdd->addConnection(node);

            auto& connections2 = node->getConnections();
            if(std::find(connections2.begin(), connections2.end(), toAdd) == connections2.end())
                node->addConnection(toAdd);
        }
    }
}
}