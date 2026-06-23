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

TraversabilityGenerator3d::TraversabilityGenerator3d(const TraversabilityConfig& config) 
    : addInitialPatch(false), config(config), patchHeight(0.02) // Set default patchHeight
{
    trMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));
    soilMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));

    double robotHalfLength = config.robotSizeX / 2.0;
    double robotHalfWidth = config.robotSizeY / 2.0;
    double robotHalfHeight = config.robotHeight / 2.0;

    robotEdges = {
        {robotHalfLength, robotHalfWidth, robotHalfHeight},    // Top-right-front
        {robotHalfLength, robotHalfWidth, -robotHalfHeight},   // Top-right-back
        {robotHalfLength, -robotHalfWidth, robotHalfHeight},   // Bottom-right-front
        {robotHalfLength, -robotHalfWidth, -robotHalfHeight},  // Bottom-right-back
        {-robotHalfLength, robotHalfWidth, robotHalfHeight},   // Top-left-front
        {-robotHalfLength, robotHalfWidth, -robotHalfHeight},  // Top-left-back
        {-robotHalfLength, -robotHalfWidth, robotHalfHeight},  // Bottom-left-front
        {-robotHalfLength, -robotHalfWidth, -robotHalfHeight}  // Bottom-left-back
    };

    robotPolyhedron = generatePolyhedron(robotEdges);

    double patchHalfLength = config.gridResolution / 2.0;
    double patchHalfWidth = config.gridResolution / 2.0;
    double patchHalfHeight = patchHeight / 2.0;

    patchEdges = {
        {patchHalfLength, patchHalfWidth, patchHalfHeight},    // Top-right-front
        {patchHalfLength, patchHalfWidth, -patchHalfHeight},   // Top-right-back
        {patchHalfLength, -patchHalfWidth, patchHalfHeight},   // Bottom-right-front
        {patchHalfLength, -patchHalfWidth, -patchHalfHeight},  // Bottom-right-back
        {-patchHalfLength, patchHalfWidth, patchHalfHeight},   // Top-left-front
        {-patchHalfLength, patchHalfWidth, -patchHalfHeight},  // Top-left-back
        {-patchHalfLength, -patchHalfWidth, patchHalfHeight},  // Bottom-left-front
        {-patchHalfLength, -patchHalfWidth, -patchHalfHeight}  // Bottom-left-back
    };

    patchPolyhedron = generatePolyhedron(patchEdges);
}

Polyhedron_3 TraversabilityGenerator3d::generatePolyhedron(const std::vector<Eigen::Vector3d>& points) {
    std::vector<Point_3> cgal_p3;
    cgal_p3.reserve(points.size());

    std::transform(points.begin(), points.end(), std::back_inserter(cgal_p3),
                   [](const Eigen::Vector3d& v) { return Point_3(v.x(), v.y(), v.z()); });

    Polyhedron_3 polyhedron;
    CGAL::convex_hull_3(cgal_p3.begin(), cgal_p3.end(), polyhedron);

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
    clearSoilMap();
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

const maps::grid::TraversabilityMap3d<SoilNode *> & TraversabilityGenerator3d::getSoilMap() const
{
    return soilMap;
}

int TraversabilityGenerator3d::getNumNodes() const
{
    return currentNodeId;
}


bool TraversabilityGenerator3d::computePlaneRansac(TravGenNode& node)
{
    Eigen::Vector3d nodePos;

    if (!trMap.fromGrid(node.getIndex(), nodePos, node.getHeight())) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Node index " << node.getIndex()
                    << " with height " << node.getHeight()
                    << " is outside of the traversability grid.";
        return false;
    }

    const double growSize = std::min(config.robotSizeX, config.robotSizeY) / 2.0;

    //get all surfaces in a cube of robotwidth and stepheight
    Eigen::Vector3d min(-growSize, -growSize, -config.maxStepHeight);
    Eigen::Vector3d max(-min);

    min += nodePos;
    max += nodePos;
    const Eigen::AlignedBox3d searchArea(min, max);
    View area = mlsGrid->intersectCuboid(searchArea);


    typedef pcl::PointXYZ PointT;

    pcl::PointCloud<PointT>::Ptr points(new pcl::PointCloud<PointT>());

    Eigen::Vector2d sizeHalf(area.getSize() / 2.0);

    const Eigen::Vector2d& res = mlsGrid->getResolution();


    const int patchCntTotal = area.getNumCells().y() * area.getNumCells().x(); //FIXME only works if there is only one patch per cell
    int patchCnt = 0;
    for(size_t y = 0; y < area.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < area.getNumCells().x(); x++)
        {
            Eigen::Vector2d pos = Eigen::Vector2d(x,y).cwiseProduct(res) - sizeHalf;

            bool hasPatch = false;
            for(const MLGrid::PatchType *p : area.at(x, y))
            {
                PointT pclP(pos.x(), pos.y(), (p->getTop()+p->getBottom())/2.);
                points->push_back(pclP);
                hasPatch = true;
            }

            if(hasPatch)
                patchCnt++;
        }
    }


    //if less than 5 planes -> hole
    //TODO where to implement ? here or in check obstacles ?
    if(patchCnt < 5)
    {
        //ransac will not produce a result below 5 points
        LOG_DEBUG_S << "TraversabilityGenerator3d: RANSAC plane fitting skipped: only " << patchCnt
            << " patches available (minimum required: 5)";
        return false;
    }

    //filter out to sparse areas
    if(patchCnt < patchCntTotal * config.minTraversablePercentage)
    {
        LOG_DEBUG_S << "TraversabilityGenerator3d: insufficient patch density — "
                    << patchCnt << "/" << patchCntTotal 
                    << " patches known (" << (100.0 * patchCnt / patchCntTotal) << "%), "
                    << "minimum required: " << (100.0 * config.minTraversablePercentage) << "%";
        return false;
    }

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (50);
    seg.setDistanceThreshold (0.1);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (points);
    seg.segment (inliers, coefficients);

    if (inliers.indices.size() <= 5) {
        LOG_DEBUG_S << "TraversabilityGenerator3d: RANSAC failed: only " << inliers.indices.size()
                    << " inliers found (minimum required: 6)";
        return false;
    }

    Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    normal.normalize();
    double distToOrigin = coefficients.values[3];

    node.getUserData().plane = Eigen::Hyperplane<double, 3>(normal, distToOrigin);

    //adjust height of patch
    Eigen::ParametrizedLine<double, 3> line(Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    Vector3d newPos =  line.intersectionPoint(node.getUserData().plane);

    if (std::abs(newPos.x()) > 0.0001 || std::abs(newPos.y()) > 0.0001) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Adjustment height calculation failed. "
                    << "Expected near-zero offset, but got newPos=(" 
                    << newPos.x() << ", " << newPos.y() << ")";
        return false;
    }

    if(newPos.allFinite())
    {
        node.setHeight(newPos.z());
    }

    const Eigen::Vector3d slopeDir = computeSlopeDirection(node.getUserData().plane);
    node.getUserData().slope = computeSlope(node.getUserData().plane);
    node.getUserData().slopeDirection = slopeDir;
    node.getUserData().slopeDirectionAtan2 = std::atan2(slopeDir.y(), slopeDir.x());

//#ifdef ENABLE_DEBUG_DRAWINGS
//    V3DD::COMPLEX_DRAWING([&]()
//    {    Eigen::Vector3d pos(node.getIndex().x() * config.gridResolution, node.getIndex().y() * config.gridResolution, node.getHeight());
//        pos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//        pos.z() += 0.06;
//        V3DD::DRAW_TEXT("slope", pos, std::to_string(node.getUserData().slope), 0.01, V3DD::Color::red);
//    });
//#endif

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
//         const base::AngleSegment segment(base::Angle::fromRad(startRad), width);
//         const base::AngleSegment segmentMirrored(base::Angle::fromRad(startRad - M_PI), width);

        //add forward allowed angles
        node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad), width);

//#ifdef ENABLE_DEBUG_DRAWINGS
//        V3DD::COMPLEX_DRAWING([&]()
//        {
//            Eigen::Vector3d patchPos(node->getIndex().x() * config.gridResolution, node->getIndex().y() * config.gridResolution, node->getHeight());
//            patchPos.x() += config.gridResolution / 2.0;
//            patchPos.y() += config.gridResolution / 2.0;
//            patchPos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * patchPos;
//            patchPos.z() += 0.06;
//            Eigen::AngleAxisd rot1(node->getUserData().allowedOrientations.back().getStart().getRad(), Eigen::Vector3d::UnitZ());
//            Eigen::AngleAxisd rot2(node->getUserData().allowedOrientations.back().getEnd().getRad(), Eigen::Vector3d::UnitZ());
//            Eigen::Vector3d end1 = rot1 * Eigen::Vector3d(0.1, 0, 0);
//            Eigen::Vector3d end2 = rot2 * Eigen::Vector3d(0.1, 0, 0);
//            V3DD::DRAW_LINE("traversability_generator3d_allowedAngles", patchPos, patchPos + end1, V3DD::Color::magenta);
//            V3DD::DRAW_LINE("traversability_generator3d_allowedAngles", patchPos, patchPos + end2, V3DD::Color::magenta);
//        });
//#endif

        //add backward allowed angles
        if(config.allowForwardDownhill)
        {
            node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad - M_PI), width);

//#ifdef ENABLE_DEBUG_DRAWINGS
//        V3DD::COMPLEX_DRAWING([&]()
//        {
//            Eigen::Vector3d patchPos(node->getIndex().x() * config.gridResolution, node->getIndex().y() * config.gridResolution, node->getHeight());
//            patchPos.x() += config.gridResolution / 2.0;
//            patchPos.y() += config.gridResolution / 2.0;
//            patchPos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * patchPos;
//            patchPos.z() += 0.06;
//            Eigen::AngleAxisd rot1(node->getUserData().allowedOrientations.back().getStart().getRad(), Eigen::Vector3d::UnitZ());
//            Eigen::AngleAxisd rot2(node->getUserData().allowedOrientations.back().getEnd().getRad(), Eigen::Vector3d::UnitZ());
        
//             Eigen::Vector3d end1 = rot1 * Eigen::Vector3d(0.1, 0, 0);
//             Eigen::Vector3d end2 = rot2 * Eigen::Vector3d(0.1, 0, 0);
        
//             V3DD::DRAW_LINE("traversability_generator3d_allowedAngles", patchPos, patchPos + end1, V3DD::Color::cyan);
//             V3DD::DRAW_LINE("traversability_generator3d_allowedAngles", patchPos, patchPos + end2, V3DD::Color::cyan);
//        });
//#endif
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

double TraversabilityGenerator3d::sampleTerrainHeightAtCorner(const Eigen::Vector3d& nodePos, double cornerX, double cornerY) const
{
    Eigen::Vector3d cornerPos = nodePos;
    cornerPos.x() += cornerX;
    cornerPos.y() += cornerY;

    const double searchRadius = config.gridResolution;
    const double robotDiagHalf = std::sqrt(config.robotSizeX * config.robotSizeX +
                                           config.robotSizeY * config.robotSizeY) / 2.0;
    const double vertSearchRange = robotDiagHalf * std::sin(config.maxSlope) + config.maxStepHeight;
    Eigen::Vector3d searchMin = cornerPos - Eigen::Vector3d(searchRadius, searchRadius, vertSearchRange);
    Eigen::Vector3d searchMax = cornerPos + Eigen::Vector3d(searchRadius, searchRadius, vertSearchRange);

    View area = mlsGrid->intersectCuboid(Eigen::AlignedBox3d(searchMin, searchMax));

    Index minIdx;
    if (!mlsGrid->toGrid(searchMin, minIdx))
    {
        Index cornerIdx;
        if (mlsGrid->toGrid(cornerPos, cornerIdx))
        {
            minIdx = Index(cornerIdx.x() - 1, cornerIdx.y() - 1);
        }
        else
        {
            minIdx = Index(0, 0);
        }
    }

    double minDistance2DSq = std::numeric_limits<double>::max();
    double bestHeight = nodePos.z() - vertSearchRange;

    for(size_t y = 0; y < area.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < area.getNumCells().x(); x++)
        {
            Index curIndex = minIdx + Index(x, y);
            Eigen::Vector3d cellPos;
            if (!mlsGrid->fromGrid(curIndex, cellPos))
                continue;

            double dx = cellPos.x() - cornerPos.x();
            double dy = cellPos.y() - cornerPos.y();
            double dist2DSq = dx * dx + dy * dy;

            for(const SurfacePatch<MLSConfig::SLOPE> *p : area.at(x, y))
            {
                // Check if it's a valid ground patch (not a wall/steep slope)
                Eigen::Vector3f normalf = p->getNormal();
                Eigen::Vector3d normal{normalf.x(), normalf.y(), normalf.z()};
                normal.normalize();
                if (std::abs(normal.z()) < std::cos(config.maxSlope))
                    continue;

                double h = (p->getTop() + p->getBottom()) / 2.0;
                if(std::abs(h - nodePos.z()) <= vertSearchRange)
                {
                    // Prioritize closer cells in 2D
                    if (dist2DSq < minDistance2DSq)
                    {
                        minDistance2DSq = dist2DSq;
                        bestHeight = h;
                    }
                    // If they are in the same cell, take the one closer in height to the center nodePos
                    else if (std::abs(dist2DSq - minDistance2DSq) < 1e-5)
                    {
                        if (std::abs(h - nodePos.z()) < std::abs(bestHeight - nodePos.z()))
                        {
                            bestHeight = h;
                        }
                    }
                }
            }
        }
    }
    return bestHeight;
}

Eigen::Vector3d TraversabilityGenerator3d::computeContactPlaneFromCorners(const std::vector<Eigen::Vector3d>& cornerPositions)
{
    // Fit plane through corner points using PCA
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for(const auto& p : cornerPositions)
        centroid += p;
    centroid /= static_cast<double>(cornerPositions.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for(const auto& p : cornerPositions)
    {
        Eigen::Vector3d centered = p - centroid;
        cov += centered * centered.transpose();
    }
    cov /= static_cast<double>(cornerPositions.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0); // smallest eigenvalue = plane normal
    normal.normalize();
    if(normal.z() < 0)
        normal = -normal;
    return normal;
}

std::vector<Eigen::Vector3d> TraversabilityGenerator3d::compute4PointContactPositions(const Eigen::Vector3d& nodePos)
{
    // Returns 4 lower OBB corners in perimeter order: +X+Y, +X-Y, -X-Y, -X+Y
    // Each at terrain height + maxStepHeight (bottom of robot body)
    const double hx = config.robotSizeX / 2.0;
    const double hy = config.robotSizeY / 2.0;
    return {
        {nodePos.x() + hx, nodePos.y() + hy, sampleTerrainHeightAtCorner(nodePos,  hx,  hy) + config.maxStepHeight},
        {nodePos.x() + hx, nodePos.y() - hy, sampleTerrainHeightAtCorner(nodePos,  hx, -hy) + config.maxStepHeight},
        {nodePos.x() - hx, nodePos.y() - hy, sampleTerrainHeightAtCorner(nodePos, -hx, -hy) + config.maxStepHeight},
        {nodePos.x() - hx, nodePos.y() + hy, sampleTerrainHeightAtCorner(nodePos, -hx,  hy) + config.maxStepHeight},
    };
}

bool TraversabilityGenerator3d::checkForFrontier(const TravGenNode* node)
{
    //check direct neighborhood for missing connected patches. If
    //patches are missing, this patch is unknown

    for(maps::grid::TraversabilityNodeBase* n : node->getConnections())
    {
        if(n == nullptr || n->getType() == TraversabilityNodeBase::UNKNOWN)
        {
            return true;
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
    /*
    V3DD::COMPLEX_DRAWING([&]
    {
        V3DD::DRAW_WIREFRAME_BOX("traversability_generator3d_mls_patch_box", position, orientation, size, colorRGBA);
    });
    */
#endif
}

bool TraversabilityGenerator3d::checkStepHeightAABB(TravGenNode *node)
{

    /** What this method does:
     * Check if any of the patches around @p node that the robot might stand on is higher than stepHeight.
     * I.e. if any of the patches is so high that it would be inside the robots body.
     */

    Eigen::Vector3d nodePos;
    if (!trMap.fromGrid(node->getIndex(), nodePos)) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Node index "
                    << node->getIndex()
                    << " is outside the traversability grid.";
        return false;
    }
    nodePos.z() += node->getHeight();

    // Use the smaller half-dimension as a uniform search radius on both axes.
    // This keeps the AABB square and avoids a large dead-zone near map boundaries
    // when robotSizeX >> robotSizeY (or vice-versa).  Cells closer than
    // min(halfX, halfY) to any obstacle are already obstacle-free by construction;
    // the remaining gap to the rotation-safe circle is covered by inflateObstacles.
    const double halfSmall = std::min(config.robotSizeX, config.robotSizeY) / 2.0;
    Eigen::Vector3d min(-halfSmall, -halfSmall, 0);
    Eigen::Vector3d max( halfSmall,  halfSmall, config.robotHeight);

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
            // Convert grid index directly to world coordinates
            if (!mlsGrid->fromGrid(curIndex, pos)) {
                LOG_ERROR_S << "TraversabilityGenerator3d: fromGrid failed for grid index "
                            << curIndex << " — index outside MLS grid bounds.";
                continue;
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
    if (!trMap.fromGrid(node->getIndex(), nodePos)) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Node index "
                    << node->getIndex()
                    << " is outside of the traversability grid bounds.";
        return false;
    }
    nodePos.z() += node->getHeight();

    // Get 4 corner positions based on local terrain heights
    std::vector<Eigen::Vector3d> cornerPositions = compute4PointContactPositions(nodePos);
    
    if(cornerPositions.size() != 4)
    {
        LOG_WARN_S << "compute4PointContactPositions returned " << cornerPositions.size() << " corners instead of 4";
        return true;
    }
    
    // Build robot edges from 4-point contact geometry:
    // Lower 4 corners (from computed positions) + Upper 4 corners (at lower + height offset)
    std::vector<Eigen::Vector3d> robotEdges4Point;
    for(const auto& corner : cornerPositions)
    {
        robotEdges4Point.push_back(corner);
    }
    
    // Compute contact plane normal from 4 corners
    Eigen::Vector3d contactNormal = computeContactPlaneFromCorners(cornerPositions);
    
    // Use contact normal directly for height offset - robot tilts to match terrain slope
    Eigen::Vector3d heightOffset = contactNormal * config.robotHeight;
    
    // Add upper 4 corners (offset from lower corners by robot height along contact normal)
    for(size_t i = 0; i < 4; i++)
    {
        robotEdges4Point.push_back(cornerPositions[i] + heightOffset);
    }
    
    // Build polyhedron from these 8 corners
    Polyhedron_3 robot = generatePolyhedron(robotEdges4Point);

    // Query MLS patches across the entire unrotated robot footprint bounding box
    const double halfX = config.robotSizeX / 2.0;
    const double halfY = config.robotSizeY / 2.0;
    Eigen::Vector3d min(-halfX, -halfY, 0);
    Eigen::Vector3d max( halfX,  halfY, config.robotHeight);

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
            // Convert grid index directly to world coordinates instead of using view-local coords
            if (!mlsGrid->fromGrid(curIndex, pos)) {
                LOG_ERROR_S << "TraversabilityGenerator3d: fromGrid failed for grid index " 
                            << curIndex << " — index outside MLS grid bounds.";
                continue;
            }

            for(const SurfacePatch<MLSConfig::SLOPE> *p : area.at(x, y))
            {
                pos.z() = (p->getTop() + p->getBottom()) / 2.0;

                Polyhedron_3 patch = createPolyhedronFromSurfacePatch(p,pos);
                if(CGAL::Polygon_mesh_processing::do_intersect(patch,robot))
                {
#ifdef ENABLE_DEBUG_DRAWINGS
                    /*
                    Eigen::Vector3f normalf = p->getNormal(); 
                    Eigen::Vector3d normal{normalf.x(), normalf.y(), normalf.z()};
                    normal.normalize();

                    Eigen::Vector3d vecV;
                    double horizNorm = std::sqrt(normal.x() * normal.x() + normal.y() * normal.y());
                    if (horizNorm > 1e-5)
                    {
                        vecV = Eigen::Vector3d(-normal.y(), normal.x(), 0.0) / horizNorm;
                    }
                    else
                    {
                        vecV = Eigen::Vector3d(0.0, 1.0, 0.0);
                    }

                    double halfThickness = 0.01;
                    double halfWidth = config.gridResolution / 2.0;
                    double h_val = std::abs(normal.z()) * config.gridResolution + (1.0 - std::abs(normal.z())) * (p->getTop() - p->getBottom());
                    double halfHeight = std::max(0.02, h_val) / 2.0;

                    Eigen::Vector3d vecThickness = normal * halfThickness;
                    Eigen::Vector3d vecWidth = vecV * halfWidth;
                    Eigen::Vector3d vecHeight = normal.cross(vecV) * halfHeight;

                    std::vector<Eigen::Vector3d> corners = {
                        pos + vecThickness + vecWidth + vecHeight,
                        pos + vecThickness + vecWidth - vecHeight,
                        pos + vecThickness - vecWidth + vecHeight,
                        pos + vecThickness - vecWidth - vecHeight,
                        pos - vecThickness + vecWidth + vecHeight,
                        pos - vecThickness + vecWidth - vecHeight,
                        pos - vecThickness - vecWidth + vecHeight,
                        pos - vecThickness - vecWidth - vecHeight
                    };

                    Eigen::Vector4d blue{0.0, 0.0, 1.0, 1.0};
                    std::string prefix = "traversability_generator3d_mls_patch_box";

                    V3DD::COMPLEX_DRAWING([&]
                    {
                        V3DD::DRAW_LINE(prefix + "_v0", corners[1], corners[0], blue);
                        V3DD::DRAW_LINE(prefix + "_v1", corners[3], corners[2], blue);
                        V3DD::DRAW_LINE(prefix + "_v2", corners[5], corners[4], blue);
                        V3DD::DRAW_LINE(prefix + "_v3", corners[7], corners[6], blue);
                        V3DD::DRAW_LINE(prefix + "_l0", corners[1], corners[3], blue);
                        V3DD::DRAW_LINE(prefix + "_l1", corners[3], corners[7], blue);
                        V3DD::DRAW_LINE(prefix + "_l2", corners[7], corners[5], blue);
                        V3DD::DRAW_LINE(prefix + "_l3", corners[5], corners[1], blue);
                        V3DD::DRAW_LINE(prefix + "_u0", corners[0], corners[2], blue);
                        V3DD::DRAW_LINE(prefix + "_u1", corners[2], corners[6], blue);
                        V3DD::DRAW_LINE(prefix + "_u2", corners[6], corners[4], blue);
                        V3DD::DRAW_LINE(prefix + "_u3", corners[4], corners[0], blue);
                    });
                    */
#endif
                    return false;
                }
            }
        }
    }

#ifdef ENABLE_DEBUG_DRAWINGS
    /*
    {
        static int boxCounter = 0;
        if(boxCounter++ % 50 == 0)
        {
            V3DD::COMPLEX_DRAWING([&]
            {
                Eigen::Vector4d darkBrown{0.4, 0.25, 0.1, 1.0};
                std::string prefix = "exact_obb_" + std::to_string(boxCounter);
                
                for(size_t i = 0; i < 4; i++)
                {
                    Eigen::Vector3d lo = cornerPositions[i];
                    Eigen::Vector3d hi = cornerPositions[i] + heightOffset;
                    size_t next = (i + 1) % 4;
                    Eigen::Vector3d loNext = cornerPositions[next];
                    Eigen::Vector3d hiNext = cornerPositions[next] + heightOffset;
                    
                    // Vertical edge
                    V3DD::DRAW_LINE(prefix + "_v" + std::to_string(i), lo, hi, darkBrown);
                    // Lower perimeter edge
                    V3DD::DRAW_LINE(prefix + "_lo" + std::to_string(i), lo, loNext, darkBrown);
                    // Upper perimeter edge
                    V3DD::DRAW_LINE(prefix + "_hi" + std::to_string(i), hi, hiNext, darkBrown);
                }
            });
        }
    }
    */
#endif

    return true;
}

Polyhedron_3 TraversabilityGenerator3d::createPolyhedronFromSurfacePatch(const SurfacePatch<MLSConfig::SLOPE> *p, const Eigen::Vector3d& position){

    // Extract the normal and center of the surface patch
    Eigen::Vector3f normalf = p->getNormal();  // Normal vector of the plane
    Eigen::Vector3d normal{normalf.x(), normalf.y(), normalf.z()};
    normal.normalize();

    // Dynamically calculate the patch height/thickness from MLS patch bounds,
    // ensuring a minimum height of 0.02m to avoid degenerate flat polyhedrons.
    double currentPatchHeight = std::max(0.02, (double)(p->getTop() - p->getBottom()));

    double patchHalfLength = config.gridResolution / 2.0;
    double patchHalfWidth = config.gridResolution / 2.0;
    double patchHalfHeight = currentPatchHeight / 2.0;

    std::vector<Eigen::Vector3d> dynamicPatchEdges = {
        {patchHalfLength, patchHalfWidth, patchHalfHeight},    // Top-right-front
        {patchHalfLength, patchHalfWidth, -patchHalfHeight},   // Top-right-back
        {patchHalfLength, -patchHalfWidth, patchHalfHeight},   // Bottom-right-front
        {patchHalfLength, -patchHalfWidth, -patchHalfHeight},  // Bottom-right-back
        {-patchHalfLength, patchHalfWidth, patchHalfHeight},   // Top-left-front
        {-patchHalfLength, patchHalfWidth, -patchHalfHeight},  // Top-left-back
        {-patchHalfLength, -patchHalfWidth, patchHalfHeight},  // Bottom-left-front
        {-patchHalfLength, -patchHalfWidth, -patchHalfHeight}  // Bottom-left-back
    };

    Polyhedron_3 patch;
    // If the patch normal is close to vertical (ground/slope), rotate to match the slope.
    // Otherwise, for near-vertical wall patches (horizontal normal), keep the column vertical (no rotation).
    if (std::abs(normal.z()) > 0.707)
    {
        patch = generatePolyhedron(dynamicPatchEdges);
        Transformation transform = generateTransform(normal, position);
        transformPolyhedron(patch, transform);
    }
    else
    {
        for (auto& pt : dynamicPatchEdges)
        {
            pt += position;
        }
        patch = generatePolyhedron(dynamicPatchEdges);
    }
    return patch;
}

bool TraversabilityGenerator3d::checkCollisionForYaw(TravGenNode* node, double yaw)
{
    /** Check if the robot, rotated to a specific yaw angle, would collide
     *  with MLS patches at the given node position.
     *  @return true if the yaw is collision-free (safe).
     */

    Eigen::Vector3d nodePos;
    if (!trMap.fromGrid(node->getIndex(), nodePos)) {
        return false;
    }
    nodePos.z() += node->getHeight();

    const double hx = config.robotSizeX / 2.0;
    const double hy = config.robotSizeY / 2.0;

    // Build yaw rotation
    Eigen::AngleAxisd yawRotation(yaw, Eigen::Vector3d::UnitZ());

    // Compute 4 lower corners of robot OBB, rotated by yaw
    std::vector<Eigen::Vector3d> robotCorners;
    std::vector<Eigen::Vector3d> localCorners = {
        { hx,  hy, 0.0},
        { hx, -hy, 0.0},
        {-hx, -hy, 0.0},
        {-hx,  hy, 0.0},
    };

    for (const auto& lc : localCorners)
    {
        Eigen::Vector3d rotated = yawRotation * lc;
        // Sample terrain height at the rotated corner position
        double terrainH = sampleTerrainHeightAtCorner(nodePos, rotated.x(), rotated.y());
        // If the corner falls off the patches, ignore this yaw
        const double robotDiagHalf = std::sqrt(hx * hx + hy * hy);
        const double vertSearchRange = robotDiagHalf * std::sin(config.maxSlope) + config.maxStepHeight;
        if (terrainH <= nodePos.z() - vertSearchRange + 1e-4)
        {
            return false; // Not on patch, reject this yaw
        }

        robotCorners.push_back({nodePos.x() + rotated.x(),
                                nodePos.y() + rotated.y(),
                                terrainH + config.maxStepHeight});
    }

    // Compute contact normal from the corners
    Eigen::Vector3d contactNormal = computeContactPlaneFromCorners(robotCorners);
    Eigen::Vector3d heightOffset = contactNormal * config.robotHeight;

    // Add upper 4 corners
    std::vector<Eigen::Vector3d> robotEdges8;
    for (const auto& c : robotCorners)
        robotEdges8.push_back(c);
    for (const auto& c : robotCorners)
        robotEdges8.push_back(c + heightOffset);

    Polyhedron_3 robot = generatePolyhedron(robotEdges8);

    // Search area: use half-diagonal as search radius (covers all rotations)
    const double halfDiag = std::sqrt(hx * hx + hy * hy);
    Eigen::Vector3d searchMin(-halfDiag, -halfDiag, 0);
    Eigen::Vector3d searchMax( halfDiag,  halfDiag, config.robotHeight);
    searchMin += nodePos;
    searchMax += nodePos;

    const Eigen::AlignedBox3d limitBox(searchMin, searchMax);
    View area = mlsGrid->intersectCuboid(limitBox);

    Index minIdx, maxIdx;
    if (!mlsGrid->toGrid(limitBox.min(), minIdx) || !mlsGrid->toGrid(limitBox.max(), maxIdx))
        return false;

    if (area.getNumCells().y() <= 0 || area.getNumCells().x() <= 0)
        return true;

    Index curIndex = minIdx;
    for (size_t y = 0; y < area.getNumCells().y() - 1; y++, curIndex.y() += 1)
    {
        curIndex.x() = minIdx.x();
        for (size_t x = 0; x < area.getNumCells().x() - 1; x++, curIndex.x() += 1)
        {
            Eigen::Vector3d pos;
            if (!mlsGrid->fromGrid(curIndex, pos))
                continue;

            for (const SurfacePatch<MLSConfig::SLOPE>* p : area.at(x, y))
            {
                pos.z() = (p->getTop() + p->getBottom()) / 2.0;

                Polyhedron_3 patch = createPolyhedronFromSurfacePatch(p, pos);
                if (CGAL::Polygon_mesh_processing::do_intersect(patch, robot))
                {
                    return false; // collision found — this yaw is not safe
                }
            }
        }
    }

    return true; // no collision — yaw is safe
}

void TraversabilityGenerator3d::inflateFrontiers()
{
    const double growRadiusSquared = std::pow(std::sqrt(config.robotSizeX * config.robotSizeX + config.robotSizeY * config.robotSizeY) / 2.0, 2);

    for(TravGenNode *n : frontierNodesGrowList)
    {
        Eigen::Vector3d nodePos = n ->getPosition(trMap);

        n->eachConnectedNode([&](maps::grid::TraversabilityNodeBase *neighbor, bool &expandNode, bool &stop)
        {

            TravGenNode* node = static_cast<TravGenNode*>(neighbor);

            if((neighbor->getPosition(trMap) - nodePos).squaredNorm() > growRadiusSquared)
            {
                //node out of radius, return
                return;
            }

            expandNode = true;

            if (neighbor->getType() == TraversabilityNodeBase::TRAVERSABLE)
            {
                neighbor->setType(n->getType());
                node->getUserData().nodeType = NodeType::INFLATED_FRONTIER;
            }
        });
    }

    frontierNodesGrowList.clear();
}

void TraversabilityGenerator3d::setConfig(const TraversabilityConfig &config)
{
    this->config = config;
    trMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));
    soilMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));

    if (mlsGrid)
    {
        Eigen::Vector2d newSize = mlsGrid->getSize().array() / trMap.getResolution().array();
        trMap.extend(Vector2ui(newSize.x(), newSize.y()));
        trMap.getLocalFrame() = mlsGrid->getLocalFrame();

        soilMap.extend(Vector2ui(newSize.x(), newSize.y()));
        soilMap.getLocalFrame() = mlsGrid->getLocalFrame();
    }

    double robotHalfLength = config.robotSizeX / 2.0;
    double robotHalfWidth = config.robotSizeY / 2.0;
    double robotHalfHeight = config.robotHeight / 2.0;

    robotEdges = {
        {robotHalfLength, robotHalfWidth, robotHalfHeight},    // Top-right-front
        {robotHalfLength, robotHalfWidth, -robotHalfHeight},   // Top-right-back
        {robotHalfLength, -robotHalfWidth, robotHalfHeight},   // Bottom-right-front
        {robotHalfLength, -robotHalfWidth, -robotHalfHeight},  // Bottom-right-back
        {-robotHalfLength, robotHalfWidth, robotHalfHeight},   // Top-left-front
        {-robotHalfLength, robotHalfWidth, -robotHalfHeight},  // Top-left-back
        {-robotHalfLength, -robotHalfWidth, robotHalfHeight},  // Bottom-left-front
        {-robotHalfLength, -robotHalfWidth, -robotHalfHeight}  // Bottom-left-back
    };

    robotPolyhedron = generatePolyhedron(robotEdges);

    double patchHalfLength = config.gridResolution / 2.0;
    double patchHalfWidth = config.gridResolution / 2.0;
    double patchHalfHeight = patchHeight / 2.0;

    patchEdges = {
        {patchHalfLength, patchHalfWidth, patchHalfHeight},    // Top-right-front
        {patchHalfLength, patchHalfWidth, -patchHalfHeight},   // Top-right-back
        {patchHalfLength, -patchHalfWidth, patchHalfHeight},   // Bottom-right-front
        {patchHalfLength, -patchHalfWidth, -patchHalfHeight},  // Bottom-right-back
        {-patchHalfLength, patchHalfWidth, patchHalfHeight},   // Top-left-front
        {-patchHalfLength, patchHalfWidth, -patchHalfHeight},  // Top-left-back
        {-patchHalfLength, -patchHalfWidth, patchHalfHeight},  // Bottom-left-front
        {-patchHalfLength, -patchHalfWidth, -patchHalfHeight}  // Bottom-left-back
    };

    patchPolyhedron = generatePolyhedron(patchEdges);
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
    if(!startNode)
        return;

    std::deque<TravGenNode *> candidates;
    candidates.push_back(startNode);

    int cnd = 0;

    while(!candidates.empty())
    {
        TravGenNode *node = candidates.front();
        candidates.pop_front();

        //check if the node was evaluated before somehow
        if(node->isExpanded())
            continue;

        cnd++;

        if((cnd % 1000) == 0)
        {
            LOG_DEBUG_S << "TraversabilityGenerator3d: Expanded " << cnd << " traversability nodes.";
        }

        if(!expandNode(node))
        {

            continue;
        }

        for(auto *n : node->getConnections())
        {
            if(!n->isExpanded())
            {
                if(expandDist > 0)
                {
                    const double dist = (startNode->getPosition(trMap) - n->getPosition(trMap)).norm();
                    if(dist <= expandDist)
                        candidates.push_back(static_cast<TravGenNode *>(n));
                }
                else
                {
                    candidates.push_back(static_cast<TravGenNode *>(n));
                }
            }
        }
    }

    inflateFrontiers();
    inflateObstacles();

#ifdef ENABLE_DEBUG_DRAWINGS
    /*
    V3DD::CLEAR_DRAWING("partially_traversable_arrows");
    for (LevelList<TravGenNode*> &l : trMap)
    {
        for (TravGenNode *node : l)
        {
            if (node->getUserData().nodeType == NodeType::PARTIALLY_TRAVERSABLE)
            {
                Eigen::Vector3d nodePos;
                trMap.fromGrid(node->getIndex(), nodePos, node->getHeight());
                nodePos.z() += 0.05; // Slightly offset upward for visibility

                const auto& allowed = node->getUserData().allowedOrientations;
                for (const auto& segment : allowed)
                {
                    // Arrow points in the middle of the allowed yaw segment
                    double yaw = segment.startRad + segment.width / 2.0;
                    Eigen::Vector3d targetDir(std::cos(yaw), std::sin(yaw), 0.0);
                    targetDir.normalize();

                    double arrowLength = config.gridResolution * 1.1;
                    Eigen::Vector3d tipPos = nodePos + targetDir * (arrowLength / 2.0);
                    // DRAW_ARROW identity points in z-direction, so rotate z->targetDir
                    Eigen::Quaterniond arrowOrientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), targetDir);
                    Eigen::Vector3d arrowSize(0.6, 0.6, arrowLength);
                    Eigen::Vector4d arrowColor(1.0, 0.0, 0.0, 1.0); // Red

                    V3DD::DRAW_ARROW("partially_traversable_arrows", tipPos, arrowOrientation, arrowSize, arrowColor);
                }
            }
        }
    }
    */
#endif

    LOG_DEBUG_S << "TraversabilityGenerator3d: Expanded " << cnd << " traversability nodes.";
}

void TraversabilityGenerator3d::inflateObstacles()
{
    const double halfRobotSizeX = config.robotSizeX / 2.0;
    const double halfRobotSizeY = config.robotSizeY / 2.0;

    // The AABB check already keeps the robot centre at least min(halfX, halfY) away
    // from any obstacle (the tight-axis footprint boundary).  We only need to inflate
    // by the remaining gap to half_diagonal so that the robot can still rotate freely
    // at the edge of the traversable zone without its corners hitting the obstacle.
    // Using the full half_diagonal here double-counts the footprint clearance and
    // closes far too much traversable space, especially in narrow corridors.
    const double halfDiagonal = std::sqrt(halfRobotSizeX * halfRobotSizeX + halfRobotSizeY * halfRobotSizeY);

    // Geometric gap between the AABB boundary and the rotation-safe circle.
    const double inflGap = halfDiagonal - std::min(halfRobotSizeX, halfRobotSizeY)/2;

    // obstacleInflationMultiplier must be at least 1.0: the AABB/OBB collision check
    // now uses only min(sizeX, sizeY)/2 as its search radius, so inflateObstacles
    // must cover the remaining gap out to the larger half-dimension.  A multiplier
    // below 1.0 would leave that gap unguarded.  Users may increase above 1.0 for
    // extra safety margins.
    if (config.obstacleInflationMultiplier < 1.0)
    {
        LOG_WARN_S << "TraversabilityGenerator3d: obstacleInflationMultiplier is "
                   << config.obstacleInflationMultiplier
                   << ", which is below the enforced minimum of 1.0. "
                      "The AABB/OBB step-height check uses only min(robotSizeX, robotSizeY)/2 "
                      "as its search radius; inflateObstacles must cover the remaining gap, "
                      "so a multiplier < 1.0 would leave part of the robot footprint unguarded. "
                      "Clamping to 1.0.";
    }
    const double effectiveMultiplier = std::max(1.0, config.obstacleInflationMultiplier);

    // Minimum is gridResolution*1.1, not gridResolution exactly: the distance check
    // compares cell centres, so a bare gridResolution threshold can miss neighbours
    // whose computed centre-to-centre distance is at gridResolution + floating-point noise.

    // This is specially relevant for small robots with sizeY/2 close to gridResolution, where the gap is sub-grid and 
    // the inflation would otherwise fail to trigger at all.
    const double inflRadius = effectiveMultiplier *
        std::max(inflGap, config.gridResolution * 1.1) + 1e-5;

    // Track nodes already evaluated to avoid redundant collision checks
    // (a node can be reached from multiple obstacle sources)
    std::unordered_set<TravGenNode*> evaluatedNodes;

    for (TravGenNode *n : obstacleNodesGrowList)
    {
        // Check the obstacle node itself first to see if any orientation is safe
        if (n->getType() == TraversabilityNodeBase::OBSTACLE)
        {
            if (evaluatedNodes.insert(n).second)
            {
                std::vector<double> safeYaws;
                for (int i = 0; i < 4; ++i)
                {
                    double yaw = i * (M_PI / 4.0);
                    if (checkCollisionForYaw(n, yaw))
                    {
                        safeYaws.push_back(yaw);
                        safeYaws.push_back(yaw + M_PI);
                    }
                }

                if (!safeYaws.empty())
                {
                    n->setType(TraversabilityNodeBase::TRAVERSABLE);
                    n->getUserData().nodeType = NodeType::PARTIALLY_TRAVERSABLE;
                    n->getUserData().allowedOrientations.clear();
                    for (double yaw : safeYaws)
                    {
                        n->getUserData().allowedOrientations.emplace_back(
                            base::Angle::fromRad(yaw - M_PI / 8.0), M_PI / 4.0
                        );
                    }
                }
            }
        }

        const Index nIdx = n->getIndex();
        n->eachConnectedNode([&] (maps::grid::TraversabilityNodeBase *neighbor, bool &expandNode, bool &stop)
        {
            TravGenNode* node = static_cast<TravGenNode*>(neighbor);

            // Use 2D (XY) grid-cell distance for the inflation check.
            // RANSAC can produce obstacle nodes with slightly wrong heights
            // (e.g. below the floor plane when wall patches pull the fitted
            // plane downward), which makes the 3D distance to an adjacent
            // traversable cell exceed inflRadius even when they are only one
            // grid step apart.  Inflation radius is a horizontal clearance
            // concept, so comparing only the XY offset is correct.
            const Index diff = neighbor->getIndex() - nIdx;

            const double dist2D = diff.matrix().cast<double>().norm() * config.gridResolution;
            if (dist2D < inflRadius)
            {
                if(node->getUserData().nodeType == NodeType::TRAVERSABLE ||
                   node->getUserData().nodeType == NodeType::INFLATED_FRONTIER ||
                   node->getUserData().nodeType == NodeType::FRONTIER ||
                   node->getUserData().nodeType == NodeType::UNKNOWN)
                {
                    if (evaluatedNodes.insert(node).second)
                    {
                        // Test 4 base yaw orientations + their π-offset mirrors = 8 total
                        std::vector<double> safeYaws;
                        for (int i = 0; i < 4; ++i)
                        {
                            double yaw = i * (M_PI / 4.0);
                            if (checkCollisionForYaw(node, yaw))
                            {
                                safeYaws.push_back(yaw);
                                safeYaws.push_back(yaw + M_PI);
                            }
                        }

                        if (safeYaws.empty())
                        {
                            // No safe orientation found — fully blocked
                            neighbor->setType(TraversabilityNodeBase::OBSTACLE);
                            node->getUserData().nodeType = NodeType::INFLATED_OBSTACLE;
                        }
                        else
                        {
                            // Some orientations are safe — partially traversable
                            neighbor->setType(TraversabilityNodeBase::TRAVERSABLE);
                            node->getUserData().nodeType = NodeType::PARTIALLY_TRAVERSABLE;
                            node->getUserData().allowedOrientations.clear();
                            for (double yaw : safeYaws)
                            {
                                // Each safe yaw gets a ±22.5° (π/8) wedge
                                node->getUserData().allowedOrientations.emplace_back(
                                    base::Angle::fromRad(yaw - M_PI / 8.0), M_PI / 4.0
                                );
                            }
                        }
                    }
                }
                expandNode = true;
            }
            else
            {
                return;
            }
        }
        );
    }
    obstacleNodesGrowList.clear();
}

void TraversabilityGenerator3d::addInitialPatchToMLS()
{
    if(patchRadius == 0)
        return;


    LOG_INFO_S << "TraversabilityGenerator3d: Adding initial patches to the MLS within a radius = " << patchRadius;
    const Vector2d res = mlsGrid->getResolution();

//         const double sizeHalfX = config.robotSizeX / 2.0;
//         const double sizeHalfY = config.robotSizeY / 2.0;

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
            if (!mlsGrid->toGrid(posMLS, idx)) {
                LOG_ERROR_S << "TraversabilityGenerator3d: Cannot add initial patch — position "
                            << posMLS.transpose() << " is outside of MLS grid.";
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

    Vector2d newSize = grid->getSize().array() / trMap.getResolution().array();
    trMap.extend(Vector2ui(newSize.x(), newSize.y()));
    trMap.getLocalFrame() = mlsGrid->getLocalFrame();
    
    soilMap.extend(Vector2ui(newSize.x(), newSize.y()));
    soilMap.getLocalFrame() = mlsGrid->getLocalFrame();   
    
    clearTrMap();
    clearSoilMap();
}

void TraversabilityGenerator3d::clearTrMap()
{
    for(LevelList<TravGenNode *> &l : trMap)
    {
        for(TravGenNode *n : l)
        {
            delete n;
        }

        l.clear();
    }
}

void TraversabilityGenerator3d::clearSoilMap()
{
    for(LevelList<SoilNode *> &l : soilMap)
    {
        for(SoilNode *n : l)
        {
            delete n;
        }

        l.clear();
    }
}

TravGenNode* TraversabilityGenerator3d::generateStartNode(const Eigen::Vector3d& startPos)
{
    Index idx;
    if (!trMap.toGrid(startPos, idx)) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Start position " 
                    << startPos.transpose()
                    << " is outside of the traversability map.";
        return nullptr;
    }

    TravGenNode* startNode = findMatchingTraversabilityPatchAt(idx, startPos.z());
    if (startNode) {
        LOG_DEBUG_S << "TraversabilityGenerator3d: Reusing existing node at index " 
                    << idx << " with height " << startNode->getHeight();
        return startNode;
    }

    startNode = createTraversabilityPatchAt(idx, startPos.z());
    if (!startNode) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Failed to create traversability node "
                    << "for start position " << startPos.transpose()
                    << " — no matching or insufficient MLS patches.";
        return startNode;
    }

    if (startNode->isExpanded() && startNode->getType() != TraversabilityNodeBase::TRAVERSABLE) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Start position " << startPos.transpose()
                    << " is on a non-traversable patch (type=" << startNode->getType() << ")";
    }

    return startNode;
}

SoilNode* TraversabilityGenerator3d::generateStartSoilNode(const Eigen::Vector3d& startPos)
{
    Index idx;
    if (!soilMap.toGrid(startPos, idx)) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Start position "
                    << startPos.transpose()
                    << " is outside of soil map.";
        return nullptr;
    }

    SoilNode* startNode = findMatchingSoilPatchAt(idx, startPos.z());
    if (startNode) {
        LOG_DEBUG_S << "TraversabilityGenerator3d: Reusing existing SoilNode at index "
                << idx << " (height=" << startNode->getHeight() << ")";
        return startNode;
    }

    startNode = createSoilPatchAt(idx, startPos.z());
    if (!startNode) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Failed to create soil node at index " 
                    << idx << " for start position " << startPos.transpose()
                    << " — no matching or insufficient MLS patches.";
        return nullptr;
    }
    return startNode;
}

bool TraversabilityGenerator3d::expandNode(TravGenNode * node)
{
    Eigen::Vector3d nodePos = node->getPosition(trMap);
    SoilNode *soilNode = generateStartSoilNode(nodePos);
    
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
        node->setType(TraversabilityNodeBase::OBSTACLE);
        node->getUserData().nodeType = NodeType::OBSTACLE;
        obstacleNodesGrowList.push_back(node);
        return false;
    }
    
    if(!checkStepHeightAABB(node))
    {
        if(!checkStepHeightOBB(node))
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            node->getUserData().nodeType = NodeType::OBSTACLE;
            obstacleNodesGrowList.push_back(node);
            return false;
        }
    } 
    
    if(config.enableInclineLimitting)
    {
        if(!computeAllowedOrientations(node))
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            node->getUserData().nodeType = NodeType::OBSTACLE;
            obstacleNodesGrowList.push_back(node);
            return false;
        }
    }

    //add surrounding
    addConnectedPatches(node);

    if(checkForFrontier(node))
    {
        node->setType(TraversabilityNodeBase::FRONTIER);
        node->getUserData().nodeType = NodeType::FRONTIER;
        frontierNodesGrowList.push_back(node);
        return false;
    }

    node->setType(TraversabilityNodeBase::TRAVERSABLE);
    node->getUserData().nodeType = NodeType::TRAVERSABLE;

    return true;
}

bool TraversabilityGenerator3d::isNodeFreeOfObstacles(const traversability_generator3d::TravGenNode* node) const
{
    //check if there is an mls patch above the ground
    Eigen::Vector3d nodePos;
    if (!trMap.fromGrid(node->getIndex(), nodePos, node->getHeight())) {
        throw std::runtime_error(
            "TraversabilityGenerator3d: Node index ("+ std::to_string(node->getIndex().x()) + ", " + std::to_string(node->getIndex().y()) + ")"
            + " with height " + std::to_string(node->getHeight())
            + " is outside of the traversability grid."
        );
    }

    Eigen::Vector3d min(-config.gridResolution/2.0 + 1e-5, -config.gridResolution / 2.0 + 1e-5, config.maxStepHeight);
    Eigen::Vector3d max(config.gridResolution/2.0 - 1e-5, config.gridResolution/2.0 - 1e-5, config.robotHeight);
    
    
    min += nodePos;
    max += nodePos;
    
    const Eigen::AlignedBox3d boundingBox(min, max);
    
    size_t numIntersections = 0;
    const View area = mlsGrid->intersectCuboid(boundingBox, numIntersections);
    if(numIntersections > 0)
        return false;
    
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
        {
            break;
        }
    }

    //Also add the interpolated height, to fill in small holes
    //if there is no support, the ransac will filter the node out
    candidates.push_back(curHeight);

    ret = new TravGenNode(0.0, idx);
    ret->getUserData().id = currentNodeId++;
    ret->getUserData().cost = 0;

    for(double height: candidates)
    {
        ret->setHeight(height);
        ret->setNotExpanded();
        ret->setType(TraversabilityNodeBase::UNSET);
        ret->getUserData().nodeType = NodeType::UNSET;

        //there is a neighboring patch in the mls that has a reachable hight
        if(!computePlaneRansac(*ret))
        {
            if(mlsIdx.x() == 1 || mlsIdx.y() == 1) {
                ret->setType(TraversabilityNodeBase::OBSTACLE);
                ret->getUserData().nodeType = NodeType::OBSTACLE;
            } else {
                ret->setType(TraversabilityNodeBase::UNKNOWN);
                ret->getUserData().nodeType = NodeType::UNKNOWN;
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
            //the patch is set to OBSTACLE if the for loop finishes without finding
            //a patch within the maxStepHeight margin of curHeight.
        }
    }

    ret->setHeight(curHeight);
    ret->setNotExpanded();
    ret->setType(TraversabilityNodeBase::OBSTACLE);
    ret->getUserData().nodeType = NodeType::OBSTACLE;
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
        const Eigen::Vector3d newPos = line.intersectionPoint(node->getUserData().plane);

        // If XY differs more than tolerance, something is off
        constexpr double kXYTolerance = 1e-3;
        const Eigen::Vector2d delta = patchPosPlane.head<2>() - newPos.head<2>();
        const double deltaNorm = delta.norm();

        if (deltaNorm > kXYTolerance) {
            LOG_ERROR_S << "TraversabilityGenerator3d: Adjustment height check failed — "
                        << "|Δxy|=" << deltaNorm << " > tol=" << kXYTolerance
                        << ", patchXY=(" << patchPosPlane.x() << ", " << patchPosPlane.y() << ")"
                        << ", newXY=(" << newPos.x() << ", " << newPos.y() << ")";
            return;
        }
        const double localHeight = newPos.z();
        //The new patch is not reachable from the current patch
        if(fabs(localHeight - curHeight) > config.maxStepHeight)
        {
//#ifdef ENABLE_DEBUG_DRAWINGS
//            V3DD::COMPLEX_DRAWING([&]()
//            {
//                maps::grid::Vector3d pos;
//                trMap.fromGrid(node->getIndex(), pos, node->getHeight(), false);
//                V3DD::DRAW_SPHERE("traversability_generator3d_expandFailStepHeight", pos, 0.05, V3DD::Color::carrot_orange);
//            });
//#endif
            continue;
        }


        TravGenNode *toAdd = nullptr;

        if (!newPos.allFinite()) {
            LOG_ERROR_S << "TraversabilityGenerator3d: newPos contains non-finite values: "
                        << newPos.transpose();
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
void TraversabilityGenerator3d::addConnectedPatches(SoilNode *  node)
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

        if(!soilMap.inGrid(idx))
        {
            continue;
        }

        SoilNode *toAdd = nullptr;
        toAdd = findMatchingSoilPatchAt(idx, curHeight);

        if(!toAdd)
        {
            toAdd = createSoilPatchAt(idx, curHeight);
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

SoilNode *TraversabilityGenerator3d::createSoilPatchAt(maps::grid::Index idx, const double curHeight)
{
    SoilNode *ret = nullptr;

    ret = new SoilNode(curHeight, idx);
    ret->setHeight(curHeight);
    ret->setNotExpanded();
    ret->setType(TraversabilityNodeBase::UNSET);
    ret->getUserData().soilType = SoilType::UNKNOWN_SOIL;
    soilMap.at(idx).insert(ret);
    return ret;
}

void TraversabilityGenerator3d::updateSoilInformation(){
    for(LevelList<TravGenNode *> &l : trMap)
    {
        for(TravGenNode *node : l)
        {
            Eigen::Vector3d nodePos = node->getPosition(trMap);
            Index idx;
            if (!soilMap.toGrid(nodePos, idx)) {
                LOG_ERROR_S << "TraversabilityGenerator3d: updateSoilInformation:  Node position "
                            << nodePos.transpose()
                            << " is outside of soil map.";
                continue;
            }

            SoilNode* soilNode = findMatchingSoilPatchAt(idx, nodePos.z());
            if (!soilNode){
                continue;
            }

            switch(soilNode->getUserData().soilType){
                case SoilType::SAND:
                    node->getUserData().cost = 500 * (1.0 - soilNode->getUserData().probSand);
                    if (!config.traverseSand){
                        node->setType(TraversabilityNodeBase::OBSTACLE);
                        node->getUserData().nodeType = NodeType::OBSTACLE;
                        obstacleNodesGrowList.push_back(node);
                    }
                    break;
                case SoilType::CONCRETE:
                    node->getUserData().cost = 500 * (1.0 - soilNode->getUserData().probConcrete);
                    if (!config.traverseConcrete){
                        node->setType(TraversabilityNodeBase::OBSTACLE);
                        node->getUserData().nodeType = NodeType::OBSTACLE;
                        obstacleNodesGrowList.push_back(node);
                    }            
                    break;
                case SoilType::GRAVEL:
                    node->getUserData().cost = 500 * (1.0 - soilNode->getUserData().probGravel);
                    if (!config.traverseGravel){
                        node->setType(TraversabilityNodeBase::OBSTACLE);
                        node->getUserData().nodeType = NodeType::OBSTACLE;
                        obstacleNodesGrowList.push_back(node);
                    }
                    break;
                case SoilType::ROCKS:
                    node->getUserData().cost = 500 * (1.0 - soilNode->getUserData().probRocks);
                    if (!config.traverseRocks){
                        node->setType(TraversabilityNodeBase::OBSTACLE);
                        node->getUserData().nodeType = NodeType::OBSTACLE;
                        obstacleNodesGrowList.push_back(node);
                    }
                    break;
                case SoilType::UNKNOWN_SOIL:
                    node->getUserData().cost = 2000;
                    break;
                default:
                    break;
            } 
        }
    }    
    inflateObstacles();
}

SoilNode* TraversabilityGenerator3d::findMatchingSoilPatchAt(Index idx, const double curHeight) const
{
    auto &trList(soilMap.at(idx));

    //check if we got an existing node
    for(SoilNode *snode : trList)
    {
        const double searchHeight = snode->getHeight();
        if(std::abs(searchHeight-curHeight) <= config.maxStepHeight)
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

void TraversabilityGenerator3d::setSoilType(SoilNode * node, SoilType type){
    node->getUserData().soilType = type;
}


double TraversabilityGenerator3d::gaussian2D(double x, double y, 
                  double meanX, double meanY, 
                  double sigmaX, double sigmaY) {

    // Exponent part of the Gaussian
    double term1 = std::pow((x - meanX) / sigmaX, 2);
    double term2 = std::pow((y - meanY) / sigmaY, 2);
    
    return std::exp(-0.5 * (term1 + term2));
}


bool TraversabilityGenerator3d::addSoilNode(const SoilSample& sample){
    SoilNode* sampleNode = generateStartSoilNode(sample.location);
    if (!sampleNode) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Failed to add soil patch at location "
                    << sample.location.transpose() << " to soilMap.";
        return false;
    }

    Eigen::Vector3d samplePos;
    if (!soilMap.fromGrid(sampleNode->getIndex(), samplePos, sampleNode->getHeight())) {
        LOG_ERROR_S << "TraversabilityGenerator3d: Soil node index "
                    << sampleNode->getIndex()
                    << " with height " << sampleNode->getHeight()
                    << " is outside of soil map grid.";
        return false;
    }

    addConnectedPatches(sampleNode);

    std::unordered_set<SoilNode*> visitedNodes;
    visitedNodes.insert(sampleNode);

    std::deque<SoilNode *> candidates;
    candidates.push_back(sampleNode);

    while(!candidates.empty())
    {
        SoilNode *currentNode = candidates.front();
        candidates.pop_front();   

        Eigen::Vector3d nodePos;
        if (!soilMap.fromGrid(currentNode->getIndex(), nodePos, currentNode->getHeight())) {
            LOG_ERROR_S << "TraversabilityGenerator3d: Soil node index "
                        << currentNode->getIndex()
                        << " with height " << currentNode->getHeight()
                        << " is outside the soil map grid.";
            return false;
        }

        double likelihood = gaussian2D(nodePos.x(), nodePos.y(), 
                                       samplePos.x(), samplePos.y(), 
                                       sample.sigmaX, sample.sigmaY);

        double likelihoodSand = likelihood;
        double likelihoodConcrete = likelihood;
        double likelihoodGravel = likelihood;
        double likelihoodRocks = likelihood;

        // Lower uncertainty means stronger adjustment
        double reductionFactor = 0.1 + (0.8 * sample.uncertainty);
        
        switch (sample.type) {
            case SoilType::SAND:
                likelihoodConcrete *= reductionFactor;
                likelihoodGravel *= reductionFactor;
                likelihoodRocks *= reductionFactor;
                break;
            case SoilType::CONCRETE:
                likelihoodSand *= reductionFactor;
                likelihoodGravel *= reductionFactor;
                likelihoodRocks *= reductionFactor;
                break;
            case SoilType::GRAVEL:
                likelihoodSand *= reductionFactor;
                likelihoodConcrete *= reductionFactor;
                likelihoodRocks *= reductionFactor;
                break;
            case SoilType::ROCKS:
                likelihoodSand *= reductionFactor;
                likelihoodConcrete *= reductionFactor;
                likelihoodGravel *= reductionFactor;
                break;
            default:
                break;
        }      
        currentNode->getUserData().updateProbabilities(likelihoodSand, 
                                                       likelihoodConcrete,
                                                       likelihoodGravel,
                                                       likelihoodRocks);

        for (auto *neighbor : currentNode->getConnections()) {

            Eigen::Vector3d neighborPos;
            if (!soilMap.fromGrid(neighbor->getIndex(), neighborPos, neighbor->getHeight())) {
                LOG_ERROR_S << "TraversabilityGenerator3d: Neighbor node index "
                            << neighbor->getIndex()
                            << " with height " << neighbor->getHeight()
                            << " is outside the soil map grid.";
                return false;
            }

            double likelihood = gaussian2D(neighborPos.x(), neighborPos.y(), 
                                           samplePos.x(), samplePos.y(), 
                                           sample.sigmaX, sample.sigmaY);

            if (likelihood < 0.05){
                continue;
            }

            SoilNode* n = static_cast<SoilNode*>(neighbor);
            if (visitedNodes.insert(n).second) {
                addConnectedPatches(n);
                candidates.push_back(n);
            }
        }
    }
    return true;
}
}
