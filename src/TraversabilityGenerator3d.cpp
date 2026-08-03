#include "TraversabilityGenerator3d.hpp"
#include <numeric/PlaneFitting.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>

#include <chrono>
#include <deque>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <unordered_set>

#ifdef _OPENMP
#include <omp.h>
#include <sched.h>
#endif

using namespace maps::grid;

namespace traversability_generator3d
{

namespace
{
/** Largest distance from the robot ORIGIN to any corner of the (possibly
 *  offset) footprint box; the radius that covers the body at every yaw. */
double footprintMaxReach(const TraversabilityConfig& config)
{
    const double reachX = config.robotSizeX / 2.0 + std::abs(config.footprintOffsetX);
    const double halfY = config.robotSizeY / 2.0;
    return std::sqrt(reachX * reachX + halfY * halfY);
}

/** Half-side of the largest origin-centered square guaranteed inside the
 *  (possibly offset) footprint box at every yaw. With an offset the origin
 *  moves toward one box edge, shrinking the yaw-independent core. */
double footprintMinHalfExtent(const TraversabilityConfig& config)
{
    return std::max(0.0, std::min(config.robotSizeX / 2.0 - std::abs(config.footprintOffsetX),
                                  config.robotSizeY / 2.0));
}

/** A robot origin outside the footprint box (|offset| >= sizeX/2) collapses
 *  the yaw-independent core to zero: the cheap AABB obstacle pre-check then
 *  passes everything and obstacle detection rests solely on the per-yaw
 *  checks near inflation seeds. No physical robot is configured like that,
 *  so treat it as a configuration error. */
void warnOnDegenerateFootprint(const TraversabilityConfig& config)
{
    if (std::abs(config.footprintOffsetX) >= config.robotSizeX / 2.0)
    {
        LOG_WARN_S << "TraversabilityGenerator3d: footprintOffsetX ("
                   << config.footprintOffsetX << ") places the robot origin outside the "
                   << config.robotSizeX << " m footprint box; yaw-independent obstacle "
                      "checks degenerate. Check robotSizeX/footprintOffsetX.";
    }
}

/** 8-neighborhood expansion offsets, shared by addConnectedPatches() and the
 *  wave-parallel expandAll(). */
const std::vector<Index> kNeighborOffsets = {
    Index(1, 1),
    Index(1, 0),
    Index(1, -1),
    Index(0, 1),
    Index(0, -1),
    Index(-1, 1),
    Index(-1, 0),
    Index(-1, -1)};

/** Key for per-cell maps during one expansion wave. */
uint64_t cellKey(const Index& idx)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(idx.y())) << 32) |
            static_cast<uint32_t>(idx.x());
}

/** Below this many items a parallel region is not launched (OpenMP if-clause):
 *  the team launch costs more than the work, and in desktop/GUI processes
 *  (llvmpipe render threads competing for cores) each launch can cost
 *  milliseconds — hundreds of tiny BFS waves then dominate the runtime. */
constexpr std::int64_t kMinParallelItems = 32;
}

TraversabilityGenerator3d::TraversabilityGenerator3d(const TraversabilityConfig& config)
    : addInitialPatch(false), config(config), patchHeight(0.02) // Set default patchHeight
{
    warnOnDegenerateFootprint(config);
    trMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));
    soilMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));

    const double offX = config.footprintOffsetX;
    double robotHalfLength = config.robotSizeX / 2.0;
    double robotHalfWidth = config.robotSizeY / 2.0;
    double robotHalfHeight = config.robotHeight / 2.0;

    robotEdges = {
        {offX + robotHalfLength, robotHalfWidth, robotHalfHeight},    // Top-right-front
        {offX + robotHalfLength, robotHalfWidth, -robotHalfHeight},   // Top-right-back
        {offX + robotHalfLength, -robotHalfWidth, robotHalfHeight},   // Bottom-right-front
        {offX + robotHalfLength, -robotHalfWidth, -robotHalfHeight},  // Bottom-right-back
        {offX - robotHalfLength, robotHalfWidth, robotHalfHeight},    // Top-left-front
        {offX - robotHalfLength, robotHalfWidth, -robotHalfHeight},   // Top-left-back
        {offX - robotHalfLength, -robotHalfWidth, robotHalfHeight},   // Bottom-left-front
        {offX - robotHalfLength, -robotHalfWidth, -robotHalfHeight}   // Bottom-left-back
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

    // Only forward finite coordinates to CGAL. CGAL's exact-arithmetic filter
    // converts every double to a GMP rational via __gmpq_set_d(), which raises
    // SIGFPE on NaN/Inf. Degenerate/near-vertical MLS patches can produce such
    // coordinates, so we defensively drop them here.
    for (const Eigen::Vector3d& v : points)
    {
        if (v.allFinite())
            cgal_p3.emplace_back(v.x(), v.y(), v.z());
    }

    Polyhedron_3 polyhedron;
    // convex_hull_3 needs at least 4 non-coplanar points to build a volume.
    if (cgal_p3.size() < 4)
        return polyhedron;

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


    // number of cells in the search area (density denominator). We count cells that
    // contribute at least one ground patch, so cells with only walls/overhead don't inflate it.
    const int patchCntTotal = area.getNumCells().y() * area.getNumCells().x();
    // Reject patches steeper than maxSlope (walls, curbs): they are not ground and would
    // otherwise pull the fitted plane. Mirrors the idiom in sampleTerrainHeightAtCorner().
    // NOTE: we deliberately do NOT gate patches by height around the node's *tentative*
    // height here -- RANSAC is what determines the plane/height, and pre-filtering to a
    // narrow band breaks hole-filling and slope fits (leaves UNKNOWN holes). The search
    // cuboid already bounds z to +-maxStepHeight, and RANSAC's inlier threshold rejects
    // the remaining outliers.
    const double minVerticalNormalZ = std::cos(config.maxSlope);
    int patchCnt = 0;
    for(size_t y = 0; y < area.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < area.getNumCells().x(); x++)
        {
            Eigen::Vector2d pos = Eigen::Vector2d(x,y).cwiseProduct(res) - sizeHalf;

            bool hasGroundPatch = false;
            for(const MLGrid::PatchType *p : area.at(x, y))
            {
                // Skip near-vertical patches (walls) so they don't contaminate the ground fit.
                Eigen::Vector3f normalf = p->getNormal();
                Eigen::Vector3d pnormal(normalf.x(), normalf.y(), normalf.z());
                if(!pnormal.allFinite())
                    continue;
                pnormal.normalize();
                if(std::abs(pnormal.z()) < minVerticalNormalZ)
                    continue;

                const double h = (p->getTop() + p->getBottom()) / 2.0;
                points->push_back(PointT(pos.x(), pos.y(), h));
                hasGroundPatch = true;
            }

            if(hasGroundPatch)
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

    // Orient the normal upward so slope = acos(normal . z) stays in [0, pi/2]. RANSAC returns
    // an arbitrary sign; a downward normal would otherwise yield a bogus > 90 deg slope.
    if (normal.z() < 0.0)
    {
        normal = -normal;
        distToOrigin = -distToOrigin;
    }

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
    const double robotDiagHalf = footprintMaxReach(config);
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
    const double offX = config.footprintOffsetX;
    const double hx = config.robotSizeX / 2.0;
    const double hy = config.robotSizeY / 2.0;
    return {
        {nodePos.x() + offX + hx, nodePos.y() + hy, sampleTerrainHeightAtCorner(nodePos, offX + hx,  hy) + config.maxStepHeight},
        {nodePos.x() + offX + hx, nodePos.y() - hy, sampleTerrainHeightAtCorner(nodePos, offX + hx, -hy) + config.maxStepHeight},
        {nodePos.x() + offX - hx, nodePos.y() - hy, sampleTerrainHeightAtCorner(nodePos, offX - hx, -hy) + config.maxStepHeight},
        {nodePos.x() + offX - hx, nodePos.y() + hy, sampleTerrainHeightAtCorner(nodePos, offX - hx,  hy) + config.maxStepHeight},
    };
}

std::vector<Eigen::Vector3d> TraversabilityGenerator3d::computeRigidRobotCorners(
    const std::vector<Eigen::Vector3d>& cornerPositions, 
    const Eigen::Vector3d& contactNormal, 
    double yaw, 
    const Eigen::Vector3d& nodePos)
{
    const double offX = config.footprintOffsetX;
    const double hx = config.robotSizeX / 2.0;
    const double hy = config.robotSizeY / 2.0;

    Eigen::Vector3d up(0, 0, 1);
    Eigen::Quaterniond slopeRotation = Eigen::Quaterniond::FromTwoVectors(up, contactNormal);
    Eigen::AngleAxisd yawRotation(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (slopeRotation * yawRotation).toRotationMatrix();

    std::vector<Eigen::Vector3d> localCorners = {
        {offX + hx,  hy, 0.0},
        {offX + hx, -hy, 0.0},
        {offX - hx, -hy, 0.0},
        {offX - hx,  hy, 0.0}
    };

    std::vector<Eigen::Vector3d> rotatedCorners;
    rotatedCorners.reserve(4);
    for(const auto& lc : localCorners)
    {
        rotatedCorners.push_back(R * lc);
    }

    // Find the height z_center of the flat bottom plane's center
    double z_center = -1e9;
    for(size_t i = 0; i < 4; ++i)
    {
        double reqZ = cornerPositions[i].z() - rotatedCorners[i].z();
        if(reqZ > z_center)
            z_center = reqZ;
    }

    std::vector<Eigen::Vector3d> flatBottomCorners;
    flatBottomCorners.reserve(4);
    for(const auto& rc : rotatedCorners)
    {
        flatBottomCorners.push_back(Eigen::Vector3d(nodePos.x(), nodePos.y(), z_center) + rc);
    }

    return flatBottomCorners;
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

    // Use the yaw-independent footprint core as a uniform search radius on both axes.
    // This keeps the AABB square and avoids a large dead-zone near map boundaries
    // when robotSizeX >> robotSizeY (or vice-versa). With a footprint offset the
    // origin-centered core shrinks accordingly. Cells closer than this to any
    // obstacle are already obstacle-free by construction; the remaining gap to the
    // rotation-safe circle is covered by inflateObstacles.
    const double halfSmall = footprintMinHalfExtent(config);
    Eigen::Vector3d min(-halfSmall, -halfSmall, config.maxStepHeight);
    Eigen::Vector3d max( halfSmall,  halfSmall, config.maxStepHeight + config.robotHeight);

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

    //Iterate ALL cells of the view: the old "-1" bounds (justified by intersectCuboid's
    //inclusive max) silently skipped the last row/column of patches inside the footprint.
    //For a collision check the conservative direction is inclusion -- an extra boundary
    //cell only costs one more test.
    for(size_t y = 0; y < area.getNumCells().y(); y++, curIndex.y() += 1)
    {
        curIndex.x() = minIdx.x();
        for(size_t x = 0; x < area.getNumCells().x(); x++, curIndex.x() += 1)
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

    // Reference surface: the node's fitted ground plane (see checkCollisionForYaw for the
    // rationale -- sampling "terrain" under the corners could hoist the modelled body onto
    // the very obstacle it should collide with).
    Eigen::Vector3d planeNormal = node->getUserData().plane.normal();
    if (!planeNormal.allFinite() || planeNormal.norm() < 1e-6 || std::abs(planeNormal.z()) < 1e-6)
        planeNormal = Eigen::Vector3d::UnitZ();
    else
    {
        planeNormal.normalize();
        if (planeNormal.z() < 0.0)
            planeNormal = -planeNormal;
    }
    //ground plane z at an xy offset from the node centre (the plane passes through nodePos)
    auto planeZAt = [&](double dx, double dy)
    {
        return nodePos.z() - (planeNormal.x() * dx + planeNormal.y() * dy) / planeNormal.z();
    };

    const double coffX = config.footprintOffsetX;
    const double chx = config.robotSizeX / 2.0;
    const double chy = config.robotSizeY / 2.0;
    std::vector<Eigen::Vector3d> bottomCorners = {
        {nodePos.x() + coffX + chx, nodePos.y() + chy, planeZAt(coffX + chx,  chy) + config.maxStepHeight},
        {nodePos.x() + coffX + chx, nodePos.y() - chy, planeZAt(coffX + chx, -chy) + config.maxStepHeight},
        {nodePos.x() + coffX - chx, nodePos.y() - chy, planeZAt(coffX - chx, -chy) + config.maxStepHeight},
        {nodePos.x() + coffX - chx, nodePos.y() + chy, planeZAt(coffX - chx,  chy) + config.maxStepHeight},
    };

    const Eigen::Vector3d heightOffset = planeNormal * config.robotHeight;

    // Lower 4 corners + Upper 4 corners (at lower + height offset)
    std::vector<Eigen::Vector3d> robotEdges4Point;
    for(const auto& corner : bottomCorners)
    {
        robotEdges4Point.push_back(corner);
    }
    for(size_t i = 0; i < 4; i++)
    {
        robotEdges4Point.push_back(bottomCorners[i] + heightOffset);
    }
    
    // Build polyhedron from these 8 corners
    Polyhedron_3 robot = generatePolyhedron(robotEdges4Point);

    // Query MLS patches across the entire unrotated robot footprint bounding box. The z
    // range is widened by the plane's possible drop/rise across the footprint so patches
    // under the downhill corners are not missed on slopes.
    const double halfX = config.robotSizeX / 2.0;
    const double halfY = config.robotSizeY / 2.0;
    const double zSlack = footprintMaxReach(config) * std::tan(config.maxSlope);
    Eigen::Vector3d min(coffX - halfX, -halfY, config.maxStepHeight - zSlack);
    Eigen::Vector3d max(coffX + halfX,  halfY, config.maxStepHeight + config.robotHeight + zSlack);

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

    //Iterate ALL cells of the view: the old "-1" bounds (justified by intersectCuboid's
    //inclusive max) silently skipped the last row/column of patches inside the footprint.
    //For a collision check the conservative direction is inclusion -- an extra boundary
    //cell only costs one more test.
    for(size_t y = 0; y < area.getNumCells().y(); y++, curIndex.y() += 1)
    {
        curIndex.x() = minIdx.x();
        for(size_t x = 0; x < area.getNumCells().x(); x++, curIndex.x() += 1)
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
                    // V3DD must NOT run from OpenMP workers: DRAW_* marshals to the
                    // Qt GUI thread with a BLOCKING invoke, and during expansion the
                    // GUI thread is parked on this parallel region's barrier — a
                    // guaranteed deadlock (observed live in the travgen GUI). Draw
                    // only when the check runs on a serial path.
#ifdef _OPENMP
                    if (!omp_in_parallel())
#endif
                    {
                        static int collisionCounter = 0;
                        collisionCounter++;
                        if (collisionCounter % 200 == 0)
                        {
                            std::string robotPrefix = "colliding_robot_" + std::to_string(collisionCounter);
                            Eigen::Vector4d red{1.0, 0.0, 0.0, 1.0};
                            
                            V3DD::COMPLEX_DRAWING([&]
                            {
                                for(size_t i = 0; i < 4; i++)
                                {
                                    Eigen::Vector3d lo = bottomCorners[i];
                                    Eigen::Vector3d hi = bottomCorners[i] + heightOffset;
                                    size_t next = (i + 1) % 4;
                                    Eigen::Vector3d loNext = bottomCorners[next];
                                    Eigen::Vector3d hiNext = bottomCorners[next] + heightOffset;
                                    
                                    V3DD::DRAW_LINE(robotPrefix + "_v" + std::to_string(i), lo, hi, red);
                                    V3DD::DRAW_LINE(robotPrefix + "_lo" + std::to_string(i), lo, loNext, red);
                                    V3DD::DRAW_LINE(robotPrefix + "_hi" + std::to_string(i), hi, hiNext, red);
                                }
                            });

                            // Draw colliding MLS patch sloped prism
                            std::vector<Eigen::Vector3f> polygonPoints;
                            Eigen::Vector2f cellCenter = pos.head<2>().cast<float>();
                            Eigen::Vector2f cellSize(config.gridResolution, config.gridResolution);
                            maps::grid::getPolygon(polygonPoints, *p, cellCenter, cellSize);

                            if (polygonPoints.size() >= 3)
                            {
                                std::string patchPrefix = "colliding_patch_" + std::to_string(collisionCounter);
                                Eigen::Vector4d yellow{1.0, 0.8, 0.0, 1.0};
                                Eigen::Vector3f normalf = p->getNormal();
                                if (normalf.z() < 0)
                                    normalf *= -1.0f;
                                Eigen::Vector3d normal{normalf.x(), normalf.y(), normalf.z()};
                                if (normal.norm() > 1e-6)
                                    normal.normalize();
                                else
                                    normal = Eigen::Vector3d::UnitZ();
                                const double thickness = 0.02;

                                V3DD::COMPLEX_DRAWING([&]
                                {
                                    for (size_t i = 0; i < polygonPoints.size(); i++)
                                    {
                                        Eigen::Vector3d hi = polygonPoints[i].cast<double>();
                                        Eigen::Vector3d lo = hi - thickness * normal;
                                        size_t next = (i + 1) % polygonPoints.size();
                                        Eigen::Vector3d hiNext = polygonPoints[next].cast<double>();
                                        Eigen::Vector3d loNext = hiNext - thickness * normal;

                                        V3DD::DRAW_LINE(patchPrefix + "_v" + std::to_string(i), lo, hi, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_lo" + std::to_string(i), lo, loNext, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_hi" + std::to_string(i), hi, hiNext, yellow);
                                    }
                                });
                            }
                            else
                            {
                                std::string patchPrefix = "colliding_patch_" + std::to_string(collisionCounter);
                                Eigen::Vector4d yellow{1.0, 0.8, 0.0, 1.0};
                                float minZ, maxZ;
                                p->getRange(minZ, maxZ);
                                double hres = config.gridResolution / 2.0;

                                std::vector<Eigen::Vector3d> corners = {
                                    {pos.x() + hres, pos.y() + hres, (double)minZ},
                                    {pos.x() + hres, pos.y() - hres, (double)minZ},
                                    {pos.x() - hres, pos.y() - hres, (double)minZ},
                                    {pos.x() - hres, pos.y() + hres, (double)minZ}
                                };
                                double height = maxZ - minZ;

                                V3DD::COMPLEX_DRAWING([&]
                                {
                                    for (size_t i = 0; i < 4; i++)
                                    {
                                        Eigen::Vector3d lo = corners[i];
                                        Eigen::Vector3d hi = corners[i] + Eigen::Vector3d(0, 0, height);
                                        size_t next = (i + 1) % 4;
                                        Eigen::Vector3d loNext = corners[next];
                                        Eigen::Vector3d hiNext = corners[next] + Eigen::Vector3d(0, 0, height);

                                        V3DD::DRAW_LINE(patchPrefix + "_v" + std::to_string(i), lo, hi, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_lo" + std::to_string(i), lo, loNext, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_hi" + std::to_string(i), hi, hiNext, yellow);
                                    }
                                });
                            }
                        }
                    }
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

    // The patch normal is only needed to detect degenerate patches below.
    Eigen::Vector3f normalf = p->getNormal();

    std::vector<Eigen::Vector3f> polygonPoints;
    Eigen::Vector2f cellCenter = position.head<2>().cast<float>();
    Eigen::Vector2f cellSize(config.gridResolution, config.gridResolution);

    maps::grid::getPolygon(polygonPoints, *p, cellCenter, cellSize);

    std::vector<Eigen::Vector3d> polyhedronPoints;
    const double thickness = 0.02;

    // getPolygon() reconstructs the patch outline from the plane equation
    // (z = (d - nx*x - ny*y) / nz). For near-vertical/degenerate patches nz -> 0,
    // yielding non-finite z. Reject such polygons and fall back to the box below,
    // otherwise the coordinates crash CGAL/GMP with SIGFPE.
    bool polygonFinite = normalf.allFinite();
    for (const auto& pt : polygonPoints)
    {
        if (!pt.allFinite())
        {
            polygonFinite = false;
            break;
        }
    }

    if (polygonPoints.size() >= 3 && polygonFinite)
    {
        // Cover the patch's REAL vertical extent. A tall obstacle patch (e.g. a fallen
        // tree) was previously modelled as a 2 cm plate at its fitted plane, so a robot
        // body polyhedron could pass above it without any intersection being detected.
        float minZ, maxZ;
        p->getRange(minZ, maxZ);
        double zTop = maxZ;
        double zBot = minZ;
        if (zTop - zBot < thickness)
        {
            const double pad = (thickness - (zTop - zBot)) / 2.0;
            zTop += pad;
            zBot -= pad;
        }
        polyhedronPoints.reserve(polygonPoints.size() * 2);
        for (const auto& pt : polygonPoints)
        {
            const Eigen::Vector3d ptd = pt.cast<double>();
            polyhedronPoints.push_back({ptd.x(), ptd.y(), std::max(ptd.z(), zTop)});
            polyhedronPoints.push_back({ptd.x(), ptd.y(), std::min(ptd.z(), zBot)});
        }
    }
    else
    {
        // Fallback: cell-aligned box if getPolygon yields less than 3 points
        float minZ, maxZ;
        p->getRange(minZ, maxZ);
        double hres = config.gridResolution / 2.0;

        std::vector<Eigen::Vector2d> corners2D = {
            {position.x() + hres, position.y() + hres},
            {position.x() + hres, position.y() - hres},
            {position.x() - hres, position.y() - hres},
            {position.x() - hres, position.y() + hres}
        };
        polyhedronPoints.reserve(8);
        for (const auto& c : corners2D)
        {
            polyhedronPoints.push_back({c.x(), c.y(), (double)maxZ});
            polyhedronPoints.push_back({c.x(), c.y(), (double)minZ});
        }
    }

    return generatePolyhedron(polyhedronPoints);
}

std::vector<TraversabilityGenerator3d::YawCheckPatch> TraversabilityGenerator3d::collectYawCheckPatches(TravGenNode* node)
{
    std::vector<YawCheckPatch> out;

    Eigen::Vector3d nodePos;
    if (!trMap.fromGrid(node->getIndex(), nodePos))
        return out;
    nodePos.z() += node->getHeight();

    // Same rotation-safe search window as the per-yaw checks (covers all yaws).
    const double halfDiag = footprintMaxReach(config);
    const double zSlack = halfDiag * std::tan(config.maxSlope);
    Eigen::Vector3d searchMin(-halfDiag, -halfDiag, config.maxStepHeight - zSlack);
    Eigen::Vector3d searchMax( halfDiag,  halfDiag, config.maxStepHeight + config.robotHeight + zSlack);
    searchMin += nodePos;
    searchMax += nodePos;
    const Eigen::AlignedBox3d limitBox(searchMin, searchMax);
    View area = mlsGrid->intersectCuboid(limitBox);

    Index minIdx, maxIdx;
    if (!mlsGrid->toGrid(limitBox.min(), minIdx) || !mlsGrid->toGrid(limitBox.max(), maxIdx))
        return out;  // boundary case: checkCollisionForYaw() re-checks and returns unsafe

    if (area.getNumCells().y() <= 0 || area.getNumCells().x() <= 0)
        return out;

    Index curIndex = minIdx;
    for (size_t y = 0; y < area.getNumCells().y(); y++, curIndex.y() += 1)
    {
        curIndex.x() = minIdx.x();
        for (size_t x = 0; x < area.getNumCells().x(); x++, curIndex.x() += 1)
        {
            Eigen::Vector3d pos;
            if (!mlsGrid->fromGrid(curIndex, pos))
                continue;

            for (const SurfacePatch<MLSConfig::SLOPE>* p : area.at(x, y))
            {
                pos.z() = (p->getTop() + p->getBottom()) / 2.0;

                YawCheckPatch cp;
                cp.poly = createPolyhedronFromSurfacePatch(p, pos);
                cp.cellXY = pos.head<2>();
                cp.patch = p;
                cp.pos = pos;
                // Top of the ACTUAL polyhedron (its z can exceed the raw patch
                // range through the padded plane reconstruction). An empty
                // polyhedron cannot collide; -inf makes the z prefilter drop it.
                double zMax = -std::numeric_limits<double>::infinity();
                for (auto it = cp.poly.points_begin(); it != cp.poly.points_end(); ++it)
                    zMax = std::max(zMax, CGAL::to_double(it->z()));
                cp.zMax = zMax;
                out.push_back(std::move(cp));
            }
        }
    }
    return out;
}

bool TraversabilityGenerator3d::checkCollisionForYaw(TravGenNode* node, double yaw,
                                                     const std::vector<YawCheckPatch>& patches)
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

    // Reference surface: the node's fitted ground plane. The previous implementation
    // sampled "terrain" height under each corner, which happily picked obstacle patches
    // (their normals are often near-vertical) and hoisted/tilted the modelled body onto
    // the very obstacle it should collide with, reporting the yaw as safe. The fitted
    // plane comes from the ground fit (walls and thick patches are excluded there), so
    // obstacles cannot drag the body upwards.
    Eigen::Vector3d planeNormal = node->getUserData().plane.normal();
    if (!planeNormal.allFinite() || planeNormal.norm() < 1e-6 || std::abs(planeNormal.z()) < 1e-6)
        planeNormal = Eigen::Vector3d::UnitZ();
    else
    {
        planeNormal.normalize();
        if (planeNormal.z() < 0.0)
            planeNormal = -planeNormal;
    }
    //ground plane z at an xy offset from the node centre (the plane passes through nodePos)
    auto planeZAt = [&](double dx, double dy)
    {
        return nodePos.z() - (planeNormal.x() * dx + planeNormal.y() * dy) / planeNormal.z();
    };

    // Build yaw rotation
    Eigen::AngleAxisd yawRotation(yaw, Eigen::Vector3d::UnitZ());

    // 4 lower corners of the robot OBB (offset along the body x axis), rotated by yaw,
    // resting on the ground plane with maxStepHeight clearance
    const double offX = config.footprintOffsetX;
    std::vector<Eigen::Vector3d> localCorners = {
        {offX + hx,  hy, 0.0},
        {offX + hx, -hy, 0.0},
        {offX - hx, -hy, 0.0},
        {offX - hx,  hy, 0.0},
    };

    std::vector<Eigen::Vector3d> bottomCorners;
    for (const auto& lc : localCorners)
    {
        const Eigen::Vector3d rotated = yawRotation * lc;
        bottomCorners.push_back({nodePos.x() + rotated.x(),
                                 nodePos.y() + rotated.y(),
                                 planeZAt(rotated.x(), rotated.y()) + config.maxStepHeight});
    }

    const Eigen::Vector3d heightOffset = planeNormal * config.robotHeight;

    // Add upper 4 corners
    std::vector<Eigen::Vector3d> robotEdges8;
    for (const auto& c : bottomCorners)
        robotEdges8.push_back(c);
    for (const auto& c : bottomCorners)
        robotEdges8.push_back(c + heightOffset);

    Polyhedron_3 robot = generatePolyhedron(robotEdges8);

    // Same boundary semantics as before the patch cache: a rotation-safe window
    // that leaves the map means the cell cannot be traversable at any yaw.
    const double halfDiag = footprintMaxReach(config);
    const double zSlack = halfDiag * std::tan(config.maxSlope);
    Eigen::Vector3d searchMin(-halfDiag, -halfDiag, config.maxStepHeight - zSlack);
    Eigen::Vector3d searchMax( halfDiag,  halfDiag, config.maxStepHeight + config.robotHeight + zSlack);
    searchMin += nodePos;
    searchMax += nodePos;
    const Eigen::AlignedBox3d limitBox(searchMin, searchMax);
    Index minIdx, maxIdx;
    if (!mlsGrid->toGrid(limitBox.min(), minIdx) || !mlsGrid->toGrid(limitBox.max(), maxIdx))
        return false;

    // Conservative prefilters applied per cached patch BEFORE any exact CGAL
    // test. XY: the patch cell (padded by its half-diagonal plus the tilt-induced
    // XY reach of the body top) must overlap the yaw-rotated footprint box.
    // Z: a patch entirely below the tilted body-bottom plane (ground under the
    // belly -- the vast majority of the window) cannot collide.
    const double cellHalfDiag = mlsGrid->getResolution().x() * M_SQRT1_2;
    const double tiltReach = config.robotHeight * planeNormal.head<2>().norm();
    const double xyMargin = cellHalfDiag + tiltReach + 1e-6;
    const double gradSlack = cellHalfDiag *
        planeNormal.head<2>().norm() / std::max(1e-6, planeNormal.z());
    const double cosYaw = std::cos(yaw);
    const double sinYaw = std::sin(yaw);

    for (const YawCheckPatch& cp : patches)
    {
        const double dx = cp.cellXY.x() - nodePos.x();
        const double dy = cp.cellXY.y() - nodePos.y();
        const double localX =  cosYaw * dx + sinYaw * dy;
        const double localY = -sinYaw * dx + cosYaw * dy;
        if (localX < offX - hx - xyMargin || localX > offX + hx + xyMargin ||
            std::abs(localY) > hy + xyMargin)
            continue;

        const double bodyBottomZ = planeZAt(dx, dy) + config.maxStepHeight;
        if (cp.zMax < bodyBottomZ - gradSlack - 1e-6)
            continue;

        if (CGAL::Polygon_mesh_processing::do_intersect(cp.poly, robot))
        {
                // Aliases keep the debug-drawing block below identical to the
                // pre-cache implementation.
                const SurfacePatch<MLSConfig::SLOPE>* p = cp.patch;
                Eigen::Vector3d pos = cp.pos;
                (void)p;
                (void)pos;
                {
#ifdef ENABLE_DEBUG_DRAWINGS
                    // V3DD must NOT run from OpenMP workers: DRAW_* marshals to the
                    // Qt GUI thread with a BLOCKING invoke, and during expansion the
                    // GUI thread is parked on this parallel region's barrier — a
                    // guaranteed deadlock (observed live in the travgen GUI). Draw
                    // only when the check runs on a serial path.
#ifdef _OPENMP
                    if (!omp_in_parallel())
#endif
                    {
                        static int yawCollisionCounter = 0;
                        yawCollisionCounter++;
                        if (yawCollisionCounter % 200 == 0)
                        {
                            std::string robotPrefix = "colliding_robot_yaw_" + std::to_string(yawCollisionCounter);
                            Eigen::Vector4d red{1.0, 0.0, 0.0, 1.0};
                            
                            V3DD::COMPLEX_DRAWING([&]
                            {
                                for(size_t i = 0; i < 4; i++)
                                {
                                    Eigen::Vector3d lo = bottomCorners[i];
                                    Eigen::Vector3d hi = bottomCorners[i] + heightOffset;
                                    size_t next = (i + 1) % 4;
                                    Eigen::Vector3d loNext = bottomCorners[next];
                                    Eigen::Vector3d hiNext = bottomCorners[next] + heightOffset;
                                    
                                    V3DD::DRAW_LINE(robotPrefix + "_v" + std::to_string(i), lo, hi, red);
                                    V3DD::DRAW_LINE(robotPrefix + "_lo" + std::to_string(i), lo, loNext, red);
                                    V3DD::DRAW_LINE(robotPrefix + "_hi" + std::to_string(i), hi, hiNext, red);
                                }
                            });

                            // Draw colliding MLS patch sloped prism
                            std::vector<Eigen::Vector3f> polygonPoints;
                            Eigen::Vector2f cellCenter = pos.head<2>().cast<float>();
                            Eigen::Vector2f cellSize(config.gridResolution, config.gridResolution);
                            maps::grid::getPolygon(polygonPoints, *p, cellCenter, cellSize);

                            if (polygonPoints.size() >= 3)
                            {
                                std::string patchPrefix = "colliding_patch_yaw_" + std::to_string(yawCollisionCounter);
                                Eigen::Vector4d yellow{1.0, 0.8, 0.0, 1.0};
                                Eigen::Vector3f normalf = p->getNormal();
                                if (normalf.z() < 0)
                                    normalf *= -1.0f;
                                Eigen::Vector3d normal{normalf.x(), normalf.y(), normalf.z()};
                                if (normal.norm() > 1e-6)
                                    normal.normalize();
                                else
                                    normal = Eigen::Vector3d::UnitZ();
                                const double thickness = 0.02;

                                V3DD::COMPLEX_DRAWING([&]
                                {
                                    for (size_t i = 0; i < polygonPoints.size(); i++)
                                    {
                                        Eigen::Vector3d hi = polygonPoints[i].cast<double>();
                                        Eigen::Vector3d lo = hi - thickness * normal;
                                        size_t next = (i + 1) % polygonPoints.size();
                                        Eigen::Vector3d hiNext = polygonPoints[next].cast<double>();
                                        Eigen::Vector3d loNext = hiNext - thickness * normal;

                                        V3DD::DRAW_LINE(patchPrefix + "_v" + std::to_string(i), lo, hi, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_lo" + std::to_string(i), lo, loNext, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_hi" + std::to_string(i), hi, hiNext, yellow);
                                    }
                                });
                            }
                            else
                            {
                                std::string patchPrefix = "colliding_patch_yaw_" + std::to_string(yawCollisionCounter);
                                Eigen::Vector4d yellow{1.0, 0.8, 0.0, 1.0};
                                float minZ, maxZ;
                                p->getRange(minZ, maxZ);
                                double hres = config.gridResolution / 2.0;

                                std::vector<Eigen::Vector3d> corners = {
                                    {pos.x() + hres, pos.y() + hres, (double)minZ},
                                    {pos.x() + hres, pos.y() - hres, (double)minZ},
                                    {pos.x() - hres, pos.y() - hres, (double)minZ},
                                    {pos.x() - hres, pos.y() + hres, (double)minZ}
                                };
                                double height = maxZ - minZ;

                                V3DD::COMPLEX_DRAWING([&]
                                {
                                    for (size_t i = 0; i < 4; i++)
                                    {
                                        Eigen::Vector3d lo = corners[i];
                                        Eigen::Vector3d hi = corners[i] + Eigen::Vector3d(0, 0, height);
                                        size_t next = (i + 1) % 4;
                                        Eigen::Vector3d loNext = corners[next];
                                        Eigen::Vector3d hiNext = corners[next] + Eigen::Vector3d(0, 0, height);

                                        V3DD::DRAW_LINE(patchPrefix + "_v" + std::to_string(i), lo, hi, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_lo" + std::to_string(i), lo, loNext, yellow);
                                        V3DD::DRAW_LINE(patchPrefix + "_hi" + std::to_string(i), hi, hiNext, yellow);
                                    }
                                });
                            }
                        }
                    }
#endif
                    return false; // collision found — this yaw is not safe
                }
        }
    }

    return true; // no collision — yaw is safe
}

bool TraversabilityGenerator3d::computeSafeOrientations(TravGenNode* node)
{
    // Exactly numYawSamples collision checks over the FULL circle [0,360deg),
    // independent of the footprint offset -- no 180deg mirroring shortcut. The
    // offset is applied to the robot geometry of EVERY check (hull corners and
    // XY prefilter in checkCollisionForYaw, offset-aware search window in
    // collectYawCheckPatches), so each yaw intersects the correctly shifted
    // set of MLS patches. numYawSamples is both the check count and the
    // angular resolution (step = 360deg / numYawSamples; 12 -> 30deg).
    const int numSamples = std::max(1, config.numYawSamples);
    const double step = 2.0 * M_PI / numSamples;

    // The patch polyhedra are yaw-independent: build them once for this node
    // and reuse them for every sampled yaw.
    const std::vector<YawCheckPatch> yawPatches = collectYawCheckPatches(node);

    std::vector<double> safeYaws;
    for (int i = 0; i < numSamples; ++i)
    {
        const double yaw = i * step;
        if (checkCollisionForYaw(node, yaw, yawPatches))
        {
            safeYaws.push_back(yaw);
        }
    }

    if (safeYaws.empty())
        return false;

    // Each safe yaw gets a wedge as wide as the sampling step (centered on the yaw), so wedges
    // tile the sampled directions without large gaps and stay tighter as resolution increases.
    node->getUserData().allowedOrientations.clear();
    for (const double yaw : safeYaws)
    {
        node->getUserData().allowedOrientations.emplace_back(
            base::Angle::fromRad(yaw - step / 2.0), step);
    }
    return true;
}

void TraversabilityGenerator3d::inflateFrontiers()
{
    const double growRadiusSquared = std::pow(footprintMaxReach(config), 2);

    for(TravGenNode *n : frontierNodesGrowList)
    {
        // fillEnclosedUnknownRegions() may have retyped a former frontier to TRAVERSABLE
        // (its pocket got filled) -- such nodes must not seed frontier inflation anymore.
        if(n->getType() != TraversabilityNodeBase::FRONTIER)
            continue;

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

void TraversabilityGenerator3d::fillEnclosedUnknownRegions()
{
    // Unmeasured cells are typed OBSTACLE at creation (tracked in unmeasuredNodesList).
    // Interior occlusion pockets (scan shadows, holes in the point cloud) that are fully
    // enclosed by mapped terrain are re-expanded here with the local-evidence gate
    // bypassed, i.e. interpolated from the surrounding measured ground -- still passing
    // the normal slope and step checks. Unmeasured cells at the outer map edge, and
    // pockets wider than the plane fit's search radius, remain OBSTACLE: unmeasured
    // space is not traversable.
    const Vector2ui numCells = trMap.getNumCells();
    if(numCells.x() == 0 || numCells.y() == 0)
        return;
    const int sizeX = static_cast<int>(numCells.x());
    const int sizeY = static_cast<int>(numCells.y());

    // 2D mask: does a cell contain any node (on any level)?
    std::vector<uint8_t> hasNode(static_cast<size_t>(sizeX) * sizeY, 0);
    for(int y = 0; y < sizeY; y++)
        for(int x = 0; x < sizeX; x++)
            if(!trMap.at(x, y).empty())
                hasNode[static_cast<size_t>(y) * sizeX + x] = 1;

    // Flood-fill from the grid border over node-less cells: exterior emptiness.
    std::vector<uint8_t> exteriorEmpty(static_cast<size_t>(sizeX) * sizeY, 0);
    std::deque<std::pair<int, int>> flood;
    auto pushEmpty = [&](int x, int y)
    {
        if(x < 0 || y < 0 || x >= sizeX || y >= sizeY)
            return;
        const size_t i = static_cast<size_t>(y) * sizeX + x;
        if(hasNode[i] || exteriorEmpty[i])
            return;
        exteriorEmpty[i] = 1;
        flood.emplace_back(x, y);
    };
    for(int x = 0; x < sizeX; x++) { pushEmpty(x, 0); pushEmpty(x, sizeY - 1); }
    for(int y = 0; y < sizeY; y++) { pushEmpty(0, y); pushEmpty(sizeX - 1, y); }
    while(!flood.empty())
    {
        const std::pair<int, int> c = flood.front();
        flood.pop_front();
        pushEmpty(c.first + 1, c.second);
        pushEmpty(c.first - 1, c.second);
        pushEmpty(c.first, c.second + 1);
        pushEmpty(c.first, c.second - 1);
    }

    // An unmeasured node is a genuine edge node iff it sits on the grid border or is
    // 8-adjacent to exterior emptiness. Everything else rims an interior pocket.
    std::deque<TravGenNode*> pocketNodes;
    for(TravGenNode* n : unmeasuredNodesList)
    {
        if(n->getUserData().nodeType != NodeType::OBSTACLE)
            continue;  // already refilled in an earlier pass
        const int x = n->getIndex().x();
        const int y = n->getIndex().y();
        bool edge = (x == 0 || y == 0 || x == sizeX - 1 || y == sizeY - 1);
        for(int dy = -1; dy <= 1 && !edge; dy++)
        {
            for(int dx = -1; dx <= 1 && !edge; dx++)
            {
                const int nx = x + dx;
                const int ny = y + dy;
                if(nx < 0 || ny < 0 || nx >= sizeX || ny >= sizeY)
                    continue;
                if(exteriorEmpty[static_cast<size_t>(ny) * sizeX + nx])
                    edge = true;
            }
        }
        if(!edge)
            pocketNodes.push_back(n);
    }
    // Entries are consumed by this pass: edge nodes simply stay OBSTACLE, refit failures
    // below re-register themselves for the next pass via createTraversabilityPatchAt.
    unmeasuredNodesList.clear();

    if(pocketNodes.empty())
        return;

    LOG_INFO_S << "TraversabilityGenerator3d: re-expanding " << pocketNodes.size()
               << " interior unmeasured nodes (enclosed pockets)";

    // Re-fit the pocket rim with the evidence gate bypassed, then re-run the expansion
    // loop so the flood fills the pocket interior. The re-expansion cannot escape the
    // pocket: every node surrounding it is already expanded.
    std::deque<TravGenNode*> candidates;
    // Refit the pocket rim in parallel: computePlaneRansac only reads the MLS and
    // writes the node's own data. Queue bookkeeping stays serial below.
    const std::vector<TravGenNode*> pocketVec(pocketNodes.begin(), pocketNodes.end());
    std::vector<uint8_t> fitOkVec(pocketVec.size(), 0);
    const std::int64_t numPocket = static_cast<std::int64_t>(pocketVec.size());
    #pragma omp parallel for schedule(dynamic) if(numPocket >= kMinParallelItems)
    for(std::int64_t i = 0; i < numPocket; i++)
    {
        TravGenNode* node = pocketVec[i];
        const float oldHeight = node->getHeight();
        node->setType(TraversabilityNodeBase::UNSET);
        node->getUserData().nodeType = NodeType::UNSET;
        const bool fitOk = computePlaneRansac(*node);
        // Keep the creation-time (interpolated) height: the node already sits in its
        // height-ordered level list, so it must not move vertically after insertion.
        node->setHeight(oldHeight);
        fitOkVec[i] = fitOk ? 1 : 0;
    }
    for(std::int64_t i = 0; i < numPocket; i++)
    {
        TravGenNode* node = pocketVec[i];
        if(!fitOkVec[i])
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            node->getUserData().nodeType = NodeType::OBSTACLE;
            continue;
        }
        node->setNotExpanded();
        candidates.push_back(node);
    }

    while(!candidates.empty())
    {
        TravGenNode* node = candidates.front();
        candidates.pop_front();
        if(node->isExpanded())
            continue;
        if(!expandNode(node))
            continue;
        for(auto* n : node->getConnections())
        {
            if(!n->isExpanded())
                candidates.push_back(static_cast<TravGenNode*>(n));
        }
    }
}

void TraversabilityGenerator3d::setConfig(const TraversabilityConfig &config)
{
    warnOnDegenerateFootprint(config);
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

    const double offX = config.footprintOffsetX;
    double robotHalfLength = config.robotSizeX / 2.0;
    double robotHalfWidth = config.robotSizeY / 2.0;
    double robotHalfHeight = config.robotHeight / 2.0;

    robotEdges = {
        {offX + robotHalfLength, robotHalfWidth, robotHalfHeight},    // Top-right-front
        {offX + robotHalfLength, robotHalfWidth, -robotHalfHeight},   // Top-right-back
        {offX + robotHalfLength, -robotHalfWidth, robotHalfHeight},   // Bottom-right-front
        {offX + robotHalfLength, -robotHalfWidth, -robotHalfHeight},  // Bottom-right-back
        {offX - robotHalfLength, robotHalfWidth, robotHalfHeight},    // Top-left-front
        {offX - robotHalfLength, robotHalfWidth, -robotHalfHeight},   // Top-left-back
        {offX - robotHalfLength, -robotHalfWidth, robotHalfHeight},   // Bottom-left-front
        {offX - robotHalfLength, -robotHalfWidth, -robotHalfHeight}   // Bottom-left-back
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

    // Wave-synchronous parallel BFS. Per wave, the expensive read-only geometry
    // (step-height AABB/OBB checks and the RANSAC plane fits of newly discovered
    // cells) runs multi-threaded; every mutation of the shared graph (node
    // insertion, ids, connections, type changes, grow lists) then runs on this
    // thread in wave order, reproducing the serial BFS. The only deviation from
    // strict FIFO order is which same-wave parent seeds a new cell's fit height;
    // competing request heights lie within maxStepHeight of each other, so only
    // borderline cells can differ.
#ifdef _OPENMP
    // config.numThreads semantics: 0 = do not parallelize (single-threaded on
    // the calling thread, no OpenMP worker team); N > 0 = use exactly N threads.
    omp_set_num_threads(std::max(1, config.numThreads));
    LOG_INFO_S << "TraversabilityGenerator3d: expanding with up to "
               << omp_get_max_threads() << " threads.";
    // OpenMP workers inherit the calling thread's CPU affinity. Some libraries
    // (notably OSG at realize()) pin their thread to a single core, which
    // silently collapses the whole team onto that core (seen live: 17 s
    // instead of 2 s in the travgen GUI). Warn instead of failing silently.
    {
        cpu_set_t affinityMask;
        if (sched_getaffinity(0, sizeof(affinityMask), &affinityMask) == 0)
        {
            const int allowedCpus = CPU_COUNT(&affinityMask);
            if (allowedCpus < omp_get_max_threads())
            {
                LOG_WARN_S << "TraversabilityGenerator3d: the calling thread is "
                           << "restricted to " << allowedCpus << " CPU(s) but "
                           << omp_get_max_threads() << " threads were requested -- "
                           << "the OpenMP team inherits this mask and will share "
                           << "those CPU(s). Widen the thread's affinity (e.g. "
                           << "sched_setaffinity) before calling expandAll().";
            }
        }
    }
#endif
    const auto expandStart = std::chrono::steady_clock::now();

    std::vector<TravGenNode*> wave;
    wave.push_back(startNode);

    int cnd = 0;
    // Accumulated BFS sub-phase timings, reported once per expansion; used to
    // pinpoint whether slowdowns sit in the parallel or the serial phases.
    double tClassify = 0.0, tCollect = 0.0, tPrefit = 0.0, tFinalize = 0.0;
    int numWaves = 0;
    typedef std::chrono::steady_clock Clock;

    while(!wave.empty())
    {
        // A node can be enqueued by several parents (also across waves): keep the
        // first occurrence of each not-yet-expanded node, mirroring the serial
        // isExpanded() check on dequeue.
        {
            std::unordered_set<TravGenNode*> seen;
            std::vector<TravGenNode*> filtered;
            filtered.reserve(wave.size());
            for(TravGenNode* n : wave)
            {
                if(!n->isExpanded() && seen.insert(n).second)
                    filtered.push_back(n);
            }
            wave.swap(filtered);
        }
        if(wave.empty())
            break;

        const std::int64_t waveSize = static_cast<std::int64_t>(wave.size());
        numWaves++;
        auto tPhase = Clock::now();

        // Phase 1 (parallel): classification checks, read-only on the shared maps.
        std::vector<uint8_t> outcome(waveSize, 0);
        #pragma omp parallel for schedule(dynamic) if(waveSize >= kMinParallelItems)
        for(std::int64_t i = 0; i < waveSize; i++)
        {
            outcome[i] = static_cast<uint8_t>(classifyNode(wave[i]));
        }

        tClassify += std::chrono::duration<double>(Clock::now() - tPhase).count();
        tPhase = Clock::now();

        // Phase 2a (serial, cheap plane math): collect the cells this wave will
        // create. emplace() keeps the FIRST request per cell = wave order, which
        // matches the serial creation order.
        PrefitCache prefits;
        for(std::int64_t i = 0; i < waveSize; i++)
        {
            if(static_cast<NodeClassification>(outcome[i]) != NodeClassification::Traversable)
                continue;
            for(const Index& idxS : kNeighborOffsets)
            {
                Index idx;
                double localHeight = 0.0;
                const NeighborRequest req = computeNeighborRequest(wave[i], idxS, idx, localHeight);
                if(req == NeighborRequest::Abort)
                    break;
                if(req == NeighborRequest::Skip)
                    continue;
                if(findMatchingTraversabilityPatchAt(idx, localHeight))
                    continue;
                prefits.emplace(cellKey(idx), PrefitEntry{idx, localHeight, PrefitPatch{}});
            }
        }

        tCollect += std::chrono::duration<double>(Clock::now() - tPhase).count();
        tPhase = Clock::now();

        // Phase 2b (parallel): RANSAC-fit the new cells.
        std::vector<PrefitEntry*> fitJobs;
        fitJobs.reserve(prefits.size());
        for(auto& kv : prefits)
            fitJobs.push_back(&kv.second);
        const std::int64_t numJobs = static_cast<std::int64_t>(fitJobs.size());
        #pragma omp parallel for schedule(dynamic) if(numJobs >= kMinParallelItems)
        for(std::int64_t i = 0; i < numJobs; i++)
        {
            fitJobs[i]->patch = buildPatchNodeAt(fitJobs[i]->idx, fitJobs[i]->requestHeight);
        }

        tPrefit += std::chrono::duration<double>(Clock::now() - tPhase).count();
        tPhase = Clock::now();

        // Phase 2c (serial): the original expansion semantics, in wave order.
        std::vector<TravGenNode*> nextWave;
        for(std::int64_t i = 0; i < waveSize; i++)
        {
            TravGenNode* node = wave[i];

            if(config.useSoilInformation)
            {
                const Eigen::Vector3d nodePos = node->getPosition(trMap);
                generateStartSoilNode(nodePos);
            }

            node->setExpanded();

            cnd++;
            if((cnd % 1000) == 0)
            {
                LOG_DEBUG_S << "TraversabilityGenerator3d: Expanded " << cnd << " traversability nodes.";
            }

            switch(static_cast<NodeClassification>(outcome[i]))
            {
                case NodeClassification::Unknown:
                    continue;
                case NodeClassification::PreexistingObstacle:
                    obstacleNodesGrowList.push_back(node);
                    continue;
                case NodeClassification::Obstacle:
                    node->setType(TraversabilityNodeBase::OBSTACLE);
                    node->getUserData().nodeType = NodeType::OBSTACLE;
                    obstacleNodesGrowList.push_back(node);
                    continue;
                case NodeClassification::Traversable:
                    break;
            }

            addConnectedPatches(node, &prefits);

            if(checkForFrontier(node))
            {
                node->setType(TraversabilityNodeBase::FRONTIER);
                node->getUserData().nodeType = NodeType::FRONTIER;
                frontierNodesGrowList.push_back(node);
                continue;
            }

            node->setType(TraversabilityNodeBase::TRAVERSABLE);
            node->getUserData().nodeType = NodeType::TRAVERSABLE;

            for(auto* n : node->getConnections())
            {
                if(n->isExpanded())
                    continue;
                if(expandDist > 0)
                {
                    const double dist = (startNode->getPosition(trMap) - n->getPosition(trMap)).norm();
                    if(dist > expandDist)
                        continue;
                }
                nextWave.push_back(static_cast<TravGenNode*>(n));
            }
        }

        // Pre-fitted nodes nobody consumed (their request was satisfied by a
        // wave-mate's insertion instead) were never part of the map: free them.
        for(auto& kv : prefits)
        {
            if(kv.second.patch.node)
            {
                delete kv.second.patch.node;
                kv.second.patch.node = nullptr;
            }
        }

        wave.swap(nextWave);
        tFinalize += std::chrono::duration<double>(Clock::now() - tPhase).count();
    }

    LOG_INFO_S << "TraversabilityGenerator3d: BFS breakdown over " << numWaves
               << " waves: classify(par) " << tClassify << " s, collect(ser) "
               << tCollect << " s, prefit(par) " << tPrefit << " s, finalize(ser) "
               << tFinalize << " s.";

    const auto tBfs = std::chrono::steady_clock::now();
    fillEnclosedUnknownRegions();
    const auto tPockets = std::chrono::steady_clock::now();
    inflateFrontiers();
    const auto tFrontiers = std::chrono::steady_clock::now();
    inflateObstacles();
    const auto tObstacles = std::chrono::steady_clock::now();

    const auto sec = [](std::chrono::steady_clock::time_point a,
                        std::chrono::steady_clock::time_point b)
    { return std::chrono::duration<double>(b - a).count(); };
    LOG_INFO_S << "TraversabilityGenerator3d: expanded " << cnd << " nodes ("
               << currentNodeId << " total in map) in "
               << sec(expandStart, tObstacles) << " s (bfs " << sec(expandStart, tBfs)
               << ", pockets " << sec(tBfs, tPockets)
               << ", frontiers " << sec(tPockets, tFrontiers)
               << ", obstacle inflation " << sec(tFrontiers, tObstacles) << ").";

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
    // The AABB check already keeps the robot centre at least footprintMinHalfExtent()
    // away from any obstacle (the tight-axis footprint boundary).  We only need to
    // inflate by the remaining gap to the rotation-safe reach so that the robot can
    // still rotate freely at the edge of the traversable zone without its corners
    // hitting the obstacle. Using the full reach here would double-count the footprint
    // clearance and close far too much traversable space in narrow corridors. Both
    // radii are offset-aware: an off-center footprint shrinks the guaranteed core and
    // extends the swept circle.
    const double halfDiagonal = footprintMaxReach(config);

    // Geometric gap between the AABB boundary and the rotation-safe circle.
    const double inflGap = halfDiagonal - footprintMinHalfExtent(config)/2;

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

    // Pass 1 (serial, cheap): walk the inflation bands of all obstacle seeds and
    // collect the unique set of nodes to evaluate. The walk only depends on the
    // graph and node types as they are NOW; evaluation results never change which
    // nodes get collected (evaluatedNodes already deduplicated re-visits before).
    std::vector<TravGenNode*> toEvaluate;

    for (TravGenNode *n : obstacleNodesGrowList)
    {
        // fillEnclosedUnknownRegions() may have refilled a former unmeasured obstacle to
        // TRAVERSABLE -- such nodes must not seed obstacle inflation anymore.
        if(n->getType() != TraversabilityNodeBase::OBSTACLE)
            continue;

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
                        toEvaluate.push_back(node);
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

    // Pass 2 (parallel): the expensive per-node yaw sampling. computeSafeOrientations
    // only reads the MLS and writes the node's OWN allowedOrientations, so distinct
    // nodes evaluate concurrently without locking.
    std::vector<uint8_t> hasSafeYaw(toEvaluate.size(), 0);
    const std::int64_t numEval = static_cast<std::int64_t>(toEvaluate.size());
    #pragma omp parallel for schedule(dynamic) if(numEval >= kMinParallelItems)
    for(std::int64_t i = 0; i < numEval; i++)
    {
        hasSafeYaw[i] = computeSafeOrientations(toEvaluate[i]) ? 1 : 0;
    }

    // Pass 3 (serial): type writeback.
    for(std::int64_t i = 0; i < numEval; i++)
    {
        TravGenNode* node = toEvaluate[i];
        if (hasSafeYaw[i])
        {
            // Some orientations are safe — partially traversable
            node->setType(TraversabilityNodeBase::TRAVERSABLE);
            node->getUserData().nodeType = NodeType::PARTIALLY_TRAVERSABLE;
        }
        else
        {
            // No safe orientation found: the robot cannot occupy this cell at
            // any yaw, so it is a plain obstacle (not merely footprint-inflated).
            node->setType(TraversabilityNodeBase::OBSTACLE);
            node->getUserData().nodeType = NodeType::OBSTACLE;
        }
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
    unmeasuredNodesList.clear();
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
    // Populate the soil map alongside expansion only when soil information is actually used.
    // Otherwise this allocates and inserts a SoilNode for every expanded cell into a map that
    // nothing consumes (the soil pipeline in the GUI is itself gated on useSoilInformation).
    if(config.useSoilInformation)
    {
        const Eigen::Vector3d nodePos = node->getPosition(trMap);
        generateStartSoilNode(nodePos);
    }

    node->setExpanded();

    switch(classifyNode(node))
    {
        case NodeClassification::Unknown:
            return false;
        case NodeClassification::PreexistingObstacle:
            obstacleNodesGrowList.push_back(node);
            return false;
        case NodeClassification::Obstacle:
            node->setType(TraversabilityNodeBase::OBSTACLE);
            node->getUserData().nodeType = NodeType::OBSTACLE;
            obstacleNodesGrowList.push_back(node);
            return false;
        case NodeClassification::Traversable:
            break;
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

TraversabilityGenerator3d::NodeClassification TraversabilityGenerator3d::classifyNode(TravGenNode* node)
{
    if(node->getType() == TraversabilityNodeBase::UNKNOWN)
    {
        return NodeClassification::Unknown;
    }

    if(node->getType() == TraversabilityNodeBase::OBSTACLE)
    {
        return NodeClassification::PreexistingObstacle;
    }

    if(node->getUserData().slope > config.maxSlope)
    {
        return NodeClassification::Obstacle;
    }

    if(!checkStepHeightAABB(node))
    {
        if(!checkStepHeightOBB(node))
        {
            return NodeClassification::Obstacle;
        }
    }

    if(config.enableInclineLimitting)
    {
        if(!computeAllowedOrientations(node))
        {
            return NodeClassification::Obstacle;
        }
    }

    return NodeClassification::Traversable;
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
    Eigen::Vector3d max(config.gridResolution/2.0 - 1e-5, config.gridResolution/2.0 - 1e-5, config.maxStepHeight + config.robotHeight);
    
    
    min += nodePos;
    max += nodePos;
    
    const Eigen::AlignedBox3d boundingBox(min, max);
    
    size_t numIntersections = 0;
    const View area = mlsGrid->intersectCuboid(boundingBox, numIntersections);
    if(numIntersections > 0)
        return false;
    
    return true;
}


TraversabilityGenerator3d::PrefitPatch TraversabilityGenerator3d::buildPatchNodeAt(const maps::grid::Index& idx, const double curHeight)
{
    PrefitPatch result;

    maps::grid::Vector3d globalPos;
    trMap.fromGrid(idx, globalPos);
    Index mlsIdx;
    if(!mlsGrid->toGrid(globalPos, mlsIdx))
    {
        return result;
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

    TravGenNode* ret = new TravGenNode(0.0, idx);
    ret->getUserData().cost = 0;

    for(double height: candidates)
    {
        ret->setHeight(height);
        ret->setNotExpanded();
        ret->setType(TraversabilityNodeBase::UNSET);
        ret->getUserData().nodeType = NodeType::UNSET;

        //there is a neighboring patch in the mls that has a reachable hight
        const bool planeOk = computePlaneRansac(*ret);
        if(!planeOk)
        {
            // Unmeasured / unfittable cells become obstacles immediately -- there is no
            // UNKNOWN state anymore. checkForFrontier() therefore never sees UNKNOWN
            // neighbours, so no FRONTIER / INFLATED_FRONTIER rings form around unmeasured
            // space. The nodes are remembered in unmeasuredNodesList so that
            // fillEnclosedUnknownRegions() can still refill interior pockets (by type
            // alone they are indistinguishable from real obstacles).
            ret->setType(TraversabilityNodeBase::OBSTACLE);
            ret->getUserData().nodeType = NodeType::OBSTACLE;
        }

        if((ret->getHeight() - config.maxStepHeight) <= curHeight && (ret->getHeight() + config.maxStepHeight) >= curHeight)
        {
            result.node = ret;
            result.unmeasured = !planeOk;
            return result;
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
    result.node = ret;
    result.unmeasured = false;
    return result;
}

TravGenNode* TraversabilityGenerator3d::finishPatchNode(const PrefitPatch& prefit, const maps::grid::Index& idx)
{
    if(!prefit.node)
        return nullptr;
    prefit.node->getUserData().id = currentNodeId++;
    trMap.at(idx).insert(prefit.node);
    if(prefit.unmeasured)
        unmeasuredNodesList.push_back(prefit.node);
    return prefit.node;
}

TravGenNode *TraversabilityGenerator3d::createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight)
{
    return finishPatchNode(buildPatchNodeAt(idx, curHeight), idx);
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

TraversabilityGenerator3d::NeighborRequest TraversabilityGenerator3d::computeNeighborRequest(
    const TravGenNode* node, const Index& idxS, Index& outIdx, double& outHeight) const
{
    outIdx = Index(node->getIndex() + idxS);

    if(!trMap.inGrid(outIdx))
    {
        return NeighborRequest::Skip;
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
        return NeighborRequest::Abort;
    }
    outHeight = newPos.z();
    //The new patch is not reachable from the current patch
    if(fabs(outHeight - node->getHeight()) > config.maxStepHeight)
    {
        return NeighborRequest::Skip;
    }

    if (!newPos.allFinite()) {
        LOG_ERROR_S << "TraversabilityGenerator3d: newPos contains non-finite values: "
                    << newPos.transpose();
        return NeighborRequest::Skip;
    }
    return NeighborRequest::Ok;
}

void TraversabilityGenerator3d::addConnectedPatches(TravGenNode *  node)
{
    addConnectedPatches(node, nullptr);
}

void TraversabilityGenerator3d::addConnectedPatches(TravGenNode* node, PrefitCache* prefits)
{
    for(const Index &idxS : kNeighborOffsets)
    {
        Index idx;
        double localHeight = 0.0;
        const NeighborRequest req = computeNeighborRequest(node, idxS, idx, localHeight);
        if(req == NeighborRequest::Abort)
        {
            return;
        }
        if(req == NeighborRequest::Skip)
        {
            continue;
        }

        //check if we got an existing node
        TravGenNode *toAdd = findMatchingTraversabilityPatchAt(idx, localHeight);

        if(!toAdd && prefits)
        {
            // Consume a node pre-fitted by the parallel wave phase -- but only if it
            // was fitted for exactly this request. A same-cell request at another
            // height (rare multi-level case) falls through to the serial creation.
            auto it = prefits->find(cellKey(idx));
            if(it != prefits->end() && it->second.patch.node != nullptr &&
               std::abs(it->second.requestHeight - localHeight) < 1e-9)
            {
                toAdd = finishPatchNode(it->second.patch, idx);
                it->second.patch.node = nullptr;
            }
        }

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
