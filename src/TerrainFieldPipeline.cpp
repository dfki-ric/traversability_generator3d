// TerrainField pipeline entry point (config.useTerrainField == true).
// Implements TraversabilityGenerator3d::expandAllTerrainField(): seed-independent
// full-map generation via the terrain_field modules (robust layered ground
// estimation, exact ESDF clearance, analytic allowed orientations).
// See TERRAIN_FIELD_ARCHITECTURE.md.

#include "TraversabilityGenerator3d.hpp"

#include "terrain_field/AttitudeIntervals.hpp"
#include "terrain_field/Cost.hpp"
#include "terrain_field/HeadingIntervals.hpp"
#include "terrain_field/TerrainField.hpp"

#include <base-logging/Logging.hpp>
#include <base/Angle.hpp>
#include <chrono>
#include <cmath>

using namespace maps::grid;

namespace traversability_generator3d
{

namespace
{

terrain_field::RobotModel robotModelFromConfig(const TraversabilityConfig& config)
{
    terrain_field::RobotModel robot;
    robot.sizeX = config.robotSizeX;
    robot.sizeY = config.robotSizeY;
    robot.height = config.robotHeight;
    robot.safetyMargin = 0.0;
    robot.maxSlope = config.maxSlope;
    robot.enableInclineLimitting = config.enableInclineLimitting;
    robot.inclineLimittingMinSlope = config.inclineLimittingMinSlope;
    robot.inclineLimittingLimit = config.inclineLimittingLimit;
    robot.allowForwardDownhill = config.allowForwardDownhill;
    robot.articulated = config.articulatedSuspension;
    return robot;
}

/** Distance from @p center to the map boundary rectangle [0, w*res] x [0, h*res].
 *  The legacy generator prunes nodes whose footprint leaves the map; here the map
 *  edge acts as an obstacle line for the clearance reasoning. */
double distToMapBorder(const Eigen::Vector2d& center, std::size_t w, std::size_t h,
                       double res)
{
    return std::min(std::min(center.x(), (double)w * res - center.x()),
                    std::min(center.y(), (double)h * res - center.y()));
}

/** Append virtual obstacle points along the map boundary within @p radius of
 *  @p center so the exact heading intervals also respect the map edge. */
void appendBorderObstacles(const Eigen::Vector2d& center, double radius,
                           std::size_t w, std::size_t h, double res,
                           std::vector<Eigen::Vector2d>& out)
{
    const double maxX = (double)w * res;
    const double maxY = (double)h * res;
    const double step = 0.5 * res;

    auto sampleLine = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
    {
        const double len = (b - a).norm();
        const int n = std::max(1, (int)std::ceil(len / step));
        for (int k = 0; k <= n; ++k)
        {
            const Eigen::Vector2d q = a + (b - a) * ((double)k / n);
            if ((q - center).norm() <= radius)
                out.push_back(q);
        }
    };

    if (center.x() - radius < 0.0)
        sampleLine(Eigen::Vector2d(0.0, std::max(0.0, center.y() - radius)),
                   Eigen::Vector2d(0.0, std::min(maxY, center.y() + radius)));
    if (center.x() + radius > maxX)
        sampleLine(Eigen::Vector2d(maxX, std::max(0.0, center.y() - radius)),
                   Eigen::Vector2d(maxX, std::min(maxY, center.y() + radius)));
    if (center.y() - radius < 0.0)
        sampleLine(Eigen::Vector2d(std::max(0.0, center.x() - radius), 0.0),
                   Eigen::Vector2d(std::min(maxX, center.x() + radius), 0.0));
    if (center.y() + radius > maxY)
        sampleLine(Eigen::Vector2d(std::max(0.0, center.x() - radius), maxY),
                   Eigen::Vector2d(std::min(maxX, center.x() + radius), maxY));
}

void setAllowedOrientations(TravGenNode* node,
                            const terrain_field::AngleIntervalSet& allowed)
{
    auto& segments = node->getUserData().allowedOrientations;
    segments.clear();
    if (allowed.isFull())
    {
        segments.emplace_back(base::Angle::fromRad(0), 2 * M_PI);
        return;
    }
    for (const auto& arc : allowed.arcs())
    {
        segments.emplace_back(base::Angle::fromRad(arc.first), arc.second - arc.first);
    }
}

}

void TraversabilityGenerator3d::expandAllTerrainField()
{
    if (terrainFieldGenerated)
        return;
    if (!mlsGrid)
    {
        LOG_ERROR_S << "expandAllTerrainField: no MLS grid set";
        return;
    }

    const auto tStart = std::chrono::steady_clock::now();

    clearTrMap();
    currentNodeId = 0;

    const Vector2ui numCells = trMap.getNumCells();
    const std::size_t w = numCells.x();
    const std::size_t h = numCells.y();
    if (w == 0 || h == 0)
    {
        LOG_ERROR_S << "expandAllTerrainField: traversability map has zero size";
        return;
    }

    // --- L0: ingest MLS patches into the (coarser) trav grid -----------------------
    terrain_field::TerrainParams params;
    params.gridResolution = config.gridResolution;
    params.maxStepHeight = config.maxStepHeight;
    params.maxSlope = config.maxSlope;

    terrain_field::TerrainGridInput input;
    input.width = w;
    input.height = h;
    input.cells.assign(w * h, {});

    const Vector2ui mlsCells = mlsGrid->getNumCells();
    std::size_t patchCount = 0;
    for (std::size_t my = 0; my < mlsCells.y(); ++my)
    {
        for (std::size_t mx = 0; mx < mlsCells.x(); ++mx)
        {
            const Index mlsIdx(mx, my);
            Vector3d pos;
            if (!mlsGrid->fromGrid(mlsIdx, pos))
                continue;
            Index tIdx;
            if (!trMap.toGrid(pos, tIdx))
                continue;
            const std::size_t cellIdx = (std::size_t)tIdx.y() * w + tIdx.x();

            for (const Patch& p : mlsGrid->at(mlsIdx))
            {
                terrain_field::PatchSample s;
                s.zMin = p.getBottom();
                s.zMax = p.getTop();
                s.mean = 0.5f * (s.zMin + s.zMax);
                Eigen::Vector3f n = p.getNormal();
                if (n.squaredNorm() < 1e-12f)
                    n = Eigen::Vector3f::UnitZ();
                if (n.z() < 0.f)
                    n = -n;
                n.normalize();
                s.normal = n;
                const float halfThickness = 0.25f * (s.zMax - s.zMin);
                s.variance = std::max((float)(params.sigma0 * params.sigma0),
                                      halfThickness * halfThickness);
                input.cells[cellIdx].push_back(s);
                ++patchCount;
            }
        }
    }

    // --- L1 + L2 --------------------------------------------------------------------
    terrain_field::TerrainField field;
    // Overhead block band = (ground+maxStepHeight, ground+robotHeight): structure above
    // the climbable-step clearance but at/below the robot body height blocks the cell.
    // A ceiling higher than robotHeight clears the robot (essential for multi-level:
    // a deck above another with clearance > robotHeight must NOT wipe the lower deck).
    field.compute(input, params, config.maxStepHeight, config.robotHeight);

    // --- L3 + L4: robot lens per (cell, layer), emit the TravGenNode graph ----------
    const terrain_field::RobotModel robot = robotModelFromConfig(config);
    const std::vector<double> diskOffsets = robot.diskOffsets();
    const double r = robot.radius();
    const double halfDiag = robot.halfDiagonal();

    terrain_field::CostWeights costWeights;
    costWeights.costFunctionDist = config.costFunctionDist;

    // nodes per cell for the connection pass
    std::vector<std::vector<TravGenNode*>> nodesAt(w * h);
    std::vector<Eigen::Vector2d> obstacles;

    for (std::size_t y = 0; y < h; ++y)
    {
        for (std::size_t x = 0; x < w; ++x)
        {
            const std::size_t i = y * w + x;
            const Eigen::Vector2d center((x + 0.5) * params.gridResolution,
                                         (y + 0.5) * params.gridResolution);

            for (const terrain_field::TerrainCell& tc : field.terrain().cells[i])
            {
                const terrain_field::TerrainField::LayerField* lf =
                    field.layerField(tc.layerId);
                if (!lf)
                    continue;

                TravGenNode* node = new TravGenNode(tc.height, Index(x, y));
                auto& data = node->getUserData();
                data.id = currentNodeId++;

                // plane in node-local xy / absolute z, as the legacy code expects
                const Eigen::Vector3d normal = tc.normal.cast<double>();
                data.plane = Eigen::Hyperplane<double, 3>(
                    normal, Eigen::Vector3d(0, 0, tc.height));
                data.slope = computeSlope(data.plane);
                data.slopeDirection = computeSlopeDirection(data.plane);
                data.slopeDirectionAtan2 =
                    std::atan2(data.slopeDirection.y(), data.slopeDirection.x());
                data.cost = (int)std::lround(
                    100.0 * terrain_field::terrainCost(tc, lf->esdf[i], costWeights));

                // ---- classification -------------------------------------------------
                if (tc.confidence < config.minTraversablePercentage)
                {
                    node->setType(TraversabilityNodeBase::UNKNOWN);
                    data.nodeType = NodeType::UNKNOWN;
                }
                else if (data.slope > config.maxSlope)
                {
                    node->setType(TraversabilityNodeBase::OBSTACLE);
                    data.nodeType = NodeType::OBSTACLE;
                }
                else if (lf->blocked[i])
                {
                    // structure intrudes into the body band at this very cell
                    node->setType(TraversabilityNodeBase::OBSTACLE);
                    data.nodeType = NodeType::OBSTACLE;
                }
                else if (std::min((double)lf->esdf[i],
                                  distToMapBorder(center, w, h, params.gridResolution)) < r)
                {
                    // the inscribed footprint does not fit at any heading
                    // (structure too close, or the footprint leaves the map)
                    node->setType(TraversabilityNodeBase::OBSTACLE);
                    data.nodeType = NodeType::INFLATED_OBSTACLE;
                }
                else
                {
                    const double borderDist =
                        distToMapBorder(center, w, h, params.gridResolution);
                    terrain_field::AngleIntervalSet clearance;
                    if (lf->esdf[i] >= halfDiag && borderDist >= halfDiag)
                    {
                        clearance = terrain_field::AngleIntervalSet::full();
                    }
                    else
                    {
                        field.collectObstaclesNear(tc.layerId, center, halfDiag + r,
                                                   obstacles);
                        appendBorderObstacles(center, halfDiag + r, w, h,
                                              params.gridResolution, obstacles);
                        clearance = terrain_field::feasibleHeadings(
                            center, diskOffsets, r, obstacles);
                    }

                    const terrain_field::AngleIntervalSet attitude =
                        terrain_field::attitudeHeadings(tc.normal, robot);
                    const terrain_field::AngleIntervalSet allowed =
                        clearance.intersect(attitude);

                    if (allowed.isEmpty())
                    {
                        node->setType(TraversabilityNodeBase::OBSTACLE);
                        data.nodeType = NodeType::INFLATED_OBSTACLE;
                    }
                    else if (allowed.isFull())
                    {
                        node->setType(TraversabilityNodeBase::TRAVERSABLE);
                        data.nodeType = NodeType::TRAVERSABLE;
                        setAllowedOrientations(node, allowed);
                    }
                    else
                    {
                        // drivable at a restricted heading set
                        node->setType(TraversabilityNodeBase::TRAVERSABLE);
                        data.nodeType = NodeType::PARTIALLY_TRAVERSABLE;
                        setAllowedOrientations(node, allowed);
                    }
                }

                node->setExpanded();
                trMap.at(Index(x, y)).insert(node);
                nodesAt[i].push_back(node);
            }
        }
    }

    // --- connections + frontier marking ---------------------------------------------
    static const std::vector<Index> surrounding = {
        Index(1, 1),  Index(1, 0),  Index(1, -1), Index(0, 1),
        Index(0, -1), Index(-1, 1), Index(-1, 0), Index(-1, -1)};

    for (std::size_t y = 0; y < h; ++y)
    {
        for (std::size_t x = 0; x < w; ++x)
        {
            const std::size_t i = y * w + x;
            for (TravGenNode* node : nodesAt[i])
            {
                bool bordersVoid = false;
                for (const Index& d : surrounding)
                {
                    const Index nIdx(Index(x, y) + d);
                    if (nIdx.x() < 0 || nIdx.y() < 0 || nIdx.x() >= (int)w ||
                        nIdx.y() >= (int)h)
                        continue;
                    const std::size_t j = (std::size_t)nIdx.y() * w + nIdx.x();
                    bool reachableNeighbor = false;
                    for (TravGenNode* nb : nodesAt[j])
                    {
                        if (std::fabs(nb->getHeight() - node->getHeight()) <=
                            config.maxStepHeight)
                        {
                            node->addConnection(nb);
                            reachableNeighbor = true;
                        }
                    }
                    if (!reachableNeighbor)
                        bordersVoid = true;
                }

                // legacy semantics: traversable but bordering missing map info
                if (bordersVoid &&
                    node->getUserData().nodeType == NodeType::TRAVERSABLE)
                {
                    node->setType(TraversabilityNodeBase::FRONTIER);
                    node->getUserData().nodeType = NodeType::FRONTIER;
                }
            }
        }
    }

    terrainFieldGenerated = true;

    const double dt = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - tStart)
                          .count();
    LOG_INFO_S << "TerrainField: generated " << currentNodeId << " nodes from "
               << patchCount << " MLS patches on a " << w << "x" << h
               << " grid in " << dt << "s";
}

}
