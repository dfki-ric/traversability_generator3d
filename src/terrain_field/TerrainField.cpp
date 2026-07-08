#include "TerrainField.hpp"
#include "Esdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace traversability_generator3d
{
namespace terrain_field
{

namespace
{
/** STRUCTURE triage predicate — must mirror TerrainEstimation (§5.2). */
bool isStructurePatch(const PatchSample& p, const TerrainParams& params)
{
    return std::fabs(p.normal.z()) < params.structureNormalZ ||
           (p.zMax - p.zMin) > 2.0 * params.maxStepHeight;
}
}

void TerrainField::compute(const TerrainGridInput& in, const TerrainParams& params,
                           double climbClearance, double bodyHeight)
{
    mResolution = params.gridResolution;
    mLayers.clear();

    estimateTerrain(in, params, mTerrain);

    const std::size_t w = mTerrain.width;
    const std::size_t h = mTerrain.height;
    const std::size_t n = w * h;

    // collect the layer ids present
    std::set<uint16_t> layerIds;
    for (const auto& cellLayers : mTerrain.cells)
        for (const TerrainCell& tc : cellLayers)
            layerIds.insert(tc.layerId);

    for (const uint16_t layerId : layerIds)
    {
        LayerField& field = mLayers[layerId];
        field.blocked.assign(n, 1); // void by default
        field.overhead.assign(n, std::numeric_limits<float>::infinity());

        for (std::size_t i = 0; i < n; ++i)
        {
            const TerrainCell* tc = nullptr;
            for (const TerrainCell& c : mTerrain.cells[i])
            {
                if (c.layerId == layerId)
                {
                    tc = &c;
                    break;
                }
            }
            if (!tc)
                continue; // stays blocked: no ground on this layer here

            const double ground = tc->height;
            // Overhead block band above the ground surface: from the climbable-step
            // clearance up to the robot body height. A ceiling AT the body height is
            // the limiting case (blocks); anything higher clears the robot.
            const double bandLo = ground + climbClearance;
            const double bandHi = ground + bodyHeight;
            const bool bandValid = bandHi > bandLo;

            bool intruded = false;
            float overhead = std::numeric_limits<float>::infinity();

            // vertical STRUCTURE patches intruding into the body band
            for (const PatchSample& p : in.cells[i])
            {
                if (!isStructurePatch(p, params))
                    continue;
                if (bandValid && p.zMax > bandLo && p.zMin < bandHi)
                    intruded = true;
                if (p.zMin > ground)
                    overhead = std::min(overhead, (float)(p.zMin - ground));
            }

            // other layers' ground intruding into the band (ceilings, decks above)
            for (const TerrainCell& other : mTerrain.cells[i])
            {
                if (other.layerId == layerId)
                    continue;
                if (bandValid && other.height > bandLo && other.height < bandHi)
                    intruded = true;
                if (other.height > ground)
                    overhead = std::min(overhead, (float)(other.height - ground));
            }

            field.overhead[i] = overhead;
            field.blocked[i] = intruded ? 1 : 0;
        }

        computeEsdf(w, h, field.blocked, mResolution, field.esdf);
    }
}

const TerrainField::LayerField* TerrainField::layerField(uint16_t layerId) const
{
    const auto it = mLayers.find(layerId);
    return it == mLayers.end() ? nullptr : &it->second;
}

void TerrainField::collectObstaclesNear(uint16_t layerId, const Eigen::Vector2d& p,
                                        double radius,
                                        std::vector<Eigen::Vector2d>& out) const
{
    out.clear();
    const auto it = mLayers.find(layerId);
    if (it == mLayers.end() || mResolution <= 0.0)
        return;
    const LayerField& field = it->second;

    const std::size_t w = mTerrain.width;
    const std::size_t h = mTerrain.height;
    // window of cells whose centers can lie within radius (+ half-cell slack)
    const double slack = radius + mResolution;
    const long x0 = std::max(0L, (long)std::floor((p.x() - slack) / mResolution));
    const long x1 = std::min((long)w - 1, (long)std::ceil((p.x() + slack) / mResolution));
    const long y0 = std::max(0L, (long)std::floor((p.y() - slack) / mResolution));
    const long y1 = std::min((long)h - 1, (long)std::ceil((p.y() + slack) / mResolution));

    const double r2 = radius * radius;
    for (long y = y0; y <= y1; ++y)
    {
        for (long x = x0; x <= x1; ++x)
        {
            if (!field.blocked[(std::size_t)y * w + x])
                continue;
            const Eigen::Vector2d c((x + 0.5) * mResolution, (y + 0.5) * mResolution);
            if ((c - p).squaredNorm() <= r2)
                out.push_back(c);
        }
    }
}

}
}
