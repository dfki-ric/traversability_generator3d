#pragma once

// L1+L2 orchestrator of the TerrainField pipeline (see TERRAIN_FIELD_ARCHITECTURE.md).
// Runs terrain estimation over a patch grid, then builds the per-layer structure
// fields (blocking mask, exact ESDF, overhead clearance). std + Eigen only.

#include "TerrainEstimation.hpp"
#include "Types.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace traversability_generator3d
{
namespace terrain_field
{

class TerrainField
{
public:
    /** Per-layer structure data (L2). All grids are row-major width*height. */
    struct LayerField
    {
        /** Blocking mask: true where the layer is not drivable — no ground on this
         *  layer (void/hole/other-layer), or vertical structure / another layer's
         *  underside intrudes into the body band above the ground. */
        std::vector<uint8_t> blocked;
        /** Exact 2D Euclidean distance to the nearest blocked cell [m]. */
        std::vector<float> esdf;
        /** Free vertical space above the ground surface [m] (+inf if unbounded). */
        std::vector<float> overhead;
    };

    /** Run L1 + L2.
     *  A cell is blocked by overhead structure when a patch (or another layer's
     *  surface) sits in the height band (ground + @p climbClearance, ground + @p
     *  bodyHeight) above it — i.e. high enough to not be a climbable step, but low
     *  enough that the robot body would hit it.
     *  @param climbClearance surmountable step clearance above ground; structure below
     *                        this is treated as ground, not overhead (legacy:
     *                        maxStepHeight) [m]
     *  @param bodyHeight     robot body height above ground; a ceiling at or below this
     *                        blocks the cell (legacy: robotHeight). Must be >
     *                        climbClearance to define a non-empty band. [m] */
    void compute(const TerrainGridInput& in, const TerrainParams& params,
                 double climbClearance, double bodyHeight);

    const TerrainGridOutput& terrain() const { return mTerrain; }

    /** Structure field of @p layerId, or nullptr if the layer does not exist. */
    const LayerField* layerField(uint16_t layerId) const;

    /** Collect blocked-cell centers of @p layerId within @p radius (meters, grid-local
     *  frame: cell (ix, iy) center = ((ix+0.5)*res, (iy+0.5)*res)) around @p p.
     *  Used to compute exact feasible-heading intervals in the maybe band. */
    void collectObstaclesNear(uint16_t layerId, const Eigen::Vector2d& p, double radius,
                              std::vector<Eigen::Vector2d>& out) const;

    std::size_t width() const { return mTerrain.width; }
    std::size_t height() const { return mTerrain.height; }
    double resolution() const { return mResolution; }

private:
    TerrainGridOutput mTerrain;
    std::unordered_map<uint16_t, LayerField> mLayers;
    double mResolution = 0.0;
};

}
}
