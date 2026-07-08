#pragma once

// L1 terrain estimation (see TERRAIN_FIELD_ARCHITECTURE.md §5.1 / §5.2):
//   1) patch triage      — SUPPORT vs STRUCTURE; STRUCTURE never enters ground fits,
//   2) layer building    — region-growing SUPPORT patches over 4-connected cells on
//                          height continuity (same-cell patches never share a layer),
//   3) robust cell fit   — weighted LSQ plane per (cell, layer) over the 3x3
//                          neighborhood with one Tukey/MAD robust reweight.
//
// Dependency-light by design: std + Eigen only, testable standalone.

#include "Types.hpp"

#include <cstddef>
#include <vector>

namespace traversability_generator3d
{
namespace terrain_field
{

/** Input: dense grid of per-cell patch lists (row-major w*h). */
struct TerrainGridInput
{
    std::size_t width = 0, height = 0;
    std::vector<std::vector<PatchSample>> cells; // size width*height
};

/** Output: per-cell list of TerrainCell (one per layer present at that cell). */
struct TerrainGridOutput
{
    std::size_t width = 0, height = 0;
    std::vector<std::vector<TerrainCell>> cells;
};

/** Run triage + layer building + robust per-(cell, layer) fitting over @p in.
 *
 *  Per output cell: one TerrainCell per support layer present at that cell,
 *  sorted by layerId. Layer ids are grid-global and stable for a given input.
 *  Cells without own SUPPORT patches are filled as INTERPOLATED from the 3x3
 *  neighborhood when at least two neighbor cells carry the layer (bounded to
 *  one cell of interpolation by construction).
 *
 *  Cell (ix, iy) is located at ((ix + 0.5), (iy + 0.5)) * params.gridResolution;
 *  fitted heights are evaluated at the cell center. */
void estimateTerrain(const TerrainGridInput& in, const TerrainParams& params,
                     TerrainGridOutput& out);

}
}
