#pragma once

// Exact 2D Euclidean distance transform for the L2 structure field (see
// TERRAIN_FIELD_ARCHITECTURE.md §5.3). Felzenszwalb & Huttenlocher two-pass
// separable squared-distance transform (1D lower envelope of parabolas per
// column, then per row). O(w*h), exact for integer grid offsets.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace traversability_generator3d
{
namespace terrain_field
{

/** Exact squared Euclidean distance (in cell units) to the nearest blocked cell.
 *  blocked is row-major w*h; out resized to w*h; cells with no blocked cell anywhere
 *  get std::numeric_limits<float>::infinity(). */
void computeEsdfSquared(std::size_t w, std::size_t h,
                        const std::vector<std::uint8_t>& blocked,
                        std::vector<float>& outSquared);

/** Euclidean distance in meters: sqrt(squared) * resolution. */
void computeEsdf(std::size_t w, std::size_t h,
                 const std::vector<std::uint8_t>& blocked,
                 double resolution, std::vector<float>& outMeters);

}
}
