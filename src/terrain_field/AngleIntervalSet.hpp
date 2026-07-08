#pragma once

// Circular interval arithmetic on [0, 2*pi). Foundation of the exact orientation
// reasoning in the TerrainField robot lens (see TERRAIN_FIELD_ARCHITECTURE.md §5.4/5.5).
//
// Canonical form: sorted, disjoint, non-wrapping arcs [start, end] with
// 0 <= start < end <= 2*pi. A wrapping arc is stored split at 2*pi; membership,
// complement and intersection remain exact under the split representation.

#include <utility>
#include <vector>

namespace traversability_generator3d
{
namespace terrain_field
{

class AngleIntervalSet
{
public:
    /** The full circle. */
    static AngleIntervalSet full();
    /** The empty set. */
    static AngleIntervalSet empty();

    /** Add (union) an arc starting at @p start (any radians, normalized internally)
     *  spanning @p width radians. width >= 2*pi makes the set full; width <= 0 is a
     *  no-op. */
    void add(double start, double width);

    AngleIntervalSet complement() const;
    AngleIntervalSet intersect(const AngleIntervalSet& other) const;
    AngleIntervalSet unite(const AngleIntervalSet& other) const;

    /** Membership of @p angle (any radians, normalized internally). Closed arcs:
     *  endpoints are inside. */
    bool contains(double angle) const;

    bool isEmpty() const;
    bool isFull() const;

    /** Sum of arc widths (0 .. 2*pi). */
    double totalWidth() const;

    /** Canonical arcs as (start, end), sorted, disjoint, non-wrapping,
     *  within [0, 2*pi]. */
    const std::vector<std::pair<double, double>>& arcs() const { return mArcs; }

private:
    void normalizeAndMerge();

    std::vector<std::pair<double, double>> mArcs;
};

}
}
