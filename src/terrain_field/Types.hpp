#pragma once

// TerrainField shared data model. See TERRAIN_FIELD_ARCHITECTURE.md.
// Modules in terrain_field/ are dependency-light (std + Eigen only) so they can be
// built and tested standalone, outside the ROCK build.

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <vector>

namespace traversability_generator3d
{
namespace terrain_field
{

/** One MLS surface patch, abstracted away from the maps library so L1 is testable
 *  standalone. Produced by the ingest (L0) from MLSMapSloped. */
struct PatchSample
{
    float mean;             //!< representative surface height [m]
    float zMin;             //!< lower extent [m]
    float zMax;             //!< upper extent [m]
    Eigen::Vector3f normal; //!< unit surface normal (z-up oriented)
    float variance;         //!< height variance of the patch [m^2]
};

/** L1 output: robust ground estimate for one (cell, layer). */
struct TerrainCell
{
    enum Flags : uint8_t
    {
        GROUND       = 1 << 0, //!< cell has own support measurements
        INTERPOLATED = 1 << 1, //!< filled from neighborhood only (bounded to 1 cell)
        SPARSE       = 1 << 2, //!< below-nominal measurement density
    };

    float height = 0.f;          //!< robust ground height [m]
    float heightSigma = 0.f;     //!< 1-sigma of the height estimate [m]
    Eigen::Vector3f normal{0.f, 0.f, 1.f}; //!< unit, layer-consistent, z-up
    float roughness = 0.f;       //!< weighted RMS fit residual [m]
    float confidence = 0.f;      //!< 0..1, density x agreement
    uint16_t layerId = 0;        //!< tile-local support-surface id
    uint8_t flags = 0;
};

/** L2 output: structure/clearance data for one (cell, layer). */
struct StructureCell
{
    float esdf = std::numeric_limits<float>::infinity();     //!< 2D distance to nearest
                                                             //!< blocking structure [m]
    float overhead = std::numeric_limits<float>::infinity(); //!< free space above ground [m]
    bool blocked = false;                                    //!< member of the blocking mask
};

/** Robot-independent parameters of the terrain field (L1/L2). */
struct TerrainParams
{
    double gridResolution = 0.5;   //!< [m]
    double maxStepHeight = 0.25;   //!< ground-continuity / step-obstacle threshold [m]
    double maxSlope = 0.45;        //!< [rad] (only used for cost/attitude, not L1)
    double structureNormalZ = 0.5; //!< patches with |normal.z| below this are STRUCTURE
    double heightBand = 2.0;       //!< deployment-max robot height for the blocking band [m]
    double sigma0 = 0.02;          //!< variance floor for fit weights [m]
    double neighborWeight = 0.6;   //!< spatial weight of 3x3 neighbors in the cell fit
};

/** Robot description consumed by the lens (L3) only — never baked into the map. */
struct RobotModel
{
    double sizeX = 1.0;        //!< body length [m]
    double sizeY = 1.0;        //!< body width [m]
    double height = 1.0;       //!< body height [m]
    double safetyMargin = 0.0; //!< added to the capsule radius [m]

    double maxSlope = 0.45;                 //!< [rad]
    bool enableInclineLimitting = false;
    double inclineLimittingMinSlope = 0.2;  //!< [rad]
    double inclineLimittingLimit = 0.1;     //!< [rad]
    bool allowForwardDownhill = true;
    bool articulated = false;               //!< attitude estimator variant

    /** Capsule disk radius. */
    double radius() const { return sizeY / 2.0 + safetyMargin; }

    /** Rotation-safe (circumscribed) radius. */
    double halfDiagonal() const
    {
        return std::sqrt(sizeX * sizeX + sizeY * sizeY) / 2.0 + safetyMargin;
    }

    /** Signed disk offsets along the body axis covering the length with the
     *  capsule radius. */
    std::vector<double> diskOffsets() const
    {
        const double r = radius();
        const double half = sizeX / 2.0;
        std::vector<double> offs;
        if (half <= r)
        {
            offs.push_back(0.0);
            return offs;
        }
        // end disks flush with the body ends; spacing <= r so the hull has no gaps
        const double span = half - r;               // outermost disk center offset
        const int perSide = std::max(1, (int)std::ceil(span / r));
        for (int i = -perSide; i <= perSide; ++i)
            offs.push_back(span * (double)i / (double)perSide);
        return offs;
    }
};

/** Weights of the continuous cost blend (L3). */
struct CostWeights
{
    double wSlope = 1.0;
    double wRough = 1.0;
    double wClear = 1.0;
    double wConf = 1.0;
    double costFunctionDist = 0.0; //!< clearance falloff distance [m]; <=0 disables
};

}
}
