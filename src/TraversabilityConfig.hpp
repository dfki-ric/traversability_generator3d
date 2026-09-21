#pragma once

namespace traversability_generator3d
{

/**Different metrics can be used to factor in the slope of a motion  */
enum SlopeMetric
{
    
    AVG_SLOPE,
    MAX_SLOPE,
    TRIANGLE_SLOPE,
    NONE
};

class TraversabilityConfig
{
public:

    TraversabilityConfig()
        : maxStepHeight(0.05)
        , maxSlope(0.5)
        , inclineLimittingMinSlope(0.2)
        , inclineLimittingLimit(0.1)
        , costFunctionDist(0.0)
        , minTraversablePercentage(0.4)
        , robotHeight(0.5)
        , robotSizeX(0.5)
        , robotSizeY(0.5)
        , footprintOffsetX(0.0)
        , distToGround(0)
        , slopeMetricScale(1.0)
        , slopeMetric(NONE)
        , gridResolution(0.3)
        , initialPatchVariance(0.01 * 0.01)
        , obstacleInflationMultiplier(1.0)

        , partiallyTraversableMultiplier(2.0)
        , allowForwardDownhill(true)
        , enableInclineLimitting(false)
        , useSoilInformation(false)
        , traverseSand(true)
        , traverseRocks(true)
        , traverseGravel(true)
        , traverseConcrete(true)
        , articulatedSuspension(true)
        , numYawSamples(12)
        , numThreads(0)
    {};

    /** The maximum step height that the robot can traverse.
     *  This is used during map expansion. Steps heigher than this become map boundaries */
    double maxStepHeight;

    /**[rad] maximum traversable slope. Above this slope no travmap entries will be generated*/
    double maxSlope;

    /** ---- incline limitting -----
     * The orientation at which the path crosses the incline of the terrain is limited. The steeper the terrain, the less is the path allowed to deviate from the steepest direction. I.e. the steeper it gets, the more
     * straight the path becomes.
     * */
    double inclineLimittingMinSlope;//[rad] below this slope the robot may move freely, the incline is not limited.
    /** [rad] At maxSlope the robot's movement direction may only deviate by +-inclineLimittingLimit
     * from the direction of the steepest slope */
    double inclineLimittingLimit;

    /**Objects within a corridor of width costFunctionDist around a trajectory will influence the cost function. */
    double costFunctionDist;

    /**
     * This value controls, how unknown patches are detected.
     * If only a certain percentage of MSL patches are present,
     * on the surface of a traversability patch, it is rated
     * an unknown patch.
     * */
    double minTraversablePercentage;

    //dimensions of the robot bounding box.
    double robotHeight;
    double robotSizeX;
    double robotSizeY;

    /** Forward (x) offset of the footprint-box CENTER from the robot origin,
     *  in the robot frame. 0 keeps the historic behavior (box centered on the
     *  origin). A robot whose origin is not at its geometric center (e.g. a
     *  front tool) sets this so the box [offset - sizeX/2, offset + sizeX/2]
     *  matches the real machine instead of mirroring the larger side. */
    double footprintOffsetX;

    /* Distance from body frame to ground
     * start and goal position are expected in body frame
     */
    double distToGround;

    /** Defines how strong the slope is factored into the
     *  motion cost.*/
    double slopeMetricScale;

    /** which metric to use to factor in the slope of a motion */
    SlopeMetric slopeMetric;

    double gridResolution;

    /** The variance that "initial patches" should have.
     *  @see Planner::setInitialPatch() */
    double initialPatchVariance;

    /** Multiplier (0.0-1.0) for obstacle inflation radius calculation.
     *  Affects how much the obstacles are inflated around the robot footprint.
     *  Default is 0.5 (half the robot diagonal). Higher values = more conservative inflation. */
    double obstacleInflationMultiplier;

    // Cost multiplier for traversing a partially traversable cell.
    double partiallyTraversableMultiplier;

    /**if true the robot is allowed to drive downhill forward, otherwise
     * it has to drive downhill backwards */
    bool allowForwardDownhill;

    /** if true, incline limitting is enabled and the robot motion is restricted when moving on steep hills. */
    bool enableInclineLimitting;

    bool useSoilInformation;
    bool traverseSand;
    bool traverseRocks;
    bool traverseGravel;
    bool traverseConcrete;
    bool articulatedSuspension;

    /** Number of yaw samples tested over the full circle [0,360deg) when computing the allowed
     *  orientations of a partially traversable cell. This is the exact collision-check count per
     *  cell, independent of the footprint offset. Higher = finer angular resolution but slower
     *  map generation. Step = 360deg / numYawSamples (e.g. 12 -> 30deg). */
    int numYawSamples;

    /** Number of OpenMP threads used by the wave-parallel map expansion (applied at the start
     *  of expandAll). 0 = do NOT parallelize: the expansion runs single-threaded on the
     *  calling thread (no OpenMP worker team). N > 0 = use exactly N threads. The map is
     *  identical for every value by design. ugv_nav4d_ros2 overwrites this with its planner
     *  numThreads parameter so one knob controls planning AND map generation. */
    int numThreads;
};
}