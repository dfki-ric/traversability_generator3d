#pragma once
#include <base/Eigen.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <boost/serialization/serialization.hpp>
#include <base/Angle.hpp>
#include <cstdint>

namespace boost::serialization{
    template<class Archive>
    inline void serialize(Archive & ar, base::AngleSegment & segment, const unsigned int version){
        ar & segment.width;
        ar & segment.startRad;
        ar & segment.endRad;
    }
}

namespace traversability_generator3d
{

enum NodeType
{
    OBSTACLE = 0,
    TRAVERSABLE,
    FRONTIER,
    INFLATED_OBSTACLE,
    INFLATED_FRONTIER,
    UNKNOWN,
    HOLE,
    UNSET,
    PARTIALLY_TRAVERSABLE
};

/** WHY a node became OBSTACLE — for debugging/visualization (the planner only
 *  reads the node type). Set wherever a node is typed OBSTACLE. */
enum class ObstacleCause : uint8_t
{
    NONE = 0,        ///< node is not an obstacle
    UNMEASURED,      ///< ground-plane fit failed (no/too sparse data)
    STEEP_SLOPE,     ///< fitted slope > maxSlope
    STEP_HEIGHT,     ///< patch collides with the robot body volume
    INCLINE_LIMIT,   ///< no allowed heading under incline limitting
    NO_SAFE_YAW,     ///< obstacle inflation found no collision-free yaw
    MAP_BOUNDARY     ///< robot body volume leaves the mapped grid
};

/**Node struct for TraversabilityMap3d */
struct TravGenTrackingData
{
    /** The plane that has been fitted to the mls at the location of this node */
    Eigen::Hyperplane<double, 3> plane;
    
    /** slope of the plane */
    double slope = 0.0;
    
    /** normalized direction of the slope. Only valid if slope > 0 */
    Eigen::Vector3d slopeDirection = Eigen::Vector3d::Zero();
    
    /** The atan2(slopeDirection.y(), slopeDirection.x()), i.e. angle of slopeDirection projected on the xy plane.
     * Precomputed for performance reasons */
    double slopeDirectionAtan2 = 0.0; 
    
    /** continuous unique id  that can be used as index for additional metadata */
    size_t id = 0; 

    /**Some orientations might be forbidden on this patch (e.g. due to slope). This vector
     * contains all orientations that are allowed */
    std::vector<base::AngleSegment> allowedOrientations;
    
    NodeType nodeType = NodeType::UNSET;

    /** Why this node is OBSTACLE (NONE otherwise); debugging/visualization only. */
    ObstacleCause obstacleCause = ObstacleCause::NONE;

    int cost = 0;

    /** Serializes the members of this class*/
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & plane.offset();
        ar & plane.normal().x();
        ar & plane.normal().y();
        ar & plane.normal().z();
        ar & slope;
        ar & slopeDirection.x();
        ar & slopeDirection.y();
        ar & slopeDirection.z();
        ar & slopeDirectionAtan2;
        ar & id;
        ar & allowedOrientations;
        ar & nodeType;
        ar & obstacleCause;
        ar & cost;
    }
};

// Inline operator<< to print NodeType as a string
inline std::ostream& operator<<(std::ostream& os, NodeType type)
{
    switch (type)
    {
        case NodeType::OBSTACLE: os << "OBSTACLE"; break;
        case NodeType::TRAVERSABLE: os << "TRAVERSABLE"; break;
        case NodeType::FRONTIER: os << "FRONTIER"; break;
        case NodeType::INFLATED_OBSTACLE: os << "INFLATED_OBSTACLE"; break;
        case NodeType::INFLATED_FRONTIER: os << "INFLATED_FRONTIER"; break;
        case NodeType::UNKNOWN: os << "UNKNOWN"; break;
        case NodeType::HOLE: os << "HOLE"; break;
        case NodeType::UNSET: os << "UNSET"; break;
        case NodeType::PARTIALLY_TRAVERSABLE: os << "PARTIALLY_TRAVERSABLE"; break;
        default: os << "INVALID_NODE_TYPE"; break;
    }
    return os;
}

// Inline operator== for NodeType
inline bool operator==(NodeType lhs, NodeType rhs)
{
    return static_cast<int>(lhs) == static_cast<int>(rhs);
}

// Inline operator== for NodeType
inline bool operator==(NodeType lhs, int rhs)
{
    return (static_cast<int>(lhs) == rhs);
}

typedef maps::grid::TraversabilityNode<TravGenTrackingData> TravGenNode;
typedef maps::grid::TraversabilityMap3d<TravGenNode *> TravMap3d;

}
