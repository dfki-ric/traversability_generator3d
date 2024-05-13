#pragma once
#include <base/Eigen.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <boost/serialization/serialization.hpp>

namespace traversability_generator3d
{

/**Node struct for TraversabilityMap3d */
struct SoilData
{

    /** The plane that has been fitted to the mls at the location of this node */
    Eigen::Hyperplane<double, 3> plane;

    /** continuous unique id  that can be used as index for additional metadata */
    size_t id; 

    Eigen::Vector3d location;
    int soil_type;

    /** Serializes the members of this class*/
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & plane.offset();
        ar & plane.normal().x();
        ar & plane.normal().y();
        ar & plane.normal().z();
        ar & id;
        ar & location.x();
        ar & location.y();
        ar & location.z();
        ar & soil_type;
    }

};
/*
template <class T>
class GroundNode : public maps::grid::TraversabilityNode<T>
{
public:

    enum GROUNDTYPE
    {
        CONCRETE,
        ROCK,
        SAND,
        GRAVEL
    };

    GroundNode(float height, const maps::grid::Index& idx) : 
        maps::grid::TraversabilityNode<T>(height, idx)
    {
    };

    void setGroundType(GROUNDTYPE t) ;
    GROUNDTYPE getGroundType() const ;

    enum GROUNDTYPE ground_type;
};

template<class T>
void GroundNode<T>::setGroundType(GroundNode<T>::GROUNDTYPE t)
{
    ground_type = t;
}

template<class T>
typename GroundNode<T>::GROUNDTYPE GroundNode<T>::getGroundType() const
{
    return ground_type;
}
*/
typedef maps::grid::TraversabilityNode<SoilData> SoilNode;
//typedef GroundNode<SoilData> SoilNode;
}
