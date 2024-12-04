#pragma once

#include <base/Eigen.hpp>

namespace traversability_generator3d
{

enum SoilType
{
    UNKNOWN = -1,
    CONCRETE,
    ROCKS,
    SAND,
    GRAVEL
};

// Serialize the enum as an integer
template<class Archive>
void serialize(Archive & ar, SoilType & type, const unsigned int version) {
    ar & reinterpret_cast<int&>(type);
}

class SoilSample
{
public:
    
    SoilSample(): type(UNKNOWN), 
                  sigmaX(1), 
                  sigmaY(1), 
                  location(0,0,0){};

    base::Vector3d location;  
    SoilType type;
    double sigmaX,sigmaY;

    bool isValid() const {
        return sigmaX > 0 && sigmaY > 0;
    }

    bool operator==(const SoilSample& other) const {
        return location.isApprox(other.location) &&
            type == other.type &&
            sigmaX == other.sigmaX &&
            sigmaY == other.sigmaY;
    }

};
}