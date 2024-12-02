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
};
}