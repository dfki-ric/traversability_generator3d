#pragma once

#include <base/Eigen.hpp>

namespace traversability_generator3d
{

enum SoilSampleType
{
    POINT,
    CIRCLE,
    BOX,
};

class SoilSample
{
public:
    
    SoilSample()
        : center(0,0,0)
        , min(0,0,0)
        , max(0,0,0)
        , radius(0.0)
        , soilType(-1)
    {};
    
    base::Vector3d center;
    base::Vector3d min;
    base::Vector3d max;
    double radius;
    int soilType;
    SoilSampleType type;
};
}