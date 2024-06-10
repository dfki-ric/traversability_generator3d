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
    
    SoilSample()
        : center(0,0,0)
        , min(0,0,0)
        , max(0,0,0)
        , radius(0.0)
        , soilType(UNKNOWN)
        , sampleType(POINT)
    {};
    
    base::Vector3d center;
    base::Vector3d min;
    base::Vector3d max;
    double radius;
    SoilType soilType;
    SoilSampleType sampleType;
};
}