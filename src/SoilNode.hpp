#pragma once
#include <base/Eigen.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <boost/serialization/serialization.hpp>

#include "SoilSample.hpp"

namespace traversability_generator3d
{
const float PRIOR_PROB = 0.25;

/**Node struct for TraversabilityMap3d */
struct SoilData
{
    double probSand = PRIOR_PROB;
    double probConcrete = PRIOR_PROB;
    double probGravel = PRIOR_PROB;
    double probRocks = PRIOR_PROB;

    // Update probabilities for each terrain type
    void updateProbabilities(double likelihoodSand, 
                             double likelihoodConcrete, 
                             double likelihoodGravel, 
                             double likelihoodRocks)
    {

        double evidence = likelihoodSand * probSand + 
                          likelihoodConcrete * probConcrete + 
                          likelihoodGravel * probGravel + 
                          likelihoodRocks * probRocks;
        
        if (evidence == 0){
            return;
        }
        
        probSand = (likelihoodSand * probSand) / evidence;
        probConcrete = (likelihoodConcrete * probConcrete) / evidence;
        probGravel = (likelihoodGravel * probGravel) / evidence;
        probRocks = (likelihoodRocks * probRocks) / evidence;
    }

    //TODO:
    //Based on the probabilities decide on a single soil type which will be used 
    //in the planning phase
    SoilType type;
};

typedef maps::grid::TraversabilityNode<SoilData> SoilNode;
}
