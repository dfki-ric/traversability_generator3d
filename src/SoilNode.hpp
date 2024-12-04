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
    double probSand, probConcrete, probGravel, probRocks;
    SoilType soilType;

    SoilData()
        : probSand(0.25), probConcrete(0.25),
          probGravel(0.25), probRocks(0.25),
          soilType(UNKNOWN) {}

    // Update probabilities for each terrain type
    bool updateProbabilities(double likelihoodSand, 
                             double likelihoodConcrete, 
                             double likelihoodGravel, 
                             double likelihoodRocks)
    {

        // Validate inputs
        if (likelihoodSand < 0 || likelihoodConcrete < 0 || 
            likelihoodGravel < 0 || likelihoodRocks < 0) {
            std::cerr << "Error: Negative likelihoods provided!\n";
            return false;
        }

        double evidence = likelihoodSand * probSand + 
                          likelihoodConcrete * probConcrete + 
                          likelihoodGravel * probGravel + 
                          likelihoodRocks * probRocks;
        
        if (evidence <= 1e-9) { // Prevent division by zero
            std::cerr << "Error: Evidence is too small or zero.\n";
            return false;
        }
            
        probSand = (likelihoodSand * probSand) / evidence;
        probConcrete = (likelihoodConcrete * probConcrete) / evidence;
        probGravel = (likelihoodGravel * probGravel) / evidence;
        probRocks = (likelihoodRocks * probRocks) / evidence;
        return true;
    }

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & probSand;
        ar & probConcrete;
        ar & probGravel;
        ar & probRocks;
        ar & soilType;
    }
};

typedef maps::grid::TraversabilityNode<SoilData> SoilNode;
typedef maps::grid::TraversabilityMap3d<SoilNode *> SoilMap3d;
}
