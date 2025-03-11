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
        : probSand(PRIOR_PROB), 
          probConcrete(PRIOR_PROB),
          probGravel(PRIOR_PROB), 
          probRocks(PRIOR_PROB),
          soilType(SoilType::UNKNOWN_SOIL) {}

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

        if (probSand > probConcrete && probSand > probGravel && probSand > probRocks)
            soilType = SoilType::SAND;
        else if (probConcrete > probSand && probConcrete > probGravel && probConcrete > probRocks)
            soilType = SoilType::CONCRETE;
        else if (probGravel > probSand && probGravel > probConcrete && probGravel > probRocks)
            soilType = SoilType::GRAVEL;
        else
            soilType = SoilType::ROCKS;

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
