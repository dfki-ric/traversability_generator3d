//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include "SoilMap3dVisualization.hpp"
#include <vizkit3d/Vizkit3DHelper.hpp>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include "PatchesGeode.hpp"

using namespace traversability_generator3d;
using namespace ::maps::grid;
using namespace vizkit3d;

vizkit3d::SoilMap3dVisualization::SoilMap3dVisualization()
    : MapVisualization< maps::grid::TraversabilityMap3d< traversability_generator3d::SoilNode* > >()
    , isoline_interval(16.0)
    , show_connections(false)
{

}

vizkit3d::SoilMap3dVisualization::~SoilMap3dVisualization()
{

}

osg::ref_ptr< osg::Node > vizkit3d::SoilMap3dVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = MapVisualization::createMainNode()->asGroup();
    localNode = new osg::Group();

    mainNode->addChild(localNode.get());

    return mainNode;
}


void vizkit3d::SoilMap3dVisualization::updateDataIntern(const maps::grid::TraversabilityMap3d< SoilNode* >& data)
{
    map = data;
}

void setSoilColor(const osg::Vec4d& color, osg::Geode* geode)
{
    osg::Material *material = new osg::Material();
    material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.6, 0.6, 0.6, 1.0));
    material->setAmbient(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setEmission(osg::Material::FRONT, color);
    material->setShininess(osg::Material::FRONT, 10.0);

    geode->getOrCreateStateSet()->setAttribute(material);    
}

template <typename T>
T clampE(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

Eigen::Vector4d getSandColor(double prob) {
    prob = clampE(prob, 0.0, 1.0);
    double intensity = pow(prob, 2) * 0.6 + 0.3;  // Exponential growth
    return Eigen::Vector4d(intensity, intensity * 0.85, intensity * 0.5, 1.0);
}

Eigen::Vector4d getConcreteColor(double prob) {
    prob = clampE(prob, 0.0, 1.0);
    double intensity = pow(prob, 1.5) * 0.7 + 0.2;  // Exponential growth f
    return Eigen::Vector4d(intensity * 0.9, intensity * 0.9, intensity, 1.0);
}

Eigen::Vector4d getGravelColor(double prob) {
    prob = clampE(prob, 0.0, 1.0);  // Clamp probability
    double intensity = pow(prob, 2.5) * 0.4 + 0.3;  // Exponential growth for stronger contrast
    return Eigen::Vector4d(intensity * 0.3, intensity * 0.7, intensity * 0.4, 1.0);  // Green tones
}

Eigen::Vector4d getRockColor(double prob) {
    prob = clampE(prob, 0.0, 1.0);  // Clamp probability
    double intensity = pow(prob, 1.8) * 0.5 + 0.3;  // Exponential increase for stronger change
    return Eigen::Vector4d(intensity * 0.8, intensity * 0.4, intensity * 0.3, 1.0);  // Red tones
}

void SoilMap3dVisualization::visualizeNode(const SoilNode* node)
{
    Eigen::Vector2f curNodePos = (node->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();

    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setPosition(curNodePos.x(), curNodePos.y());

    LOG_INFO_S << "Node type: " << typeid(*node).name();


    const SoilNode* soilNode = static_cast<const SoilNode*>(node);
    if (!soilNode) {
        LOG_ERROR_S << "Failed to cast node to SoilNode!";
        return;
    }

    const SoilData& soilData = soilNode->getUserData();
    std::cout << "probSand=" << soilData.probSand 
            << ", probConcrete=" << soilData.probConcrete 
            << ", probGravel=" << soilData.probGravel 
            << ", probRocks=" << soilData.probRocks
            << ", Type= " << soilNode->getUserData().soilType << "\n";

    double probSand = soilNode->getUserData().probSand;
    double probConcrete = soilNode->getUserData().probConcrete;
    double probGravel = soilNode->getUserData().probGravel;
    double probRocks = soilNode->getUserData().probRocks;

    osg::Vec4d color;
    Eigen::Vector4d eColor;

    if (soilNode->getUserData().soilType == -1) {
        eColor = Eigen::Vector4d(0,1,1,1);
    }

    if (soilNode->getUserData().soilType == 0) {
        eColor = getConcreteColor(probConcrete);
    }

    if (soilNode->getUserData().soilType == 1) {
        eColor = getRockColor(probRocks);
    }

    if (soilNode->getUserData().soilType == 2) {
        eColor = getSandColor(probSand);
    }

    if (soilNode->getUserData().soilType == 3) {
        eColor = getGravelColor(probGravel);
    }

    color[0] = eColor[0];
    color[1] = eColor[1];
    color[2] = eColor[2];
    color[3] = eColor[3];

    geode->setColor(color);
    
    if(fabs(soilNode->getHeight()) > 30000)
    {
        //FIXME if nodes are too far aways, the culling mechanism of osg breaks.
        // I.e. verticves disappear when zooming in on them.
        LOG_WARN_S << "SoilMap3dVisualization:: Ignoring node with height above +-30000 "; 
        return;
    }
    
    if(std::isnan(soilNode->getHeight()))
        throw std::runtime_error("FOOOOOOOO");
    
    geode->drawHorizontalPlane(soilNode->getHeight());

}

void SoilMap3dVisualization::visualizeConnection(const SoilNode* from, const SoilNode* to)
{
    Eigen::Vector2f toPos = (to->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    Eigen::Vector2f fromPos = (from->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    osg::Vec3 fromOsg(fromPos.x(), fromPos.y(), from->getHeight());
    osg::Vec3 toOsg(toPos.x(), toPos.y(), to->getHeight());
    
    linesNode->addLine(fromOsg, toOsg);
}

void vizkit3d::SoilMap3dVisualization::addNodeList(const maps::grid::LevelList< SoilNode* >& l, osg::Group* group)
{
    for(const auto &entry: l)
    {
        visualizeNode(entry);
    }
}

void vizkit3d::SoilMap3dVisualization::updateMainNode(osg::Node* node)
{
    // Apply local frame.
    setLocalFrame(map.getLocalFrame());

    // Draw map extents.
    visualizeMapExtents(map.calculateCellExtents(), map.getResolution());

    //clear old data
    localNode->removeChildren(0, localNode->getNumChildren());

    nodeGeode = new PatchesGeode(map.getResolution().x(), map.getResolution().y());
    linesNode = new osgviz::LinesNode(osg::Vec4(1, 1, 1, 1));
    localNode->addChild(nodeGeode);
    localNode->addChild(linesNode);
    
    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setColor(osg::Vec4d(1,0,0,1));
    geode->setShowPatchExtents(true);
    geode->setShowNormals(true);
    
    for(size_t y = 0; y < map.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < map.getNumCells().x(); x++)
        {
            const LevelList<SoilNode *> &l(map.at(x, y));
            addNodeList(l, localNode);
        }
    }
}

void vizkit3d::SoilMap3dVisualization::setIsolineInterval(const double& val)
{
    isoline_interval = val;
    emit propertyChanged("isoline_interval");
    setDirty();
}

bool SoilMap3dVisualization::getShowConnections()
{
    return show_connections;
}

void SoilMap3dVisualization::setShowConnections(bool val)
{
    show_connections = val;
    emit propertyChanged("show_connections");
    setDirty();
}





