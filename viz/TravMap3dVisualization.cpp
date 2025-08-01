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
#include "TravMap3dVisualization.hpp"
#include <vizkit3d/Vizkit3DHelper.hpp>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include "PatchesGeode.hpp"
#include <iostream>
using namespace traversability_generator3d;
using namespace ::maps::grid;
using namespace vizkit3d;

vizkit3d::TravMap3dVisualization::TravMap3dVisualization()
    : MapVisualization< maps::grid::TraversabilityMap3d< traversability_generator3d::TravGenNode* > >()
    , isoline_interval(16.0)
    , show_connections(false)
{

}

vizkit3d::TravMap3dVisualization::~TravMap3dVisualization()
{

}

osg::ref_ptr< osg::Node > vizkit3d::TravMap3dVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = MapVisualization::createMainNode()->asGroup();
    localNode = new osg::Group();

    mainNode->addChild(localNode.get());

    return mainNode;
}


void vizkit3d::TravMap3dVisualization::updateDataIntern(const maps::grid::TraversabilityMap3d< TravGenNode* >& data)
{
    map = data;
}

template <typename T>
T clampE(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void TravMap3dVisualization::visualizeNode(const TravGenNode* node)
{

    Eigen::Vector2f curNodePos = (node->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();

    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setPosition(curNodePos.x(), curNodePos.y());

    if (!node) {
        LOG_ERROR_S << "Failed due to null node!";
        return;
    }

    const TravGenTrackingData& nodeData = node->getUserData();
    osg::Vec4d color;
    Eigen::Vector4d eColor;

    if (nodeData.nodeType == 0) {
        eColor = Eigen::Vector4d(1,0,0,1);
    }

    else if (nodeData.nodeType == 1) {
        eColor = Eigen::Vector4d(0,1,0,1);
    }

    else if (nodeData.nodeType == 2) {
        eColor = Eigen::Vector4d(0,0,1,1);
    }

    else if (nodeData.nodeType == 3) {
        eColor = Eigen::Vector4d(1, 0.647, 0, 1);
    }

    else if (nodeData.nodeType == 4) {
        eColor = Eigen::Vector4d(0.529, 0.808, 0.922, 1);
    }

    else if (nodeData.nodeType == 5) {
        eColor = Eigen::Vector4d(1,0,1,1);
    }

    else if (nodeData.nodeType == 6) {
        eColor = Eigen::Vector4d(1,1,1,1);
    }

    else if (nodeData.nodeType == 7) {
        eColor = Eigen::Vector4d(1,1,0,1);
    }

    else {
        eColor = Eigen::Vector4d(0,0,0,1);
    }

    color[0] = eColor[0];
    color[1] = eColor[1];
    color[2] = eColor[2];
    color[3] = eColor[3];

    geode->setColor(color);
    
    if(fabs(node->getHeight()) > 30000)
    {
        //FIXME if nodes are too far aways, the culling mechanism of osg breaks.
        // I.e. verticves disappear when zooming in on them.
        LOG_WARN_S << "TravMap3dVisualization:: Ignoring node with height above +-30000 "; 
        return;
    }
    
    if(std::isnan(node->getHeight()))
        throw std::runtime_error("FOOOOOOOO");
    
    geode->drawHorizontalPlane(node->getHeight());

}

void TravMap3dVisualization::visualizeConnection(const TravGenNode* from, const TravGenNode* to)
{
    Eigen::Vector2f toPos = (to->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    Eigen::Vector2f fromPos = (from->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    osg::Vec3 fromOsg(fromPos.x(), fromPos.y(), from->getHeight());
    osg::Vec3 toOsg(toPos.x(), toPos.y(), to->getHeight());
    
    linesNode->addLine(fromOsg, toOsg);
}

void vizkit3d::TravMap3dVisualization::addNodeList(const maps::grid::LevelList< TravGenNode* >& l, osg::Group* group)
{
    for(const auto &entry: l)
    {
        visualizeNode(entry);
    }
}

void vizkit3d::TravMap3dVisualization::updateMainNode(osg::Node* node)
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
            const LevelList<TravGenNode *> &l(map.at(x, y));
            addNodeList(l, localNode);
        }
    }
}

void vizkit3d::TravMap3dVisualization::setIsolineInterval(const double& val)
{
    isoline_interval = val;
    emit propertyChanged("isoline_interval");
    setDirty();
}

bool TravMap3dVisualization::getShowConnections()
{
    return show_connections;
}

void TravMap3dVisualization::setShowConnections(bool val)
{
    show_connections = val;
    emit propertyChanged("show_connections");
    setDirty();
}





