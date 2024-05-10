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
    : MapVisualization< maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > >()
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


void vizkit3d::SoilMap3dVisualization::updateDataIntern(const maps::grid::TraversabilityMap3d< TraversabilityNodeBase* >& data)
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

void SoilMap3dVisualization::visualizeNode(const TraversabilityNodeBase* node)
{
    Eigen::Vector2f curNodePos = (node->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();

    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setPosition(curNodePos.x(), curNodePos.y());

    //std::cout << "Node Type: " << node->getType() << std::endl;

    const SoilNode* soilNode = static_cast<const SoilNode*>(node);

    if (!soilNode){
        std::cout << "Failed to cast to SoilNode" << std::endl;
    }

    switch(node->getType())
    {
        case TraversabilityNodeBase::OBSTACLE:
            geode->setColor(osg::Vec4d(1,0,0,1));
            break;
        case TraversabilityNodeBase::UNKNOWN:
            geode->setColor(osg::Vec4d(1,0,1,1));
            break;
        case TraversabilityNodeBase::TRAVERSABLE:
            geode->setColor(osg::Vec4d(0, 1, 0, 1));
            break;
        case TraversabilityNodeBase::FRONTIER:
            geode->setColor(osg::Vec4d(0, 0, 1, 1));
            break;
        case TraversabilityNodeBase::HOLE:
            geode->setColor(osg::Vec4d(0, 1, 1, 1));
            break;
        case TraversabilityNodeBase::UNSET:
            geode->setColor(osg::Vec4d(1, 1, 0, 1));
            break;            
        default:
            LOG_WARN_S << "Unknown node type!";
            geode->setColor(osg::Vec4d(0,0,1,1));
    }

    //std::cout << "Soil Type: " << soilNode->getUserData().id << std::endl;
    /*
    switch(soilNode->getType())
    {
        case SoilNode::CONCRETE:
            geode->setColor(osg::Vec4d(0.5, 0.5, 0.5, 1.0));
            break;
        case SoilNode::ROCK:
            geode->setColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
            break;
        case SoilNode::SAND:
            geode->setColor(osg::Vec4d(0.647, 0.165, 0.165, 1.0));
            break;
        case SoilNode::GRAVEL:
            geode->setColor(osg::Vec4d(1.0, 1.0, 0.0, 1.0));
            break;
        default:
            LOG_WARN_S << "Unknown node type!";
            geode->setColor(osg::Vec4d(0,0,1,1));
    }
    */
    
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
    
    if(show_connections)
    {
        for(const TraversabilityNodeBase *conNode: soilNode->getConnections())
        {
            visualizeConnection(soilNode, conNode);
        }
    }
}

void SoilMap3dVisualization::visualizeConnection(const TraversabilityNodeBase* from, const TraversabilityNodeBase* to)
{
    Eigen::Vector2f toPos = (to->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    Eigen::Vector2f fromPos = (from->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    osg::Vec3 fromOsg(fromPos.x(), fromPos.y(), from->getHeight());
    osg::Vec3 toOsg(toPos.x(), toPos.y(), to->getHeight());
    
    linesNode->addLine(fromOsg, toOsg);
}

void vizkit3d::SoilMap3dVisualization::addNodeList(const maps::grid::LevelList< TraversabilityNodeBase* >& l, osg::Group* group)
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
            const LevelList<TraversabilityNodeBase *> &l(map.at(x, y));
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





