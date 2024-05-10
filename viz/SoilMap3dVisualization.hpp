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
#pragma once
#include <vizkit3d/MapVisualization.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>
#include <osgViz/modules/viz/Primitives/Primitives/LinesNode.h>

#if QT_VERSION >= 0x050000 || !defined(Q_MOC_RUN)
    #include <maps/grid/TraversabilityMap3d.hpp>
#endif

#include <traversability_generator3d/TravGenNode.hpp>


namespace vizkit3d
{

class SoilMap3dVisualization
    : public vizkit3d::MapVisualization<::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *>>
{
    Q_OBJECT

    Q_PROPERTY(bool showMapExtents READ areMapExtentsShown WRITE setShowMapExtents)
    Q_PROPERTY(double isoline_interval READ getIsolineInterval WRITE setIsolineInterval)
    Q_PROPERTY(bool show_connections READ getShowConnections WRITE setShowConnections)

protected:
    virtual void updateDataIntern(const ::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *>& data);
    virtual void updateMainNode(osg::Node* node);

    virtual osg::ref_ptr< osg::Node > createMainNode();

    ::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *> map;

    void addNodeList(const ::maps::grid::LevelList<::maps::grid::TraversabilityNodeBase *> &l, osg::Group* group);

    void visualizeNode(const ::maps::grid::TraversabilityNodeBase *node);
    void visualizeConnection(const ::maps::grid::TraversabilityNodeBase *from, const ::maps::grid::TraversabilityNodeBase *to);

    osg::ref_ptr<osg::Geode> nodeGeode;
    osg::ref_ptr<osgviz::LinesNode> linesNode;

    osg::Group* connectionGroup;

    double isoline_interval;
    bool show_connections;

public:
    SoilMap3dVisualization();
    virtual ~SoilMap3dVisualization();

    Q_INVOKABLE void updateData(::maps::grid::TraversabilityBaseMap3d const &sample)
    {
        vizkit3d::Vizkit3DPlugin<::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *>>::updateData(sample);
    }

    Q_INVOKABLE void updateTrMap(::maps::grid::TraversabilityBaseMap3d const &sample)
    {
        vizkit3d::Vizkit3DPlugin<::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *>>::updateData(sample);
    }

    double getIsolineInterval() const { return isoline_interval; }
    void setIsolineInterval(const double& val);

    bool getShowConnections();
    void setShowConnections(bool val);

private:
    osg::ref_ptr<osg::Group> localNode;
};

}
