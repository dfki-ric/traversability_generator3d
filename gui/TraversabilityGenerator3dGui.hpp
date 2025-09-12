#pragma once
#include <QObject>
#include <QWidget>
#include <atomic> 

#ifndef Q_MOC_RUN
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TravMap3dVisualization.hpp>
#include <vizkit3d/GridVisualization.hpp>
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/Eigen.hpp>
#include <traversability_generator3d/TraversabilityConfig.hpp>
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>
#endif

class QPushButton;
class QDoubleSpinBox;

namespace vizkit3d {
    class Vizkit3DWidget;
}

class TraversabilityGenerator3dGui : public QObject
{
    Q_OBJECT
    
    void setupTravGen(int argc, char** argv);
    void setupUI();
    
public:
    TraversabilityGenerator3dGui(int argc, char** argv);
    
    void show();

public slots:
    void picked(float x, float y,float z, int buttonMask, int modifierMask);
    void expandAll();
    
private:
    void loadMls(const std::string& path);
    
private:
    QPushButton* expandButton;
    vizkit3d::Vizkit3DWidget* widget;
    QDoubleSpinBox* time;
    QWidget window;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TravMap3dVisualization trav3dViz;
    vizkit3d::RigidBodyStateVisualization startViz;
    vizkit3d::GridVisualization gridViz;
    maps::grid::MLSMapSloped mlsMap;
    base::Pose start;
    bool pickStart = true;
    bool startPicked = false;
    traversability_generator3d::TraversabilityConfig travConfig;
    std::shared_ptr<traversability_generator3d::TraversabilityGenerator3d> travGen;
};
