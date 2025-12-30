#pragma once
#include <QObject>
#include <QWidget>
#include <QDoubleSpinBox>
#include <atomic> 
#include <vector>
#include <string>

#ifndef Q_MOC_RUN
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TravMap3dVisualization.hpp>
#include <vizkit3d/SoilMap3dVisualization.hpp>
#include <vizkit3d/GridVisualization.hpp>
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/Eigen.hpp>
#include <traversability_generator3d/TraversabilityConfig.hpp>
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>
#include <traversability_generator3d/SoilNode.hpp>
#endif

class QPushButton;
class QDoubleSpinBox;
class QComboBox;
class QLabel;

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
    void resetTravMap();

private:
    void loadMls(const std::string& path);
    void loadTravConfigFromYaml(const std::string& path);
    std::vector<traversability_generator3d::SoilSample> soilSamplesList;
    void applySoilInformationToGenerator();

private:
    QPushButton* resetButton;
    vizkit3d::Vizkit3DWidget* widget;
    QDoubleSpinBox* time;
    QWidget window;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TravMap3dVisualization trav3dViz;
    vizkit3d::SoilMap3dVisualization soil3dViz;

    vizkit3d::RigidBodyStateVisualization startViz;
    vizkit3d::GridVisualization gridViz;
    maps::grid::MLSMapSloped mlsMap;
    base::Pose start;
    bool pickStart = true;
    bool startPicked = false;
    traversability_generator3d::TraversabilityConfig travConfig;
    std::shared_ptr<traversability_generator3d::TraversabilityGenerator3d> travGen;

    QComboBox* soilTypeCombo = nullptr;
    QDoubleSpinBox* sigmaXSpin = nullptr;
    QDoubleSpinBox* sigmaYSpin = nullptr;
    QDoubleSpinBox* uncertaintySpin = nullptr;
    QLabel* soilHintLabel = nullptr;

};
