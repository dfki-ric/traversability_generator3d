#include "TraversabilityGenerator3dGui.hpp"
#include <QFileDialog>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <base-logging/Logging.hpp>
#include <fstream>
#include <traversability_generator3d/SoilNode.hpp>
#include <yaml-cpp/yaml.h>

#ifdef ENABLE_DEBUG_DRAWINGS
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#endif

using namespace traversability_generator3d;

TraversabilityGenerator3dGui::TraversabilityGenerator3dGui(int argc, char** argv) : QObject()
{
    setupUI();
    setupTravGen(argc, argv);
}

void TraversabilityGenerator3dGui::setupUI()
{
    start.orientation.setIdentity();    
    widget = new vizkit3d::Vizkit3DWidget();
#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(widget);
#endif
    trav3dViz.setPluginName("TraversabilityMap3d");
    soil3dViz.setPluginName("SoilMap3d");
    startViz.setPluginName("Start Pose");

    widget->setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);
    widget->addPlugin(&mlsViz);
    widget->addPlugin(&trav3dViz);
    widget->addPlugin(&soil3dViz);
    widget->addPlugin(&startViz);
    widget->addPlugin(&gridViz);
    
    gridViz.setPluginEnabled(false);
    gridViz.setPluginName("Grid");

    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false); 
    mlsViz.setShowNormals(false);
    mlsViz.setPluginName("MLSMap");

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(widget);
    
    QHBoxLayout* soilLayout = new QHBoxLayout();

    soilTypeCombo = new QComboBox();
    soilTypeCombo->addItem("No selection");   // index 0
    soilTypeCombo->addItem("CONCRETE");
    soilTypeCombo->addItem("ROCKS");
    soilTypeCombo->addItem("SAND");
    soilTypeCombo->addItem("GRAVEL");

    sigmaXSpin = new QDoubleSpinBox();
    sigmaXSpin->setRange(0.01, 100.0);
    sigmaXSpin->setValue(1.0);
    sigmaXSpin->setSingleStep(0.1);
    sigmaXSpin->setPrefix("σx=");

    sigmaYSpin = new QDoubleSpinBox();
    sigmaYSpin->setRange(0.01, 100.0);
    sigmaYSpin->setValue(1.0);
    sigmaYSpin->setSingleStep(0.1);
    sigmaYSpin->setPrefix("σy=");

    uncertaintySpin = new QDoubleSpinBox();
    uncertaintySpin->setRange(0.0, 1.0);
    uncertaintySpin->setValue(0.0);
    uncertaintySpin->setSingleStep(0.05);
    uncertaintySpin->setPrefix("u=");

    soilHintLabel = new QLabel("Click map to add soil sample");

    soilLayout->addWidget(new QLabel("Soil:"));
    soilLayout->addWidget(soilTypeCombo);
    soilLayout->addWidget(sigmaXSpin);
    soilLayout->addWidget(sigmaYSpin);
    soilLayout->addWidget(uncertaintySpin);

    layout->addLayout(soilLayout);
    layout->addWidget(soilHintLabel);

    resetButton = new QPushButton("Reset Traversability and Soil Maps");
    resetButton->setEnabled(false);
    layout->addWidget(resetButton);
    window.setLayout(layout);

    qRegisterMetaType<maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase*>>(
        "maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase*>"
    );

    connect(&mlsViz, SIGNAL(picked(float,float,float,int,int)), 
            this, SLOT(picked(float,float,float,int,int)));
    connect(&trav3dViz, SIGNAL(picked(float,float,float,int,int)), 
            this, SLOT(picked(float,float,float,int,int)));
    connect(resetButton, &QPushButton::clicked, this, &TraversabilityGenerator3dGui::resetTravMap);
}

void TraversabilityGenerator3dGui::loadTravConfigFromYaml(const std::string& file)
{
    YAML::Node cfg = YAML::LoadFile(file)["travConfig"];

    LOG_INFO_S << "Loaded Parameters: ";
    LOG_INFO_S << cfg;

    // doubles
    travConfig.maxStepHeight            = cfg["maxStepHeight"].as<double>();
    travConfig.maxSlope                 = cfg["maxSlope"].as<double>();
    travConfig.inclineLimittingMinSlope = cfg["inclineLimittingMinSlope"].as<double>();
    travConfig.inclineLimittingLimit    = cfg["inclineLimittingLimit"].as<double>();
    travConfig.costFunctionDist         = cfg["costFunctionDist"].as<double>();
    travConfig.minTraversablePercentage = cfg["minTraversablePercentage"].as<double>();
    travConfig.robotHeight              = cfg["robotHeight"].as<double>();
    travConfig.robotSizeX               = cfg["robotSizeX"].as<double>();
    travConfig.robotSizeY               = cfg["robotSizeY"].as<double>();
    travConfig.distToGround             = cfg["distToGround"].as<double>();
    travConfig.slopeMetricScale         = cfg["slopeMetricScale"].as<double>();
    travConfig.gridResolution           = cfg["gridResolution"].as<double>();
    travConfig.initialPatchVariance     = cfg["initialPatchVariance"] ? cfg["initialPatchVariance"].as<double>() : travConfig.initialPatchVariance;

    // booleans
    travConfig.allowForwardDownhill     = cfg["allowForwardDownhill"].as<bool>();
    travConfig.enableInclineLimitting   = cfg["enableInclineLimitting"].as<bool>();

    travConfig.useSoilInformation       = cfg["useSoilInformation"].as<bool>();
    travConfig.traverseSand             = cfg["traverseSand"].as<bool>();
    travConfig.traverseRocks            = cfg["traverseRocks"].as<bool>();
    travConfig.traverseGravel           = cfg["traverseGravel"].as<bool>();
    travConfig.traverseConcrete         = cfg["traverseConcrete"].as<bool>();

    // enum
    std::string s = cfg["slopeMetric"].as<std::string>();
    if     (s == "AVG_SLOPE")      travConfig.slopeMetric = traversability_generator3d::AVG_SLOPE;
    else if(s == "MAX_SLOPE")      travConfig.slopeMetric = traversability_generator3d::MAX_SLOPE;
    else if(s == "TRIANGLE_SLOPE") travConfig.slopeMetric = traversability_generator3d::TRIANGLE_SLOPE;
    else                            travConfig.slopeMetric = traversability_generator3d::NONE;
}

static traversability_generator3d::SoilType soilTypeFromString(const std::string& s)
{
    if (s == "CONCRETE") return SoilType::CONCRETE;
    if (s == "ROCKS")    return SoilType::ROCKS;
    if (s == "SAND")     return SoilType::SAND;
    if (s == "GRAVEL")   return SoilType::GRAVEL;

    return SoilType::UNKNOWN_SOIL;
}

static bool soilTypeFromCombo(QComboBox* box, SoilType& outType)
{
    if (!box || box->currentIndex() == 0)
        return false; // No selection

    const std::string s = box->currentText().toStdString();
    outType = soilTypeFromString(s);
    return outType != SoilType::UNKNOWN_SOIL;
}

void TraversabilityGenerator3dGui::applySoilInformationToGenerator()
{
    if (!travGen)
        return;

    if (travConfig.useSoilInformation){
        for (const auto& s : soilSamplesList)
            travGen->addSoilNode(s);
        travGen->updateSoilInformation();
    }
    soilSamplesList.clear();
}

void TraversabilityGenerator3dGui::setupTravGen(int argc, char** argv)
{
    std::string yaml_file = std::string(TRAV3D_CONFIG_DIR) + "/parameters.yaml";
    loadTravConfigFromYaml(yaml_file);
    travGen.reset(new traversability_generator3d::TraversabilityGenerator3d(travConfig));

    if(argc > 1)
    {
        const std::string mls(argv[1]);
        loadMls(mls);
    }
}

void TraversabilityGenerator3dGui::loadMls(const std::string& path)
{
    std::ifstream fileIn(path);       

    if(path.find(".ply") != std::string::npos)
    {
        LOG_INFO_S << "Loading PLY: " << path;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ mi, ma; 
            pcl::getMinMax3D (*cloud, mi, ma); 
            LOG_INFO_S << "MIN: " << mi << ", MAX: " << ma;

            const double mls_res = travConfig.gridResolution;
            const double size_x = ma.x - mi.x;
            const double size_y = ma.y - mi.y;

            const maps::grid::Vector2ui numCells(size_x / mls_res + 2, size_y / mls_res + 2);
            LOG_INFO_S << "NUM CELLS: " << numCells.transpose();

            maps::grid::MLSConfig cfg;
            cfg.gapSize = 0.1;
            const maps::grid::Vector2d mapSize(numCells[0]*mls_res, numCells[1]*mls_res);
            const maps::grid::Vector3d offset(mi.x-0.5*mls_res, mi.y-0.5*mls_res, 0);

            mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mlsMap.translate(offset);
            mlsMap.mergePointCloud(*cloud, base::Transform3d::Identity());
            mlsViz.updateMLSSloped(mlsMap);

            std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
            travGen->setMLSGrid(mlsPtr);            

            resetButton->setEnabled(true);
        }
        else{
            LOG_ERROR_S << "Unable to load mls from PLY: " << path;
        }
    }
    else{
        try
        {
            LOG_INFO_S << "Loading MLS from Bin file: " << path;
            boost::archive::binary_iarchive mlsIn(fileIn);
            mlsIn >> mlsMap;
            mlsViz.updateMLSSloped(mlsMap);

            std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
            travGen->setMLSGrid(mlsPtr);    

            resetButton->setEnabled(true);
        }
        catch(...) {
            LOG_ERROR_S << "Unable to load mls. File is neither in PLY nor Bin format!";
        }
    }
    return;
}

void TraversabilityGenerator3dGui::picked(float x, float y, float z,
                                          int buttonMask, int /*modifierMask*/)
{
    if (buttonMask != 1) // left click only
        return;

    SoilType selectedType;
    if (soilTypeFromCombo(soilTypeCombo, selectedType))
    {
        SoilSample s;
        s.location = base::Vector3d(x, y, z);
        s.type = selectedType;
        s.sigmaX = sigmaXSpin->value();
        s.sigmaY = sigmaYSpin->value();
        s.uncertainty = uncertaintySpin->value();

        if (!s.isValid())
        {
            LOG_WARN_S << "Invalid soil sample — not added";
            return;
        }

        soilSamplesList.push_back(s);

        LOG_INFO_S << "Added soil sample: "
                   << "type=" << soilTypeCombo->currentText().toStdString()
                   << " loc=" << s.location.transpose()
                   << " σ=(" << s.sigmaX << "," << s.sigmaY << ")"
                   << " u=" << s.uncertainty;
    }

    start.position << x, y, z;
    start.position.z() += travConfig.distToGround;

#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
    V3DD::DRAW_WIREFRAME_BOX(
        "ugv_nav4d_start_aabb",
        start.position + base::Vector3d(0, 0, travConfig.distToGround / 2.0),
        start.orientation,
        base::Vector3d(travConfig.robotSizeX,
                       travConfig.robotSizeY,
                       travConfig.robotHeight - travConfig.distToGround),
        V3DD::Color::cyan);
#endif

    startViz.setTranslation(QVector3D(
        start.position.x(),
        start.position.y(),
        start.position.z()));

    startPicked = true;
    expandAll();
}


void TraversabilityGenerator3dGui::show()
{
    window.show();
}

void TraversabilityGenerator3dGui::expandAll()
{   
    if(!startPicked) {
        LOG_WARN_S << "Start pose not picked yet!";
        return;
    }
    travGen->expandAll(Eigen::Vector3d(start.position.x(), start.position.y(), start.position.z()));
    
    applySoilInformationToGenerator();
    
    trav3dViz.updateData(travGen->getTraversabilityMap());
    soil3dViz.updateData(travGen->getSoilMap());

    //Clear previous travmap to incorporate updated soil information in the next travmap expansion
    travGen->clearTrMap();
}

void TraversabilityGenerator3dGui::resetTravMap()
{
    LOG_INFO_S << "Resetting Maps";

    if(!travGen) {
        LOG_WARN_S << "Traversability generator not initialized!";
        return;
    }

    travGen.reset(new traversability_generator3d::TraversabilityGenerator3d(travConfig));

    if(mlsMap.getNumCells().x() > 0) {
        LOG_INFO_S << "Reassigning MLS map to new generator";
        auto mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
        travGen->setMLSGrid(mlsPtr);
    }

    // Clear visualization
    trav3dViz.updateData(traversability_generator3d::TravMap3d());
    soil3dViz.updateData(traversability_generator3d::SoilMap3d());

    soilSamplesList.clear();

    // Reset start pose state
    startPicked = false;
    start.position.setZero();
    start.orientation.setIdentity();
    startViz.setTranslation(QVector3D(0,0,0));

#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
#endif

    LOG_INFO_S << "Traversability map reset complete.";
}

