#include "TraversabilityGenerator3dGui.hpp"
#include <QFileDialog>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QGridLayout>
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
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

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

    QHBoxLayout* yawLayout = new QHBoxLayout();
    yawSpin_ = new QDoubleSpinBox();
    yawSpin_->setRange(-180.0, 180.0);
    yawSpin_->setValue(0.0);
    yawSpin_->setSingleStep(5.0);
    yawSpin_->setSuffix("\xc2\xb0");  // degree sign
    yawSpin_->setToolTip("Rotate the robot bounding box preview around its vertical axis");
    yawLayout->addWidget(new QLabel("Robot yaw:"));
    yawLayout->addWidget(yawSpin_);
    yawLayout->addStretch();
    layout->addLayout(yawLayout);

    // --- Config Parameters panel ---
    QGridLayout* paramLayout = new QGridLayout();

    paramLayout->addWidget(new QLabel("Grid Res:"), 0, 0);
    gridResolutionSpin = new QDoubleSpinBox();
    gridResolutionSpin->setRange(0.05, 10.0);
    gridResolutionSpin->setSingleStep(0.05);
    paramLayout->addWidget(gridResolutionSpin, 0, 1);

    paramLayout->addWidget(new QLabel("Robot Height:"), 0, 2);
    robotHeightSpin = new QDoubleSpinBox();
    robotHeightSpin->setRange(0.1, 10.0);
    robotHeightSpin->setSingleStep(0.05);
    paramLayout->addWidget(robotHeightSpin, 0, 3);

    paramLayout->addWidget(new QLabel("Robot Size X:"), 0, 4);
    robotSizeXSpin = new QDoubleSpinBox();
    robotSizeXSpin->setRange(0.1, 20.0);
    robotSizeXSpin->setSingleStep(0.1);
    paramLayout->addWidget(robotSizeXSpin, 0, 5);

    paramLayout->addWidget(new QLabel("Robot Size Y:"), 0, 6);
    robotSizeYSpin = new QDoubleSpinBox();
    robotSizeYSpin->setRange(0.1, 20.0);
    robotSizeYSpin->setSingleStep(0.1);
    paramLayout->addWidget(robotSizeYSpin, 0, 7);

    paramLayout->addWidget(new QLabel("Max Step Height:"), 1, 0);
    maxStepHeightSpin = new QDoubleSpinBox();
    maxStepHeightSpin->setRange(0.0, 5.0);
    maxStepHeightSpin->setSingleStep(0.05);
    paramLayout->addWidget(maxStepHeightSpin, 1, 1);

    paramLayout->addWidget(new QLabel("Max Slope [rad]:"), 1, 2);
    maxSlopeSpin = new QDoubleSpinBox();
    maxSlopeSpin->setRange(0.0, 1.57);
    maxSlopeSpin->setSingleStep(0.05);
    paramLayout->addWidget(maxSlopeSpin, 1, 3);

    paramLayout->addWidget(new QLabel("Obstacle Infl Mult:"), 1, 4);
    obstacleInflationMultiplierSpin = new QDoubleSpinBox();
    obstacleInflationMultiplierSpin->setRange(1.0, 10.0);
    obstacleInflationMultiplierSpin->setSingleStep(0.1);
    paramLayout->addWidget(obstacleInflationMultiplierSpin, 1, 5);

    paramLayout->addWidget(new QLabel("Dist to Ground:"), 1, 6);
    distToGroundSpin = new QDoubleSpinBox();
    distToGroundSpin->setRange(-5.0, 5.0);
    distToGroundSpin->setSingleStep(0.05);
    paramLayout->addWidget(distToGroundSpin, 1, 7);

    paramLayout->addWidget(new QLabel("Inc Lim Min Slope:"), 2, 0);
    inclineLimittingMinSlopeSpin = new QDoubleSpinBox();
    inclineLimittingMinSlopeSpin->setRange(0.0, 1.57);
    inclineLimittingMinSlopeSpin->setSingleStep(0.05);
    paramLayout->addWidget(inclineLimittingMinSlopeSpin, 2, 1);

    paramLayout->addWidget(new QLabel("Inc Lim Limit:"), 2, 2);
    inclineLimittingLimitSpin = new QDoubleSpinBox();
    inclineLimittingLimitSpin->setRange(0.0, 1.57);
    inclineLimittingLimitSpin->setSingleStep(0.05);
    paramLayout->addWidget(inclineLimittingLimitSpin, 2, 3);

    paramLayout->addWidget(new QLabel("Cost Func Dist:"), 2, 4);
    costFunctionDistSpin = new QDoubleSpinBox();
    costFunctionDistSpin->setRange(0.0, 50.0);
    costFunctionDistSpin->setSingleStep(0.5);
    paramLayout->addWidget(costFunctionDistSpin, 2, 5);

    paramLayout->addWidget(new QLabel("Min Trav %:"), 2, 6);
    minTraversablePercentageSpin = new QDoubleSpinBox();
    minTraversablePercentageSpin->setRange(0.0, 1.0);
    minTraversablePercentageSpin->setSingleStep(0.05);
    paramLayout->addWidget(minTraversablePercentageSpin, 2, 7);

    paramLayout->addWidget(new QLabel("Slope Metric:"), 3, 0);
    slopeMetricCombo = new QComboBox();
    slopeMetricCombo->addItem("NONE");
    slopeMetricCombo->addItem("AVG_SLOPE");
    slopeMetricCombo->addItem("MAX_SLOPE");
    slopeMetricCombo->addItem("TRIANGLE_SLOPE");
    paramLayout->addWidget(slopeMetricCombo, 3, 1);

    allowForwardDownhillCheck = new QCheckBox("Allow Forward Downhill");
    paramLayout->addWidget(allowForwardDownhillCheck, 3, 2, 1, 2);

    enableInclineLimittingCheck = new QCheckBox("Enable Incline Limiting");
    paramLayout->addWidget(enableInclineLimittingCheck, 3, 4, 1, 2);

    useSoilInformationCheck = new QCheckBox("Use Soil Info");
    paramLayout->addWidget(useSoilInformationCheck, 3, 6, 1, 2);

    traverseSandCheck = new QCheckBox("Traverse Sand");
    paramLayout->addWidget(traverseSandCheck, 4, 0, 1, 2);

    traverseRocksCheck = new QCheckBox("Traverse Rocks");
    paramLayout->addWidget(traverseRocksCheck, 4, 2, 1, 2);

    traverseGravelCheck = new QCheckBox("Traverse Gravel");
    paramLayout->addWidget(traverseGravelCheck, 4, 4, 1, 2);

    traverseConcreteCheck = new QCheckBox("Traverse Concrete");
    paramLayout->addWidget(traverseConcreteCheck, 4, 6, 1, 2);

    layout->addLayout(paramLayout);

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
    connect(resetButton, SIGNAL(clicked(bool)), this, SLOT(resetTravMap()));
    connect(yawSpin_, SIGNAL(valueChanged(double)), this, SLOT(onYawChanged(double)));

    connect(gridResolutionSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(robotHeightSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(robotSizeXSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(robotSizeYSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(distToGroundSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(maxSlopeSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(maxStepHeightSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(inclineLimittingMinSlopeSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(inclineLimittingLimitSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(costFunctionDistSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(minTraversablePercentageSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(obstacleInflationMultiplierSpin, SIGNAL(valueChanged(double)), this, SLOT(updateConfigFromUI()));
    connect(slopeMetricCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateConfigFromUI()));
    connect(allowForwardDownhillCheck, SIGNAL(toggled(bool)), this, SLOT(updateConfigFromUI()));
    connect(enableInclineLimittingCheck, SIGNAL(toggled(bool)), this, SLOT(updateConfigFromUI()));
    connect(useSoilInformationCheck, SIGNAL(toggled(bool)), this, SLOT(updateConfigFromUI()));
    connect(traverseSandCheck, SIGNAL(toggled(bool)), this, SLOT(updateConfigFromUI()));
    connect(traverseRocksCheck, SIGNAL(toggled(bool)), this, SLOT(updateConfigFromUI()));
    connect(traverseGravelCheck, SIGNAL(toggled(bool)), this, SLOT(updateConfigFromUI()));
    connect(traverseConcreteCheck, SIGNAL(toggled(bool)), this, SLOT(updateConfigFromUI()));
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
    
    travConfig.obstacleInflationMultiplier = cfg["obstacleInflationMultiplier"] ? cfg["obstacleInflationMultiplier"].as<double>() : travConfig.obstacleInflationMultiplier;

    // enum
    std::string s = cfg["slopeMetric"].as<std::string>();
    if     (s == "AVG_SLOPE")      travConfig.slopeMetric = traversability_generator3d::AVG_SLOPE;
    else if(s == "MAX_SLOPE")      travConfig.slopeMetric = traversability_generator3d::MAX_SLOPE;
    else if(s == "TRIANGLE_SLOPE") travConfig.slopeMetric = traversability_generator3d::TRIANGLE_SLOPE;
    else                            travConfig.slopeMetric = traversability_generator3d::NONE;

    // Update GUI widgets with loaded values
    gridResolutionSpin->blockSignals(true);
    gridResolutionSpin->setValue(travConfig.gridResolution);
    gridResolutionSpin->blockSignals(false);

    robotHeightSpin->blockSignals(true);
    robotHeightSpin->setValue(travConfig.robotHeight);
    robotHeightSpin->blockSignals(false);

    robotSizeXSpin->blockSignals(true);
    robotSizeXSpin->setValue(travConfig.robotSizeX);
    robotSizeXSpin->blockSignals(false);

    robotSizeYSpin->blockSignals(true);
    robotSizeYSpin->setValue(travConfig.robotSizeY);
    robotSizeYSpin->blockSignals(false);

    distToGroundSpin->blockSignals(true);
    distToGroundSpin->setValue(travConfig.distToGround);
    distToGroundSpin->blockSignals(false);

    maxSlopeSpin->blockSignals(true);
    maxSlopeSpin->setValue(travConfig.maxSlope);
    maxSlopeSpin->blockSignals(false);

    maxStepHeightSpin->blockSignals(true);
    maxStepHeightSpin->setValue(travConfig.maxStepHeight);
    maxStepHeightSpin->blockSignals(false);

    inclineLimittingMinSlopeSpin->blockSignals(true);
    inclineLimittingMinSlopeSpin->setValue(travConfig.inclineLimittingMinSlope);
    inclineLimittingMinSlopeSpin->blockSignals(false);

    inclineLimittingLimitSpin->blockSignals(true);
    inclineLimittingLimitSpin->setValue(travConfig.inclineLimittingLimit);
    inclineLimittingLimitSpin->blockSignals(false);

    costFunctionDistSpin->blockSignals(true);
    costFunctionDistSpin->setValue(travConfig.costFunctionDist);
    costFunctionDistSpin->blockSignals(false);

    minTraversablePercentageSpin->blockSignals(true);
    minTraversablePercentageSpin->setValue(travConfig.minTraversablePercentage);
    minTraversablePercentageSpin->blockSignals(false);

    obstacleInflationMultiplierSpin->blockSignals(true);
    obstacleInflationMultiplierSpin->setValue(travConfig.obstacleInflationMultiplier);
    obstacleInflationMultiplierSpin->blockSignals(false);

    slopeMetricCombo->blockSignals(true);
    if      (travConfig.slopeMetric == traversability_generator3d::AVG_SLOPE)      slopeMetricCombo->setCurrentText("AVG_SLOPE");
    else if (travConfig.slopeMetric == traversability_generator3d::MAX_SLOPE)      slopeMetricCombo->setCurrentText("MAX_SLOPE");
    else if (travConfig.slopeMetric == traversability_generator3d::TRIANGLE_SLOPE) slopeMetricCombo->setCurrentText("TRIANGLE_SLOPE");
    else                                                                           slopeMetricCombo->setCurrentText("NONE");
    slopeMetricCombo->blockSignals(false);

    allowForwardDownhillCheck->blockSignals(true);
    allowForwardDownhillCheck->setChecked(travConfig.allowForwardDownhill);
    allowForwardDownhillCheck->blockSignals(false);

    enableInclineLimittingCheck->blockSignals(true);
    enableInclineLimittingCheck->setChecked(travConfig.enableInclineLimitting);
    enableInclineLimittingCheck->blockSignals(false);

    useSoilInformationCheck->blockSignals(true);
    useSoilInformationCheck->setChecked(travConfig.useSoilInformation);
    useSoilInformationCheck->blockSignals(false);

    traverseSandCheck->blockSignals(true);
    traverseSandCheck->setChecked(travConfig.traverseSand);
    traverseSandCheck->blockSignals(false);

    traverseRocksCheck->blockSignals(true);
    traverseRocksCheck->setChecked(travConfig.traverseRocks);
    traverseRocksCheck->blockSignals(false);

    traverseGravelCheck->blockSignals(true);
    traverseGravelCheck->setChecked(travConfig.traverseGravel);
    traverseGravelCheck->blockSignals(false);

    traverseConcreteCheck->blockSignals(true);
    traverseConcreteCheck->setChecked(travConfig.traverseConcrete);
    traverseConcreteCheck->blockSignals(false);
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
    setupRobotBoxViz();
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
            cfg.gapSize = 0.5;
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
        start.position + base::Vector3d(0, 0, (travConfig.robotHeight - travConfig.distToGround) / 2.0),
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
    updateRobotBoxTransform();
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

    if (robotBoxTransform_)
        robotBoxTransform_->setNodeMask(0);

#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
#endif

    LOG_INFO_S << "Traversability map reset complete.";
}

void TraversabilityGenerator3dGui::setupRobotBoxViz()
{
    // Wireframe box matching the robot footprint; hidden until the user clicks.
    osg::ref_ptr<osg::Box> osgBox = new osg::Box(
        osg::Vec3(0.0f, 0.0f, static_cast<float>(travConfig.robotHeight / 2.0)),
        static_cast<float>(travConfig.robotSizeX),
        static_cast<float>(travConfig.robotSizeY),
        static_cast<float>(travConfig.robotHeight));

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(osgBox);
    sd->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // red wireframe

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(sd);

    osg::ref_ptr<osg::StateSet> ss = geode->getOrCreateStateSet();
    osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
    pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    ss->setAttribute(pm, osg::StateAttribute::ON);
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    robotBoxTransform_ = new osg::MatrixTransform;
    robotBoxTransform_->addChild(geode);
    robotBoxTransform_->setNodeMask(0);  // hidden until first click
    widget->getRootNode()->addChild(robotBoxTransform_);
}

void TraversabilityGenerator3dGui::updateRobotBoxTransform()
{
    if (!robotBoxTransform_ || !startPicked)
        return;

    const double yawRad = yawSpin_->value() * M_PI / 180.0;

    // Update the startViz arrow to reflect the chosen heading
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()));
    start.orientation = q;
    startViz.setRotation(QQuaternion(
        static_cast<float>(q.w()),
        static_cast<float>(q.x()),
        static_cast<float>(q.y()),
        static_cast<float>(q.z())));

    // Rotate around Z then translate to the picked position
    const osg::Matrix mat =
        osg::Matrix::rotate(yawRad, osg::Vec3d(0.0, 0.0, 1.0)) *
        osg::Matrix::translate(start.position.x(), start.position.y(), start.position.z());

    robotBoxTransform_->setMatrix(mat);
    robotBoxTransform_->setNodeMask(~0u);
}

void TraversabilityGenerator3dGui::onYawChanged(double /*deg*/)
{
    updateRobotBoxTransform();
}

void TraversabilityGenerator3dGui::updateConfigFromUI()
{
    // doubles
    travConfig.gridResolution           = gridResolutionSpin->value();
    travConfig.robotHeight              = robotHeightSpin->value();
    travConfig.robotSizeX               = robotSizeXSpin->value();
    travConfig.robotSizeY               = robotSizeYSpin->value();
    travConfig.distToGround             = distToGroundSpin->value();
    travConfig.maxSlope                 = maxSlopeSpin->value();
    travConfig.maxStepHeight            = maxStepHeightSpin->value();
    travConfig.inclineLimittingMinSlope = inclineLimittingMinSlopeSpin->value();
    travConfig.inclineLimittingLimit    = inclineLimittingLimitSpin->value();
    travConfig.costFunctionDist         = costFunctionDistSpin->value();
    travConfig.minTraversablePercentage = minTraversablePercentageSpin->value();
    travConfig.obstacleInflationMultiplier = obstacleInflationMultiplierSpin->value();

    // booleans
    travConfig.allowForwardDownhill     = allowForwardDownhillCheck->isChecked();
    travConfig.enableInclineLimitting   = enableInclineLimittingCheck->isChecked();
    travConfig.useSoilInformation       = useSoilInformationCheck->isChecked();
    travConfig.traverseSand             = traverseSandCheck->isChecked();
    travConfig.traverseRocks            = traverseRocksCheck->isChecked();
    travConfig.traverseGravel           = traverseGravelCheck->isChecked();
    travConfig.traverseConcrete         = traverseConcreteCheck->isChecked();

    // enum
    std::string s = slopeMetricCombo->currentText().toStdString();
    if      (s == "AVG_SLOPE")      travConfig.slopeMetric = traversability_generator3d::AVG_SLOPE;
    else if (s == "MAX_SLOPE")      travConfig.slopeMetric = traversability_generator3d::MAX_SLOPE;
    else if (s == "TRIANGLE_SLOPE") travConfig.slopeMetric = traversability_generator3d::TRIANGLE_SLOPE;
    else                            travConfig.slopeMetric = traversability_generator3d::NONE;

    if (travGen)
    {
        travGen->setConfig(travConfig);
    }

    if (robotBoxTransform_ && widget && widget->getRootNode())
    {
        widget->getRootNode()->removeChild(robotBoxTransform_);
        robotBoxTransform_ = nullptr;
    }
    setupRobotBoxViz();
    if (startPicked)
    {
        updateRobotBoxTransform();
    }
}

