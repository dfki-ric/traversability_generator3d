#include "TraversabilityGenerator3dGui.hpp"
#include <QFileDialog>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <base-logging/Logging.hpp>

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
    startViz.setPluginName("Start Pose");

    widget->setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);
    widget->addPlugin(&mlsViz);
    widget->addPlugin(&trav3dViz);
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
    
    resetButton = new QPushButton("Reset Traversability Map");
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

void TraversabilityGenerator3dGui::setupTravGen(int argc, char** argv)
{
    double res = 0.3;
    if(argc > 2){
        std::setlocale(LC_ALL, "C");
        res = atof(argv[2]);
    }

    travConfig.gridResolution = res;
    travConfig.maxSlope = 0.45;
    travConfig.maxStepHeight = 0.25;
    travConfig.robotSizeX = 0.5;
    travConfig.robotSizeY = 0.5;
    travConfig.robotHeight = 0.5;
    travConfig.slopeMetricScale = 1.0;
    travConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    travConfig.inclineLimittingMinSlope = 0.22;
    travConfig.inclineLimittingLimit = 0.43;
    travConfig.costFunctionDist = 0.0;
    travConfig.distToGround = 0.0;
    travConfig.minTraversablePercentage = 0.5;
    travConfig.allowForwardDownhill = true;
    travConfig.enableInclineLimitting = false;

    if (travConfig.gridResolution >= std::min(travConfig.robotSizeX, travConfig.robotSizeY)) {
        LOG_WARN_S << "Grid resolution (" << travConfig.gridResolution
                  << " m) is greater than or equal to the robot footprint size ("
                  << std::min(travConfig.robotSizeX, travConfig.robotSizeY)
                  << " m). Results may be inaccurate. Please use a smaller resolution.";
    }

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
        LOG_INFO_S << "Loading PLY";
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
        return;
    }
    try
    {
        LOG_INFO_S << "Loading MLS";
        boost::archive::binary_iarchive mlsIn(fileIn);
        mlsIn >> mlsMap;
        mlsViz.updateMLSSloped(mlsMap);

        std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
        travGen->setMLSGrid(mlsPtr);    

        resetButton->setEnabled(true);
        return;
    }
    catch(...) {}
    std::cerr << "Unable to load mls. Unknown format";
}

void TraversabilityGenerator3dGui::picked(float x, float y, float z, int buttonMask, int modifierMask)
{   
    if(buttonMask == 1) // left click
    {
        start.position << x, y, z;
        start.position.z() += travConfig.distToGround;

#ifdef ENABLE_DEBUG_DRAWINGS
        V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
        V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_start_aabb", start.position + base::Vector3d(0, 0, travConfig.distToGround / 2.0), start.orientation,
                           base::Vector3d(travConfig.robotSizeX, travConfig.robotSizeY, travConfig.robotHeight - travConfig.distToGround), V3DD::Color::cyan);
#endif
        QVector3D pos(start.position.x(), start.position.y(), start.position.z());
        startViz.setTranslation(pos);
        LOG_INFO_S << "Start: " << start.position.transpose();
        startPicked = true;
        expandAll();
    }
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
    trav3dViz.updateData(travGen->getTraversabilityMap());
}

void TraversabilityGenerator3dGui::resetTravMap()
{
    LOG_INFO_S << "Resetting Traversability Map";

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

