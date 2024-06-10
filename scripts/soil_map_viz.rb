require 'vizkit'
require 'orocos'
require 'cmath'
require 'eigen'
include Orocos

Orocos.initialize
Orocos.load_typekit "base"
Orocos.load_typekit "ugv_nav4d"
#Orocos.load_typekit "traversability_generator3d"

if(ARGV.size > 0)
    sim = ARGV[0]
end

### Plugins for the visualization
urf_model_file                                      = "#{ENV['AUTOPROJ_CURRENT_ROOT']}/models/robots/artemis/urdf/artemis.urdf"

# Plugins
robot_visualization_current_state_plugin            = Vizkit.default_loader.RobotVisualization
robot_visualization_current_state_plugin.modelFile  = urf_model_file.dup
soil_map_plugin				                        = Vizkit.default_loader.TraversabilityMap3dVisualization
planned_trajectory_plugin                           = Vizkit.default_loader.TrajectoryVisualization

# load GUI created with the Qt Designer
widget = Vizkit.load "widget.ui"
widget.show

#Set frame
Vizkit.vizkit3d_widget.setPluginDataFrame("artemis", robot_visualization_current_state_plugin )
Vizkit.vizkit3d_widget.setPluginDataFrame("ground_truth", soil_map_plugin )
Vizkit.vizkit3d_widget.setPluginDataFrame("ground_truth", planned_trajectory_plugin )

### Configure plugins

# Planner trajectory2d
planned_trajectory_plugin.setColor(Eigen::Vector3.new(255, 0, 0))#Red line
planned_trajectory_plugin.setPluginName("Planned Robot Trajectory")

# Planner Traversability Map
soil_map_plugin.setPluginName("Soil Map")

#### ROCK
# ROCK Tasks
planner_task             = Orocos::Async.name_service.get "path_planner"
planner_task_ns          = Orocos.name_service.get 'path_planner'

# Connect ports to the plugins
planner_task.on_reachable do
    Vizkit.display planner_task.port("trajectory2D"), :widget =>planned_trajectory_plugin
    Vizkit.display planner_task.port("soil_map"), :widget =>soil_map_plugin
end

if (sim == "sim")
    bogie_dispatcher = Orocos::Async.name_service.get 'joint_dispatcher_sim'
    bogie_dispatcher.on_reachable do

        bogie_dispatcher.port('motion_status').on_data do |joints,_|
            robot_visualization_current_state_plugin.updateData(joints)
        end

    end
else
    bogie_dispatcher = Orocos::Async.name_service.get 'joint_dispatcher'
    bogie_dispatcher.on_reachable do

        bogie_dispatcher.port('motion_status').on_data do |joints,_|
            robot_visualization_current_state_plugin.updateData(joints)
        end

    end
end

#Task Ports
planner_port_soil_sample        = planner_task.port "soil_sample"
planner_port_soil_sample_writer = planner_port_soil_sample.writer

############################################################################################
#Visualization 

widget.btn_concrete.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()

    if (widget.listSampleTypes.currentItem.text == "POINT")
        sample.sampleType = 0
    end

    if (widget.listSampleTypes.currentItem.text == "CIRCLE")
        sample.sampleType = 1
    end

    if (widget.listSampleTypes.currentItem.text == "BOX")
        sample.sampleType = 2
    end

    sample.min = Eigen::Vector3.new(widget.min_x.value, widget.min_y.value, 0)
    sample.max = Eigen::Vector3.new(widget.max_x.value, widget.max_y.value, 0)

    sample.radius = widget.sb_radius.value
    sample.soilType = 0

    samples = planner_port_soil_sample_writer.new_sample
    samples.push(sample)
    planner_port_soil_sample_writer.write(samples)    
    puts "Concrete!"
end

widget.btn_rocks.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()

    if (widget.listSampleTypes.currentItem.text == "POINT")
        sample.sampleType = 0
    end

    if (widget.listSampleTypes.currentItem.text == "CIRCLE")
        sample.sampleType = 1
    end

    if (widget.listSampleTypes.currentItem.text == "BOX")
        sample.sampleType = 2
    end

    sample.min = Eigen::Vector3.new(widget.min_x.value, widget.min_y.value, 0)
    sample.max = Eigen::Vector3.new(widget.max_x.value, widget.max_y.value, 0)


    sample.radius = widget.sb_radius.value
    sample.soilType = 1

    samples = planner_port_soil_sample_writer.new_sample
    samples.push(sample)
    planner_port_soil_sample_writer.write(samples)    
    puts "Rocks!"
end

widget.btn_sand.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()

    if (widget.listSampleTypes.currentItem.text == "POINT")
        sample.sampleType = 0
    end

    if (widget.listSampleTypes.currentItem.text == "CIRCLE")
        sample.sampleType = 1
    end

    if (widget.listSampleTypes.currentItem.text == "BOX")
        sample.sampleType = 2
    end

    sample.min = Eigen::Vector3.new(widget.min_x.value, widget.min_y.value, 0)
    sample.max = Eigen::Vector3.new(widget.max_x.value, widget.max_y.value, 0)

    sample.radius = widget.sb_radius.value
    sample.soilType = 2

    samples = planner_port_soil_sample_writer.new_sample
    samples.push(sample)
    planner_port_soil_sample_writer.write(samples)    
    puts "Sand!"
end

widget.btn_gravel.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()

    if (widget.listSampleTypes.currentItem.text == "POINT")
        sample.sampleType = 0
    end

    if (widget.listSampleTypes.currentItem.text == "CIRCLE")
        sample.sampleType = 1
    end

    if (widget.listSampleTypes.currentItem.text == "BOX")
        sample.sampleType = 2
    end

    sample.min = Eigen::Vector3.new(widget.min_x.value, widget.min_y.value, 0)
    sample.max = Eigen::Vector3.new(widget.max_x.value, widget.max_y.value, 0)

    sample.radius = widget.sb_radius.value
    sample.soilType = 3

    samples = planner_port_soil_sample_writer.new_sample
    samples.push(sample)
    planner_port_soil_sample_writer.write(samples)    
    puts "Gravel!"
end

widget.btn_clear.connect(SIGNAL('clicked()')) do
    result = planner_task_ns.clearSoilMap()

    if result
        puts "Soil Map Cleared!"
    else
        puts "Failed to clear!"
    end
end

Vizkit.exec