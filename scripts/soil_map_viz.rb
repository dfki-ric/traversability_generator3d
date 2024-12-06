require 'vizkit'
require 'orocos'
require 'cmath'
require 'eigen'
include Orocos

Orocos.initialize
Orocos.load_typekit "base"
Orocos.load_typekit "ugv_nav4d"
#Orocos.load_typekit "traversability_generator3d"

# Define a class to represent a 3D Circle
class Circle3D
    attr_accessor :center, :radius, :points
  
    def initialize(center, radius, num_points)
      @center = center    # [x_center, y_center, z_center]
      @radius = radius    # radius of the circle
      @num_points = num_points # Number of points to generate on the circle
      @points = []        # Array to hold the points on the circle
      generate_points     # Generate the points
    end
  
    # Generate the points on the circle
    def generate_points
      # The circle lies in the XY plane (fixed z-coordinate)
      (0...@num_points).each do |i|
        # Parameterize the circle using an angle θ
        theta = 2 * Math::PI * i / @num_points
        x = @center[0] + @radius * Math.cos(theta)
        y = @center[1] + @radius * Math.sin(theta)
        z = @center[2]  # Assuming the circle lies in a plane parallel to the XY plane
        @points << {x: x, y: y, z: z}
      end
    end
  
    # Function to get the points of the circle
    def get_points
      @points
    end
  
    # Print the points on the circle
    def print_points
      @points.each_with_index do |point, index|
        puts "Point #{index + 1}: (#{point[:x]}, #{point[:y]}, #{point[:z]})"
      end
    end
end
  
if(ARGV.size > 0)
    sim = ARGV[0]
end

### Plugins for the visualization
urf_model_file                                      = "#{ENV['AUTOPROJ_CURRENT_ROOT']}/models/robots/artemis/urdf/artemis.urdf"

# Plugins
robot_visualization_current_state_plugin            = Vizkit.default_loader.RobotVisualization
robot_visualization_current_state_plugin.modelFile  = urf_model_file.dup
soil_map_plugin				                        = Vizkit.default_loader.SoilMap3dVisualization
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

circle = Circle3D.new([0, 0, 0], 1, 10)
widget.btn_concrete.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()
    sample.type = 0
    sample.sigmaX = widget.sigma_x.value
    sample.sigmaY = widget.sigma_y.value
    sample.uncertainty = widget.uncertainty.value
    sample.location = Eigen::Vector3.new(widget.center_x.value, widget.center_y.value, widget.center_z.value)
    samples = planner_port_soil_sample_writer.new_sample
    samples.push(sample)
    planner_port_soil_sample_writer.write(samples)    
    puts "Concrete!"
end

widget.btn_rocks.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()
    sample.type = 1
    sample.sigmaX = widget.sigma_x.value
    sample.sigmaY = widget.sigma_y.value
    sample.uncertainty = widget.uncertainty.value
    sample.location = Eigen::Vector3.new(widget.center_x.value, widget.center_y.value, widget.center_z.value)
    samples = planner_port_soil_sample_writer.new_sample
    samples.push(sample)
    planner_port_soil_sample_writer.write(samples)    
    puts "Rocks!"
end

widget.btn_sand.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()
    sample.type = 2
    sample.sigmaX = widget.sigma_x.value
    sample.sigmaY = widget.sigma_y.value
    sample.uncertainty = widget.uncertainty.value
    sample.location = Eigen::Vector3.new(widget.center_x.value, widget.center_y.value, widget.center_z.value)
    samples = planner_port_soil_sample_writer.new_sample
    samples.push(sample)
    planner_port_soil_sample_writer.write(samples)    
    puts "Sand!"
end

widget.btn_gravel.connect(SIGNAL('clicked()')) do
    sample = Types::Traversability_generator3d::SoilSample.new()
    sample.type = 3
    sample.sigmaX = widget.sigma_x.value
    sample.sigmaY = widget.sigma_y.value
    sample.uncertainty = widget.uncertainty.value
    sample.location = Eigen::Vector3.new(widget.center_x.value, widget.center_y.value, widget.center_z.value)
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