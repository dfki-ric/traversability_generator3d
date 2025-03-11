Vizkit::UiLoader.register_3d_plugin('TravMap3dVisualization', "traversability_generator3d", 'TravMap3dVisualization' )
Vizkit::UiLoader.register_3d_plugin_for('TravMap3dVisualization', "/traversability_generator3d/TravMap3d", :updateData )

Vizkit::UiLoader.register_3d_plugin('SoilMap3dVisualization', "traversability_generator3d", 'SoilMap3dVisualization' )
Vizkit::UiLoader.register_3d_plugin_for('SoilMap3dVisualization', "/traversability_generator3d/SoilMap3d", :updateData )