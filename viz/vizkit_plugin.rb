Vizkit::UiLoader.register_3d_plugin('PCLPointCloudVisualization', 'vizkit3d_pcl', 'PCLPointCloudVisualization')
Vizkit::UiLoader.register_3d_plugin_for('PCLPointCloudVisualization', "/pcl/PCLPointCloud2", :updateData )
Vizkit::UiLoader.register_3d_plugin('PolygonMeshVisualization', 'vizkit3d_pcl', 'PolygonMeshVisualization')
Vizkit::UiLoader.register_3d_plugin_for('PolygonMeshVisualization', "/pcl/PolygonMesh", :updateData )