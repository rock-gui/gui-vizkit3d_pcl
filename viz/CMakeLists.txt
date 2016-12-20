rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)
add_definitions(-DEIGEN_USE_NEW_STDVECTOR)

rock_vizkit_plugin(vizkit3d_pcl-viz
        PluginLoader.cpp
    DEPS_PKGCONFIG
        pcl_common-1.7
    MOC
        PCLPointCloudVisualization.cpp
        PolygonMeshVisualization.cpp
    HEADERS
        PCLPointCloudVisualization.hpp
        PolygonMeshVisualization.hpp
        PointCloudDispatcher.hpp)

