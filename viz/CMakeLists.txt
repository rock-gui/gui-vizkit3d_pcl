rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)
add_definitions(-DEIGEN_USE_NEW_STDVECTOR)

rock_vizkit_plugin(vizkit3d_pcl-viz
    DEPS_PKGCONFIG
        pcl_common-1.8
    MOC
        PCLPointCloudVisualization.cpp
    HEADERS
        PCLPointCloudVisualization.hpp
        PointCloudDispatcher.hpp)

