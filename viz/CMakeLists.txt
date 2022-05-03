# Default to C++14 (PCL requirement)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

rock_find_qt5(Core)

rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)
add_definitions(-DEIGEN_USE_NEW_STDVECTOR)
find_package(PCL 1.7 REQUIRED COMPONENTS common)

rock_vizkit_plugin(vizkit3d_pcl-viz
        PluginLoader.cpp
    DEPS_PKGCONFIG
        pcl_common-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
    MOC
        PCLPointCloudVisualization.cpp
        PolygonMeshVisualization.cpp
    HEADERS
        PCLPointCloudVisualization.hpp
        PolygonMeshVisualization.hpp
        PointCloudDispatcher.hpp)
