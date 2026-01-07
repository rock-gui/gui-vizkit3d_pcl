# Default to C++14 (PCL requirement)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include(RockQt)

rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)
add_definitions(-DEIGEN_USE_NEW_STDVECTOR)
find_package(PCL 1.7 REQUIRED COMPONENTS common)
IF("${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}" VERSION_LESS 1.14)
    SET(PCL_VERSION_SUFFIX "-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
ENDIF()

rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

rock_qt_vizkit_plugin(
    TARGETPREFIX vizkit3d_pcl-viz
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        PluginLoader.cpp
        PCLPointCloudVisualization.cpp
        PolygonMeshVisualization.cpp
        PCLPointCloudNode.cpp
    HEADERS
        PCLPointCloudNode.hpp
        PCLPointCloudVisualization.hpp
        PolygonMeshVisualization.hpp
    MOC
        PluginLoader.hpp
        PCLPointCloudVisualization.hpp
        PolygonMeshVisualization.hpp
    DEPS_PKGCONFIG eigen3 pcl_common${PCL_VERSION_SUFFIX}
)
