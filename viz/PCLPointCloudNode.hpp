#pragma once

#include <osg/LOD>
#include <osg/Geode>
#include <osg/Geometry>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PCLPointCloudNode : public osg::LOD {
 public:
    struct LodLevel {
        float downsample;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
        osg::ref_ptr<osg::Geode> geode;
    };


    PCLPointCloudNode();

    void clear() {
        this->removeChildren(0, this->getNumChildren());
        lodlevels.clear();
    }

    void addLodLevel(const float& from, const float& to, const float& downsample);

    void dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample);

    // static void dispatch(const pcl::PCLPointCloud2& pcl_cloud, const osg::Vec4f& default_feature_color,
    //                          bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample);



 private:

    std::vector<LodLevel> lodlevels; 
    
    osg::ref_ptr<osg::Vec3Array> osg_points;
    osg::ref_ptr<osg::Vec4Array> osg_colors;


};
