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
        int downsampleSkip;
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

    void dispatch(const pcl::PCLPointCloud2& pcl_cloud, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample);

 protected:

    void dispatch(const pcl::PointCloud<pcl::PointXYZRGBA>& pcl_cloud, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, double maxz, float downsample);
    void dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud, double maxz, float downsample);
    void dispatch(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, const osg::Vec4f& default_feature_color, double maxz, float downsample);
    void dispatch(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const osg::Vec4f& default_feature_color, double maxz, float downsample);


    template <class CLOUD> void traversePoints(const CLOUD& pc, const float& downsample, const float& maxz, std::function <void(const CLOUD&, LodLevel&, const size_t&)> cb) {
        for (auto &lodlevel: lodlevels)
        {
            lodlevel.pointsOSG->clear();
            lodlevel.color->clear();
            int cloudsize = pc.size() * downsample;
            lodlevel.pointsOSG->reserve(cloudsize);
            lodlevel.color->reserve(cloudsize);
            lodlevel.downsampleSkip = lodlevel.downsample / downsample;
        }
        for(size_t i = 0; i < pc.size(); ++i)
        {
            for (auto &lodlevel: lodlevels)
            {
                if (i % lodlevel.downsampleSkip == 0) {
                    if (pc[i].z < maxz) {
                        cb(pc, lodlevel, i);
                    }
                }
            }
        }
        //after sorting, add sizes
        for (const auto &lodlevel: lodlevels)
        {
            lodlevel.drawArrays->setCount(lodlevel.pointsOSG->size());
        }

    }



 private:

    std::vector<LodLevel> lodlevels;
    float cycle_color_interval;
    
    osg::ref_ptr<osg::Vec3Array> osg_points;
    osg::ref_ptr<osg::Vec4Array> osg_colors;


};
