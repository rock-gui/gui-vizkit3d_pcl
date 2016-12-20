#ifndef vizkit3d_pcl_PointCloudDispatcher_HPP
#define vizkit3d_pcl_PointCloudDispatcher_HPP

#include <pcl/PCLPointCloud2.h>
#include <osg/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

namespace vizkit3d
{
    class PointCloudDispatcher
    {
    public:
        static void dispatch(const pcl::PCLPointCloud2& pcl_cloud, osg::ref_ptr<osg::Vec3Array> osg_points,
                             osg::ref_ptr<osg::Vec4Array> osg_colors, const osg::Vec4f& default_feature_color,
                             bool show_color, bool show_intensity)
        {
            if(pcl::getFieldIndex(pcl_cloud, "rgba") != -1 && (show_color || show_intensity))
            {
                pcl::PointCloud<pcl::PointXYZRGBA> pc;
                pcl::fromPCLPointCloud2(pcl_cloud, pc);

                for(size_t i = 0; i < pc.size(); i++)
                {
                    osg_points->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
                    if(!show_color)
                        osg_colors->push_back(osg::Vec4f(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), pc[i].a/255.0));
                    else if(!show_intensity)
                        osg_colors->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, 1.0));
                    else
                        osg_colors->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, pc[i].a/255.0));

                }
            }
            else if(show_color && pcl::getFieldIndex(pcl_cloud, "rgb") != -1)
            {
                pcl::PointCloud<pcl::PointXYZRGB> pc;
                pcl::fromPCLPointCloud2(pcl_cloud, pc);

                for(size_t i = 0; i < pc.size(); i++)
                {
                    osg_points->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
                    osg_colors->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, 1.0));
                }
            }
            else if(show_intensity && pcl::getFieldIndex(pcl_cloud, "intensity") != -1)
            {
                pcl::PointCloud<pcl::PointXYZI> pc;
                pcl::fromPCLPointCloud2(pcl_cloud, pc);

                osg::Vec4f feature_color = default_feature_color;
                for(size_t i = 0; i < pc.size(); i++)
                {
                    osg_points->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
                    feature_color.w() = pc[i].intensity;
                    osg_colors->push_back(feature_color);
                }
            }
            else
            {
                pcl::PointCloud<pcl::PointXYZ> pc;
                pcl::fromPCLPointCloud2(pcl_cloud, pc);

                for(size_t i = 0; i < pc.size(); i++)
                {
                    osg_points->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
                    osg_colors->push_back(default_feature_color);

                }
            }
        }

    };
}

#endif