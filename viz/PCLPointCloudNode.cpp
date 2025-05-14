#include "PCLPointCloudNode.hpp"


#include <vizkit3d/ColorConversionHelper.hpp>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

PCLPointCloudNode::PCLPointCloudNode():subCloudsX(1),subCloudsY(1),subCloudsZ(1),pointsize(1) {
    subClouds = std::make_shared<SubClouds>(subCloudsX,subCloudsY,subCloudsZ);
    addChild(subClouds->osgGroup);
}

void PCLPointCloudNode::addLodLevel(const float& from, const float& to, const float& downsample) {
    for (auto& subcloud : *subClouds) {
        subcloud->addLodLevel(from, to, downsample);
    }
}

void PCLPointCloudNode::setPointSize(const double & size) {
    pointsize = size;
    for (auto& lodCube : *subClouds)
    {
        for (auto &lodlevel: lodCube->getLodLevels())
        {
            lodlevel.setPointSize(pointsize);
        }
    }
}

double PCLPointCloudNode::getPointSize() {
    return pointsize;
}


void PCLPointCloudNode::dispatch(const pcl::PCLPointCloud2& pcl_cloud, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample) {
    if(pcl::getFieldIndex(pcl_cloud, "rgba") != -1 && (show_color || show_intensity))
    {
        pcl::PointCloud<pcl::PointXYZRGBA> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, show_color, show_intensity, maxz, downsample);
    }
    else if(show_color && pcl::getFieldIndex(pcl_cloud, "rgb") != -1)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, maxz, downsample);
    }
    else if(show_intensity && pcl::getFieldIndex(pcl_cloud, "intensity") != -1)
    {
        pcl::PointCloud<pcl::PointXYZI> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, maxz, downsample);

    }
    else if (useHeightColoring) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, show_color, show_intensity, useHeightColoring, maxz, downsample);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, maxz, downsample);   
    }
        
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZRGBA>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, double maxz, float downsample) {
    traversePoints<pcl::PointXYZRGBA>(pc, downsample, maxz, [&](const pcl::PointXYZRGBA& point, LodLevel &lodlevel, const pcl::PointXYZRGBA& minPt, const pcl::PointXYZRGBA& maxPt){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        if(!show_color)
            lodlevel.getColors()->push_back(osg::Vec4f(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), point.a/255.0));
        else if(!show_intensity)
            lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
        else
            lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, point.a/255.0));
    });
}


void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pc, double maxz, float downsample) {
    traversePoints<pcl::PointXYZRGB>(pc, downsample, maxz, [&](const pcl::PointXYZRGB& point, LodLevel &lodlevel, const pcl::PointXYZRGB& minPt, const pcl::PointXYZRGB& maxPt){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZI>& pc, const osg::Vec4f& default_feature_color, double maxz, float downsample) {        
    traversePoints<pcl::PointXYZI>(pc, downsample, maxz, [&](const pcl::PointXYZI& point, LodLevel &lodlevel, const pcl::PointXYZI& minPt, const pcl::PointXYZI& maxPt){
        osg::Vec4f feature_color = default_feature_color;
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        feature_color.w() = point.intensity;
        lodlevel.getColors()->push_back(feature_color);
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample) {
    bool color_cycle_set = false;

    traversePoints<pcl::PointXYZ>(pc, downsample, maxz, [&](const pcl::PointXYZ& point, LodLevel &lodlevel, const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt){

        if (!color_cycle_set) {
            if (maxz == std::numeric_limits<double>::max()) {
                cycle_color_interval = sqrt((maxPt.z - minPt.z) * (maxPt.z - minPt.z));
            } else {
                cycle_color_interval = sqrt((maxz - minPt.z) * (maxz - minPt.z));
            }
            color_cycle_set = true;
        }

        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        //cal height based on
        float hue = (point.z - std::floor(point.z / cycle_color_interval) * cycle_color_interval) / cycle_color_interval;
        float sat = 1; 
        float lum = 0.5;
        float alpha = default_feature_color[3]; //use opacity of set color
        osg::Vec4f heightcolor;
        vizkit3d::hslToRgb(hue, sat, lum , heightcolor.x(), heightcolor.y(), heightcolor.z());
        heightcolor.w() = alpha;
        lodlevel.getColors()->push_back(heightcolor);
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, double maxz, float downsample) {
    traversePoints<pcl::PointXYZ>(pc, downsample, maxz, [&](const pcl::PointXYZ& point, LodLevel &lodlevel, const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        lodlevel.getColors()->push_back(default_feature_color);
    });
}