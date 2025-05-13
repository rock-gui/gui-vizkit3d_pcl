#include "PCLPointCloudNode.hpp"


#include <vizkit3d/ColorConversionHelper.hpp>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

PCLPointCloudNode::PCLPointCloudNode():subCloudsX(1),subCloudsY(1),subCloudsZ(1) {
    subClouds = std::make_shared<SubClouds>(subCloudsX,subCloudsY,subCloudsZ);
    addChild(subClouds->osgGroup);
}

void PCLPointCloudNode::addLodLevel(const float& from, const float& to, const float& downsample) {
    for (auto& subcloud : *subClouds) {
        subcloud->addLodLevel(from, to, downsample);
    }
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
    traversePoints<pcl::PointXYZRGBA>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZRGBA>& pc, LodLevel &lodlevel, const size_t& i, const pcl::PointXYZRGBA& minPt, const pcl::PointXYZRGBA& maxPt){
        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        if(!show_color)
            lodlevel.color->push_back(osg::Vec4f(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), pc[i].a/255.0));
        else if(!show_intensity)
            lodlevel.color->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, 1.0));
        else
            lodlevel.color->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, pc[i].a/255.0));
    });
}


void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pc, double maxz, float downsample) {
    traversePoints<pcl::PointXYZRGB>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZRGB>& pc, LodLevel &lodlevel, const size_t& i, const pcl::PointXYZRGB& minPt, const pcl::PointXYZRGB& maxPt){
        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        lodlevel.color->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, 1.0));
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZI>& pc, const osg::Vec4f& default_feature_color, double maxz, float downsample) {        
    traversePoints<pcl::PointXYZI>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZI>& pc, LodLevel &lodlevel, const size_t& i, const pcl::PointXYZI& minPt, const pcl::PointXYZI& maxPt){
        osg::Vec4f feature_color = default_feature_color;
        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        feature_color.w() = pc[i].intensity;
        lodlevel.color->push_back(feature_color);
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample) {
    bool color_cycle_set = false;

    traversePoints<pcl::PointXYZ>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZ>& pc, LodLevel &lodlevel, const size_t& i, const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt){

        if (!color_cycle_set) {
            if (maxz == std::numeric_limits<double>::max()) {
                cycle_color_interval = sqrt((maxPt.z - minPt.z) * (maxPt.z - minPt.z));
            } else {
                cycle_color_interval = sqrt((maxz - minPt.z) * (maxz - minPt.z));
            }
            color_cycle_set = true;
        }

        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        //cal height based on
        float hue = (pc[i].z - std::floor(pc[i].z / cycle_color_interval) * cycle_color_interval) / cycle_color_interval;
        float sat = 1; 
        float lum = 0.5;
        float alpha = default_feature_color[3]; //use opacity of set color
        osg::Vec4f heightcolor;
        vizkit3d::hslToRgb(hue, sat, lum , heightcolor.x(), heightcolor.y(), heightcolor.z());
        heightcolor.w() = alpha;
        lodlevel.color->push_back(heightcolor);
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, double maxz, float downsample) {
    traversePoints<pcl::PointXYZ>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZ>& pc, LodLevel &lodlevel, const size_t& i, const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt){
        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        lodlevel.color->push_back(default_feature_color);
    });
}