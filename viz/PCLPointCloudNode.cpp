#include "PCLPointCloudNode.hpp"


#include <vizkit3d/ColorConversionHelper.hpp>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

PCLPointCloudNode::PCLPointCloudNode() {

}

void PCLPointCloudNode::addLodLevel(const float& from, const float& to, const float& downsample) {
    LodLevel level;
    level.downsample = downsample;
    level.pointGeom = new osg::Geometry;
    level.pointsOSG = new osg::Vec3Array;
    level.pointGeom->setVertexArray(level.pointsOSG);
    level.color = new osg::Vec4Array;
    level.pointGeom->setColorArray(level.color);
    level.pointGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    level.pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    level.drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, level.pointsOSG->size() );
    level.pointGeom->addPrimitiveSet(level.drawArrays.get());
    level.geode = new osg::Geode;
    level.geode->addDrawable(level.pointGeom.get());
    
    lodlevels.push_back(level);

    this->addChild(level.geode, from, to);
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
    traversePoints<pcl::PointCloud<pcl::PointXYZRGBA>>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZRGBA>& pc, LodLevel &lodlevel, const size_t& i){
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
    traversePoints<pcl::PointCloud<pcl::PointXYZRGB>>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZRGB>& pc, LodLevel &lodlevel, const size_t& i){
        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        lodlevel.color->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, 1.0));
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZI>& pc, const osg::Vec4f& default_feature_color, double maxz, float downsample) {        
    traversePoints<pcl::PointCloud<pcl::PointXYZI>>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZI>& pc, LodLevel &lodlevel, const size_t& i){
        osg::Vec4f feature_color = default_feature_color;
        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        feature_color.w() = pc[i].intensity;
        lodlevel.color->push_back(feature_color);
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample) {
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (pc, minPt, maxPt);
    
    if (maxz == std::numeric_limits<double>::max()) {
        cycle_color_interval = sqrt((maxPt.z - minPt.z) * (maxPt.z - minPt.z));
    } else {
        cycle_color_interval = sqrt((maxz - minPt.z) * (maxz - minPt.z));
    }
    

    traversePoints<pcl::PointCloud<pcl::PointXYZ>>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZ>& pc, LodLevel &lodlevel, const size_t& i){
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
    traversePoints<pcl::PointCloud<pcl::PointXYZ>>(pc, downsample, maxz, [&](const pcl::PointCloud<pcl::PointXYZ>& pc, LodLevel &lodlevel, const size_t& i){
        lodlevel.pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
        lodlevel.color->push_back(default_feature_color);
    });
}