#include "PCLPointCloudNode.hpp"


#include <pcl/common/common.h>
#include <vizkit3d/ColorConversionHelper.hpp>

PCLPointCloudNode::PCLPointCloudNode() {

}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample) {
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (pc, minPt, maxPt);
    float cycle_color_interval = 0;
    if (maxz == std::numeric_limits<double>::max()) {
        cycle_color_interval = sqrt((maxPt.z - minPt.z) * (maxPt.z - minPt.z));
    } else {
        cycle_color_interval = sqrt((maxz - minPt.z) * (maxz - minPt.z));
    }
    
    for (const auto &lodlevel: lodlevels)
    {
        lodlevel.pointsOSG->clear();
        lodlevel.color->clear();
        int cloudsize = pc.size() * downsample;
        lodlevel.pointsOSG->reserve(cloudsize);
        lodlevel.color->reserve(cloudsize);
    }

    for(size_t i = 0; i < pc.size(); ++i)
    {
        for (const auto &lodlevel: lodlevels)
        {
            int downsampleSkip = lodlevel.downsample / downsample;
            if (i % downsampleSkip == 0) {
                if (pc[i].z < maxz) {
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


