#include <iostream>
#include <limits>
#include "PCLPointCloudVisualization.hpp"
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <osg/LOD>
#include <osgUtil/Simplifier>

using namespace vizkit3d;

const double DEFAULT_POINT_SIZE = 2.0;

struct PCLPointCloudVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    pcl::PCLPointCloud2 pc2;
    pcl::PointCloud<pcl::PointXYZ> pcxyz;
};


PCLPointCloudVisualization::PCLPointCloudVisualization()
    : p(new Data),
    default_feature_color(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)),
    show_color(true),
    show_intensity(false),
    updateDataFramePosition(false),
    useHeightColoring(false),
    maxZ(std::numeric_limits<double>::max()),
    downsampleRatio(1),
    pointSize(1),
    autoLod(false),
    cubeSplitting(1)
{
}

PCLPointCloudVisualization::~PCLPointCloudVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> PCLPointCloudVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();

    cloudnode = new PCLPointCloudNode();
    mainNode->addChild(cloudnode);

    setPointSize(DEFAULT_POINT_SIZE);
    return mainNode;
}

void PCLPointCloudVisualization::updateMainNode ( osg::Node* node )
{
    cloudnode->setSubCloudBoxes(cubeSplitting,cubeSplitting,cubeSplitting);
    cloudnode->clear(); // <- reset to use setSubCloudBoxes
    float maxviz = FLT_MAX;
    if (autoLod){
        // lower max visibility of the full cloud (set below)
        maxviz = 50;
        cloudnode->addLodLevel(maxviz,100,4);  //each from 50 to 100m distance display each 2nd point
        cloudnode->addLodLevel(100,200,16);
        cloudnode->addLodLevel(200,300,32);
        cloudnode->addLodLevel(300,FLT_MAX,64); // actually clipped out be near far plane
    }

    // add the default layer (not downsampled)
    cloudnode->addLodLevel(0, maxviz, 1);

    // dispatch PCLPointCloud2 to osg format
    if (p->pc2.width > 0 || p->pc2.height > 0) {
        cloudnode->dispatch(p->pc2, default_feature_color, show_color, show_intensity, useHeightColoring, maxZ, downsampleRatio);
    } else if (p->pcxyz.size()) {
        cloudnode->dispatch(p->pcxyz, default_feature_color, useHeightColoring, maxZ, downsampleRatio);
    }
    cloudnode->setPointSize(pointSize);
}

void PCLPointCloudVisualization::updateDataIntern(const pcl::PCLPointCloud2 &data)
{
    if (updateDataFramePosition)
    {
        updateManualVizPose();
    }
    p->pc2 = data;
}

void PCLPointCloudVisualization::updateDataIntern(const pcl::PointCloud<pcl::PointXYZ> &data)
{
    if (updateDataFramePosition)
    {
        updateManualVizPose();
    }
    p->pcxyz = data;
}

QColor PCLPointCloudVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void PCLPointCloudVisualization::setDefaultFeatureColor(QColor color)
{
    setDirty();
    default_feature_color.x() = color.redF();
    default_feature_color.y() = color.greenF();
    default_feature_color.z() = color.blueF();
    default_feature_color.w() = color.alphaF();
    emit propertyChanged("defaultFeatureColor");
}

double PCLPointCloudVisualization::getPointSize()
{
    if(cloudnode)
    {
        return cloudnode->getPointSize();
    }
    return DEFAULT_POINT_SIZE;
    
}

void PCLPointCloudVisualization::setPointSize(double size)
{
    if (size <= 0.0) {
        size = 0.01;
    }
    if (cloudnode){
        cloudnode->setPointSize(size);
    }
    pointSize = size;
    emit propertyChanged("pointSize");
}

bool PCLPointCloudVisualization::getShowColor()
{
    return show_color;
}

void PCLPointCloudVisualization::setShowColor(bool b)
{
    if(show_color != b)
        setDirty();
    show_color = b;
    emit propertyChanged("showColor");
}

bool PCLPointCloudVisualization::getShowIntensity()
{
    return show_intensity;
}

void PCLPointCloudVisualization::setShowIntensity(bool b)
{
    if(show_intensity != b)
        setDirty();
    show_intensity = b;
    emit propertyChanged("showIntensity");
}

bool PCLPointCloudVisualization::getUseHeightColoring()
{
    return useHeightColoring;
}

void PCLPointCloudVisualization::setUseHeightColoring(bool b)
{
    if(useHeightColoring != b)
        setDirty();
    useHeightColoring = b;
    emit propertyChanged("useHeightColoring");
}

double PCLPointCloudVisualization::getMaxZ()
{
    return maxZ;
}

void PCLPointCloudVisualization::setMaxZ(double value)
{
    maxZ=value;
    setDirty();
    emit propertyChanged("maxZ");
}

double PCLPointCloudVisualization::getDownsampleRatio()
{
    return downsampleRatio;
}

void PCLPointCloudVisualization::setDownsampleRatio(double value)
{
    downsampleRatio=value;
    setDirty();
    emit propertyChanged("downsampleRatio");
}

bool PCLPointCloudVisualization::getAutoLod()
{
    return autoLod;
}

void PCLPointCloudVisualization::setAutoLod(bool b)
{
    if(autoLod != b)
        setDirty();
    autoLod = b;
    emit propertyChanged("autoLod");
}

int PCLPointCloudVisualization::getCubeSplitting()
{
    return cubeSplitting;
}

void PCLPointCloudVisualization::setCubeSplitting(int b)
{
    if(cubeSplitting != b)
        setDirty();
    cubeSplitting = (b<1)?1:b;
    emit propertyChanged("cubeSplitting");
}

