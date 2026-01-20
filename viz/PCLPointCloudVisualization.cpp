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
    show_color(true),
    show_intensity(false),
    updateDataFramePosition(false),
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
    if ((p->pc2.width > 0 || p->pc2.height > 0) && p->pc2.data.size()) {
        cloudnode->dispatch(p->pc2, dispatchConfig, show_color, show_intensity, getCamera());
    } else if (p->pcxyz.size()) {
        cloudnode->dispatch(p->pcxyz, dispatchConfig, getCamera());
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
    color.setRgbF(dispatchConfig.default_feature_color.x(), dispatchConfig.default_feature_color.y(), dispatchConfig.default_feature_color.z(), dispatchConfig.default_feature_color.w());
    return color;
}

void PCLPointCloudVisualization::setDefaultFeatureColor(QColor color)
{
    setDirty();
    dispatchConfig.default_feature_color.x() = color.redF();
    dispatchConfig.default_feature_color.y() = color.greenF();
    dispatchConfig.default_feature_color.z() = color.blueF();
    dispatchConfig.default_feature_color.w() = color.alphaF();
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
    return dispatchConfig.useHeightColoring;
}

void PCLPointCloudVisualization::setUseHeightColoring(bool b)
{
    if(dispatchConfig.useHeightColoring != b)
        setDirty();
    dispatchConfig.useHeightColoring = b;
    emit propertyChanged("useHeightColoring");
}

bool PCLPointCloudVisualization::getCutZ()
{
    return dispatchConfig.cut;
}
void PCLPointCloudVisualization::setCutZ(bool b)
{
    dispatchConfig.cut = b;
    setDirty();
    emit propertyChanged("cutZ");
}

double PCLPointCloudVisualization::getMinZ()
{
    return dispatchConfig.minz;
}

void PCLPointCloudVisualization::setMinZ(double value)
{
    dispatchConfig.minz=value;
    if (dispatchConfig.cut)
    {
        setDirty();
    }
    emit propertyChanged("minZ");
}

double PCLPointCloudVisualization::getMaxZ()
{
    return dispatchConfig.maxz;
}

void PCLPointCloudVisualization::setMaxZ(double value)
{
    dispatchConfig.maxz=value;
    {
        setDirty();
    }
    emit propertyChanged("maxZ");
}

double PCLPointCloudVisualization::getDownsampleRatio()
{
    return dispatchConfig.downsample;
}

void PCLPointCloudVisualization::setDownsampleRatio(double value)
{
    dispatchConfig.downsample=value;
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

