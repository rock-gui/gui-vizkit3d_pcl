#include <iostream>
#include <limits>
#include "PCLPointCloudVisualization.hpp"
#include "PointCloudDispatcher.hpp"
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <osg/Point>
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
    : p(new Data), default_feature_color(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)), show_color(true), show_intensity(false), updateDataFramePosition(false), useHeightColoring(false), maxZ(std::numeric_limits<double>::max()), downsampleRatio(1), autoLod(false)
{
}

PCLPointCloudVisualization::~PCLPointCloudVisualization()
{
    delete p;
}


void PCLPointCloudVisualization::addLodLevel(const float& from, const float& to, const float& downsample) {
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

    lodnode->addChild(level.geode, from, to);
}

osg::ref_ptr<osg::Node> PCLPointCloudVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();

    lodnode = new osg::LOD();

    float maxviz = FLT_MAX;
    if (autoLod){
        // lower max visibility of the full cloud (set below)
        maxviz = 50;
        addLodLevel(50,100,0.5);
        addLodLevel(100,130,0.1);
        addLodLevel(130,FLT_MAX,0.01);
    }

    // add the default layer (not downsampled)
    addLodLevel(0, maxviz, 1);

    mainNode->addChild(lodnode);

    setPointSize(DEFAULT_POINT_SIZE);
    return mainNode;
}

void PCLPointCloudVisualization::updateMainNode ( osg::Node* node )
{
    lodlevels.clear();
    lodnode->removeChildren(0, lodnode->getNumChildren());

    float maxviz = FLT_MAX;
    if (autoLod){
        // lower max visibility of the full cloud (set below)
        maxviz = 50;
        addLodLevel(50,100,4);  //each from 50 to 100m distance display each 2nd point
        addLodLevel(100,130,16);
        addLodLevel(130,FLT_MAX,100); // actually clipped out be near far plane
    }

    // add the default layer (not downsampled)
    addLodLevel(0, maxviz, 1);

    for (const auto& lodlevel : lodlevels) {

        lodlevel.pointsOSG->clear();
        lodlevel.color->clear();
        // dispatch PCLPointCloud2 to osg format
        int skip = lodlevel.downsample / downsampleRatio; // downsample skip should increase with a lower ratio

        if (p->pc2.width > 0 || p->pc2.height > 0) {
            PointCloudDispatcher::dispatch(p->pc2, lodlevel.pointsOSG, lodlevel.color, default_feature_color, show_color, show_intensity, useHeightColoring, maxZ, skip);
        } else if (p->pcxyz.size()) {
            PointCloudDispatcher::dispatch(p->pcxyz, lodlevel.pointsOSG, lodlevel.color, default_feature_color, show_color, show_intensity, useHeightColoring, maxZ, skip);
        }
        lodlevel.drawArrays->setCount(lodlevel.pointsOSG->size());
        lodlevel.pointGeom->setVertexArray(lodlevel.pointsOSG);
        lodlevel.pointGeom->setColorArray(lodlevel.color);
    }
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
    if(lodlevels.size() && lodlevels.front().pointGeom.valid())
    {
        osg::Point *pt = dynamic_cast<osg::Point*>(lodlevels.front().pointGeom->getOrCreateStateSet()->getAttribute(osg::StateAttribute::POINT));
        if(pt)
            return pt->getSize();
    }
    return DEFAULT_POINT_SIZE;
}

void PCLPointCloudVisualization::setPointSize(double size)
{
    if (size <= 0.0) {
        size = 0.01;
    }
    osg::ref_ptr<osg::Point> pt = new osg::Point(size);

    for (const auto& lodlevel : lodlevels) {
        lodlevel.pointGeom->getOrCreateStateSet()->setAttribute(pt, osg::StateAttribute::ON);
    }
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
