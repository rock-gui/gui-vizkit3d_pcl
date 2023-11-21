#include <iostream>
#include "PCLPointCloudVisualization.hpp"
#include "PointCloudDispatcher.hpp"
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <osg/Point>

using namespace vizkit3d;

const double DEFAULT_POINT_SIZE = 2.0;

struct PCLPointCloudVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    pcl::PCLPointCloud2 data;
};


PCLPointCloudVisualization::PCLPointCloudVisualization()
    : p(new Data), default_feature_color(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)), show_color(true), show_intensity(false), updateDataFramePosition(false)
{
}

PCLPointCloudVisualization::~PCLPointCloudVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> PCLPointCloudVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();

    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    setPointSize(DEFAULT_POINT_SIZE);
    return mainNode;
}

void PCLPointCloudVisualization::updateMainNode ( osg::Node* node )
{
    pointsOSG->clear();
    color->clear();

    // dispatch PCLPointCloud2 to osg format
    PointCloudDispatcher::dispatch(p->data, pointsOSG, color, default_feature_color, show_color, show_intensity);

    drawArrays->setCount(pointsOSG->size());
    pointGeom->setVertexArray(pointsOSG);
    pointGeom->setColorArray(color);
}

void PCLPointCloudVisualization::updateDataIntern(pcl::PCLPointCloud2 const& value)
{
    if (updateDataFramePosition)
    {
        updateManualVizPose();
    }
    p->data = value;
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
    if(pointGeom.valid())
    {
        osg::Point *pt = dynamic_cast<osg::Point*>(pointGeom->getOrCreateStateSet()->getAttribute(osg::StateAttribute::POINT));
        if(pt)
            return pt->getSize();
    }
    return DEFAULT_POINT_SIZE;
}

void PCLPointCloudVisualization::setPointSize(double size)
{
    if(pointGeom.valid())
    {
        if(size <= 0.0)
            size = 0.01;
        osg::ref_ptr<osg::Point> pt = new osg::Point(size);
        pointGeom->getOrCreateStateSet()->setAttribute(pt, osg::StateAttribute::ON);
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
