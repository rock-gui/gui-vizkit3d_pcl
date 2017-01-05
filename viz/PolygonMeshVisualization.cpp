#include <iostream>
#include "PolygonMeshVisualization.hpp"
#include "PointCloudDispatcher.hpp"
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <osg/Point>
#include <vizkit3d/ColorConversionHelper.hpp>

using namespace vizkit3d;

struct PolygonMeshVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    pcl::PolygonMesh data;
};

PolygonMeshVisualization::PolygonMeshVisualization() : p(new Data), default_feature_color(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)), show_color(true), show_intensity(false), colorize_height(false), colorize_interval(1.0)
{

}

PolygonMeshVisualization::~PolygonMeshVisualization()
{
    delete p;
}

osg::ref_ptr< osg::Node > PolygonMeshVisualization::createMainNode()
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

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    return mainNode;
}

void PolygonMeshVisualization::updateMainNode(osg::Node* node)
{
    pointsOSG->clear();
    color->clear();
    pointGeom->removePrimitiveSet(0, pointGeom->getNumPrimitiveSets());

    // dispatch PCLPointCloud2 to osg format
    PointCloudDispatcher::dispatch(p->data.cloud, pointsOSG, color, default_feature_color, show_color, show_intensity);

    if(colorize_height)
    {
        for(size_t i = 0; i < pointsOSG->size(); i++)
        {
            double hue = (pointsOSG->at(i).z() - std::floor(pointsOSG->at(i).z() / colorize_interval) * colorize_interval) / colorize_interval;
            osg::Vec4& c = color->at(i);
            hslToRgb(hue, 1.0, 0.5, c.r(), c.g(), c.b());
        }
    }

    pointGeom->setVertexArray(pointsOSG);
    pointGeom->setColorArray(color);

    // generate polygons
    for(std::vector<pcl::Vertices>::const_iterator it = p->data.polygons.begin(); it != p->data.polygons.end(); it++)
    {
        if(it->vertices.empty())
            continue;
        osg::ref_ptr<osg::DrawElementsUInt> polygon = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, it->vertices.size());
        for(unsigned i = 0; i < it->vertices.size(); i++)
        {
            (*polygon)[i] = it->vertices[i];
        }
        pointGeom->addPrimitiveSet(polygon);
    }
}

void PolygonMeshVisualization::updateDataIntern(const pcl::PolygonMesh& value)
{
    p->data = value;
}

QColor PolygonMeshVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void PolygonMeshVisualization::setDefaultFeatureColor(QColor color)
{
    setDirty();
    default_feature_color.x() = color.redF();
    default_feature_color.y() = color.greenF();
    default_feature_color.z() = color.blueF();
    default_feature_color.w() = color.alphaF();
    emit propertyChanged("defaultFeatureColor");
}

bool PolygonMeshVisualization::getShowColor()
{
    return show_color;
}

void PolygonMeshVisualization::setShowColor(bool b)
{
    if(show_color != b)
        setDirty();
    show_color = b;
    emit propertyChanged("showColor");
}

bool PolygonMeshVisualization::getShowIntensity()
{
    return show_intensity;
}

void PolygonMeshVisualization::setShowIntensity(bool b)
{
    if(show_intensity != b)
        setDirty();
    show_intensity = b;
    emit propertyChanged("showIntensity");
}

bool PolygonMeshVisualization::isColorizeHeightEnabled() const
{
    return colorize_height;
}

void PolygonMeshVisualization::setColorizeHeight(bool value)
{
    colorize_height = value;
    emit propertyChanged("colorizeHeight");
}

double PolygonMeshVisualization::getColorizeInterval() const
{
    return colorize_interval;
}

void PolygonMeshVisualization::setColorizeInterval(double value)
{
    if(value != 0.0)
    {
        colorize_interval = value;
        emit propertyChanged("colorizeInterval");
    }
}