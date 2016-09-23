#include <iostream>
#include "PCLPointCloudVisualization.hpp"
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
    : p(new Data), default_feature_color(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)), new_points(false), show_color(true), show_intensity(false)
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
    if(new_points)
    {
        new_points = false;

        pointsOSG->clear();
        color->clear();

        if(pcl::getFieldIndex(p->data, "rgba") != -1 && (show_color || show_intensity))
        {
            pcl::PointCloud<pcl::PointXYZRGBA> pc;
            pcl::fromPCLPointCloud2(p->data, pc);

            for(size_t i = 0; i < pc.size(); i++)
            {
                pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
                if(!show_color)
                    color->push_back(osg::Vec4f(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), pc[i].a/255.0));
                else if(!show_intensity)
                    color->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, 1.0));
                else
                    color->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, pc[i].a/255.0));

            }
        }
        else if(pcl::getFieldIndex(p->data, "rgb") != -1 && show_color)
        {
            pcl::PointCloud<pcl::PointXYZRGB> pc;
            pcl::fromPCLPointCloud2(p->data, pc);

            for(size_t i = 0; i < pc.size(); i++)
            {
                pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
                color->push_back(osg::Vec4f(pc[i].r/255.0, pc[i].g/255.0, pc[i].b/255.0, 1.0));
            }
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZ> pc;
            pcl::fromPCLPointCloud2(p->data, pc);

            for(size_t i = 0; i < pc.size(); i++)
            {
                pointsOSG->push_back(osg::Vec3f(pc[i].x, pc[i].y, pc[i].z));
                color->push_back(default_feature_color);

            }
        }

        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);
    }
}

void PCLPointCloudVisualization::updateDataIntern(pcl::PCLPointCloud2 const& value)
{
    p->data = value;
    new_points = true;
}

QColor PCLPointCloudVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void PCLPointCloudVisualization::setDefaultFeatureColor(QColor color)
{
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
    show_color = b;
    emit propertyChanged("showColor");
}

bool PCLPointCloudVisualization::getShowIntensity()
{
    return show_intensity;
}

void PCLPointCloudVisualization::setShowIntensity(bool b)
{
    show_intensity = b;
    emit propertyChanged("showIntensity");
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(PCLPointCloudVisualization)
