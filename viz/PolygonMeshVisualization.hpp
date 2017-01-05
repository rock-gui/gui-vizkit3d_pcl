#ifndef vizkit3d_pcl_PolygonMeshVisualization_H
#define vizkit3d_pcl_PolygonMeshVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <pcl/PolygonMesh.h>
#include <osg/Geometry>

namespace vizkit3d
{
    class PolygonMeshVisualization
        : public vizkit3d::Vizkit3DPlugin<pcl::PolygonMesh>
        , boost::noncopyable
    {
    Q_OBJECT
    Q_PROPERTY(QColor defaultFeatureColor READ getDefaultFeatureColor WRITE setDefaultFeatureColor)
    Q_PROPERTY(bool showColor READ getShowColor WRITE setShowColor)
    Q_PROPERTY(bool showIntensity READ getShowIntensity WRITE setShowIntensity)
    Q_PROPERTY(bool colorizeHeight READ isColorizeHeightEnabled WRITE setColorizeHeight)
    Q_PROPERTY(double colorizeInterval READ getColorizeInterval WRITE setColorizeInterval)

    public slots:
        QColor getDefaultFeatureColor();
        void setDefaultFeatureColor(QColor color);
        bool getShowColor();
        void setShowColor(bool b);
        bool getShowIntensity();
        void setShowIntensity(bool b);
        void setColorizeHeight(bool value);
        bool isColorizeHeightEnabled()const;
        void setColorizeInterval(double value);
        double getColorizeInterval()const;

    public:
        PolygonMeshVisualization();
        ~PolygonMeshVisualization();

        Q_INVOKABLE void updateData(pcl::PolygonMesh const &sample)
        {vizkit3d::Vizkit3DPlugin<pcl::PolygonMesh>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(pcl::PolygonMesh const& value);

    private:
        struct Data;
        Data* p;
        osg::Vec4f default_feature_color;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
        bool show_color;
        bool show_intensity;
        bool colorize_height;
        double colorize_interval;
    };
}
#endif
