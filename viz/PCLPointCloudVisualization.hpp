#ifndef vizkit3d_pcl_PCLPointCloudVisualization_H
#define vizkit3d_pcl_PCLPointCloudVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <pcl/PCLPointCloud2.h>
#include <osg/Geometry>

namespace vizkit3d
{
    class PCLPointCloudVisualization
        : public vizkit3d::Vizkit3DPlugin<pcl::PCLPointCloud2>
        , boost::noncopyable
    {
    Q_OBJECT
    Q_PROPERTY(QColor defaultFeatureColor READ getDefaultFeatureColor WRITE setDefaultFeatureColor)
    Q_PROPERTY(double pointSize READ getPointSize WRITE setPointSize)
    Q_PROPERTY(bool showColor READ getShowColor WRITE setShowColor)
    Q_PROPERTY(bool useHeightColoring READ getUseHeightColoring WRITE setUseHeightColoring)
    Q_PROPERTY(bool showIntensity READ getShowIntensity WRITE setShowIntensity)
    Q_PROPERTY(bool updateFrameOnlyOnNewData READ getUpdateFramePositionOnlyOnNewData WRITE setUpdateFramePositionOnlyOnNewData)

    public slots:
        QColor getDefaultFeatureColor();
        void setDefaultFeatureColor(QColor color);
        double getPointSize();
        void setPointSize(double size);
        bool getShowColor();
        void setShowColor(bool b);
        bool getShowIntensity();
        void setShowIntensity(bool b);
        bool getUseHeightColoring();
        void setUseHeightColoring(bool b);


    public:
        PCLPointCloudVisualization();
        ~PCLPointCloudVisualization();

        Q_INVOKABLE void updateData(pcl::PCLPointCloud2 const &sample)
        {vizkit3d::Vizkit3DPlugin<pcl::PCLPointCloud2>::updateData(sample);}

        bool getUpdateFramePositionOnlyOnNewData() {
            return updateDataFramePosition;
        }

        void setUpdateFramePositionOnlyOnNewData(const bool &newvalue) {
            updateDataFramePosition = newvalue;
            setManualVizPoseUpdateEnabled(updateDataFramePosition);
        }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(pcl::PCLPointCloud2 const& plan);

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
        bool updateDataFramePosition;
        bool useHeightColoring;
    };
}
#endif
