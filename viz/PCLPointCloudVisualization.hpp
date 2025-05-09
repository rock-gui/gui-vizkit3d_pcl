#ifndef vizkit3d_pcl_PCLPointCloudVisualization_H
#define vizkit3d_pcl_PCLPointCloudVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <osg/Geometry>

namespace vizkit3d
{
    class PCLPointCloudVisualization
        : public vizkit3d::Vizkit3DPlugin<pcl::PCLPointCloud2>
        , public vizkit3d::VizPluginAddType<pcl::PointCloud<pcl::PointXYZ>>
        , boost::noncopyable
    {

        struct LodLevel {
            float downsample;
            osg::ref_ptr<osg::Vec3Array> pointsOSG;
            osg::ref_ptr<osg::DrawArrays> drawArrays;
            osg::ref_ptr<osg::Geometry> pointGeom;
            osg::ref_ptr<osg::Vec4Array> color;
            osg::ref_ptr<osg::Geode> geode;
        };

    Q_OBJECT
    Q_PROPERTY(QColor defaultFeatureColor READ getDefaultFeatureColor WRITE setDefaultFeatureColor)
    Q_PROPERTY(double pointSize READ getPointSize WRITE setPointSize)
    Q_PROPERTY(bool showColor READ getShowColor WRITE setShowColor)
    Q_PROPERTY(bool useHeightColoring READ getUseHeightColoring WRITE setUseHeightColoring)
    Q_PROPERTY(bool showIntensity READ getShowIntensity WRITE setShowIntensity)
    Q_PROPERTY(bool updateFrameOnlyOnNewData READ getUpdateFramePositionOnlyOnNewData WRITE setUpdateFramePositionOnlyOnNewData)
    Q_PROPERTY(double maxZ READ getMaxZ WRITE setMaxZ)
    Q_PROPERTY(double downsampleRatio READ getDownsampleRatio WRITE setDownsampleRatio)
    Q_PROPERTY(bool autoLod READ getAutoLod WRITE setAutoLod)

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
        double getMaxZ();
        void setMaxZ(double value);
        double getDownsampleRatio();
        void setDownsampleRatio(double value);
        bool getAutoLod();
        void setAutoLod(bool b);

    public:
        PCLPointCloudVisualization();
        ~PCLPointCloudVisualization();

        Q_INVOKABLE void updateData(pcl::PCLPointCloud2 const &sample)
        {vizkit3d::Vizkit3DPlugin<pcl::PCLPointCloud2>::updateData(sample);}

        Q_INVOKABLE void updateData(pcl::PointCloud<pcl::PointXYZ> const &sample)
        {vizkit3d::Vizkit3DPlugin<pcl::PCLPointCloud2>::updateData(sample);}


        bool getUpdateFramePositionOnlyOnNewData() {
            return updateDataFramePosition;
        }

        void setUpdateFramePositionOnlyOnNewData(const bool &newvalue) {
            updateDataFramePosition = newvalue;
            setManualVizPoseUpdateEnabled(updateDataFramePosition);
        }

        void addLodLevel(const float& from, const float& to, const float& downsample);

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);

        virtual void updateDataIntern(const pcl::PCLPointCloud2 &data);
        virtual void updateDataIntern(const pcl::PointCloud<pcl::PointXYZ> &data);

    private:
        struct Data;
        Data* p;
        osg::Vec4f default_feature_color;
        osg::ref_ptr<osg::LOD> lodnode;
        std::vector<LodLevel> lodlevels; 

        bool show_color;
        bool show_intensity;
        bool updateDataFramePosition;
        bool useHeightColoring;
        double maxZ;
        double downsampleRatio;
        bool autoLod;
    };
}
#endif
