#ifndef vizkit3d_pcl_PCLPointCloudVisualization_H
#define vizkit3d_pcl_PCLPointCloudVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

#include "PCLPointCloudNode.hpp"

namespace vizkit3d
{
    class PCLPointCloudVisualization
        : public vizkit3d::Vizkit3DPlugin<pcl::PCLPointCloud2>
        , public vizkit3d::VizPluginAddType<pcl::PointCloud<pcl::PointXYZ>>
        , boost::noncopyable
    {

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
    Q_PROPERTY(int cubeSplitting READ getCubeSplitting WRITE setCubeSplitting)

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
        int getCubeSplitting();
        void setCubeSplitting(int b);

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

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);

        virtual void updateDataIntern(const pcl::PCLPointCloud2 &data);
        virtual void updateDataIntern(const pcl::PointCloud<pcl::PointXYZ> &data);

    private:
        struct Data;
        Data* p;
        osg::Vec4f default_feature_color;
        osg::ref_ptr<PCLPointCloudNode> cloudnode;

        bool show_color;
        bool show_intensity;
        bool updateDataFramePosition;
        bool useHeightColoring;
        double maxZ;
        double downsampleRatio;
        bool autoLod;
        int cubeSplitting;
    };
}
#endif
