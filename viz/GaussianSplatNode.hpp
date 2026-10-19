#pragma once

#include <future>
#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Uniform>

#include <pcl/PCLPointCloud2.h>

/**
 * @brief Renders 3D gaussian splats (as exported by 3DGS/Brush/etc. in PLY format)
 *
 * Expects a pcl::PCLPointCloud2 with the standard 3DGS fields:
 * x, y, z, scale_0..2 (log scale), rot_0..3 (quaternion w,x,y,z), opacity (logit), f_dc_0..2 (SH degree 0).
 * Higher order SH (f_rest_*) is ignored, colors are view independent.
 *
 * Each splat is drawn as a screen space ellipse (EWA projection of the 3D covariance)
 * expanded from a point in a geometry shader. Splats are sorted back to front in a
 * background thread whenever the view direction changes.
 */
class GaussianSplatNode : public osg::Group {
 public:
    GaussianSplatNode();
    virtual ~GaussianSplatNode();

    /**
     * @brief true if the cloud contains all fields needed to render it as gaussian splats
     */
    static bool isSplatCloud(const pcl::PCLPointCloud2& cloud);

    void setData(const pcl::PCLPointCloud2& cloud);

    void clear();

    /**
     * @brief multiplier on the splat size (1.0 is the trained size)
     */
    void setSplatScale(float scale);
    float getSplatScale() const { return splatScale; }

    size_t getNumSplats() const { return centers.valid() ? centers->size() : 0; }

    /**
     * @brief called on cull traversal with the current model view matrix and viewport size
     */
    void updateView(const osg::Matrix& modelview, float viewportWidth, float viewportHeight);

 private:
    static std::vector<GLuint> sortBackToFront(const osg::Vec3Array& centers, const osg::Vec3f& viewdir);
    void applySortResult(std::vector<GLuint>&& order);

    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::Geometry> geometry;
    osg::ref_ptr<osg::DrawElementsUInt> indices;
    osg::ref_ptr<osg::Uniform> viewportUniform;
    osg::ref_ptr<osg::Uniform> scaleUniform;

    // shared with the sort thread
    osg::ref_ptr<const osg::Vec3Array> centers;
    std::future<std::vector<GLuint>> sortResult;
    osg::Vec3f sortedViewDir;
    osg::Vec3f requestedViewDir;
    bool sorted;

    float splatScale;
};
