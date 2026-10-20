#pragma once

#include <memory>
#include <vector>

#include <osg/Array>
#include <osg/Geometry>
#include <osg/PrimitiveSet>

#include <pcl/PCLPointCloud2.h>

/**
 * @brief Helpers to render 3D gaussian splats (as exported by 3DGS/Brush/etc. in PLY format)
 *
 * Expects a pcl::PCLPointCloud2 with the standard 3DGS fields:
 * x, y, z, scale_0..2 (log scale), rot_0..3 (quaternion w,x,y,z), opacity (logit), f_dc_0..2 (SH degree 0).
 * Higher order SH (f_rest_*) is ignored, colors are view independent.
 *
 * Each splat is drawn as a screen space ellipse (EWA projection of the 3D covariance)
 * expanded from a point in a geometry shader. The geometries are created by
 * PCLPointCloudNode::dispatch, which distributes the splats into its sub cube/LOD
 * structure and keeps the indices sorted back to front.
 */
namespace GaussianSplats {

    /**
     * @brief true if the cloud contains all fields needed to render it as gaussian splats
     */
    bool isSplatCloud(const pcl::PCLPointCloud2& cloud);

    /**
     * @brief byte offsets of the splat fields within one point of a PCLPointCloud2
     */
    struct FieldOffsets {
        int offsets[14];
    };

    /**
     * @brief collect the splat field offsets, returns false if a field is missing (not a splat cloud)
     */
    bool getFieldOffsets(const pcl::PCLPointCloud2& cloud, FieldOffsets& fields);

    /**
     * @brief render data of a single splat, converted from the raw PLY fields
     */
    struct Splat {
        bool valid;        ///< false for fully transparent or non finite splats, skip those
        osg::Vec3f covA;   ///< covariance xx, xy, xz
        osg::Vec3f covB;   ///< covariance yy, yz, zz
        osg::Vec4ub color; ///< SH degree 0 color and opacity
    };

    /**
     * @brief convert the raw fields of one point (at pointData) into render data
     */
    Splat extract(const uint8_t* pointData, const FieldOffsets& fields);

    /**
     * @brief create a geometry that draws the splats as points expanded in the splat shaders,
     * including all required state (program, blending, depth test without write, transparent bin).
     *
     * The shaders expect the uniforms u_viewport (vec2, viewport size in pixels) and
     * u_scale (float, splat size multiplier) to be provided by a parent node.
     * The caller fills the arrays and keeps indices sorted back to front.
     */
    osg::ref_ptr<osg::Geometry> createGeometry(osg::Vec3Array* positions,
                                               osg::Vec3Array* covA,
                                               osg::Vec3Array* covB,
                                               osg::Vec4ubArray* colors,
                                               osg::DrawElementsUInt* indices);

    /**
     * @brief indices of centers ordered back to front along viewdir (16 bit counting sort)
     */
    std::vector<GLuint> sortBackToFront(const osg::Vec3Array& centers, const osg::Vec3f& viewdir);

}
