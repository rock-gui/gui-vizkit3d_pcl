#pragma once

#include <cstdint>
#include <future>
#include <list>
#include <memory>
#include <vector>

#include <osg/BoundingBox>
#include <osg/LOD>
#include <osg/Point>
#include <osg/Geode>
#include <osg/Geometry>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#include "GaussianSplats.hpp"

class PCLPointCloudNode : public osg::Group {
 public:

    struct DispatchConfig {
        DispatchConfig():
            default_feature_color(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)),
            useHeightColoring(false),
            downsample(1),
            cut(false),
            minz(std::numeric_limits<double>::min()),
            maxz(std::numeric_limits<double>::max()),
            renderSplats(true)
        {}

        DispatchConfig(const osg::Vec4f& default_feature_color, const bool useHeightColoring, const bool cut, const double minz, const double maxz, const double downsample):
            default_feature_color(default_feature_color),
            useHeightColoring(useHeightColoring),
            downsample(downsample),
            cut(cut),
            minz(minz),
            maxz(maxz),
            renderSplats(true)
        {}

        osg::Vec4f default_feature_color;
        bool useHeightColoring;
        double downsample;
        bool cut;
        double minz, maxz;
        /**
         * render clouds containing gaussian splat fields (scale_*, rot_*, opacity, f_dc_*)
         * as splats instead of points
         */
        bool renderSplats;

    };

    /**
     * @brief one splat geometry (one lod level of one sub cloud cube),
     * tracked for back to front sorting
     */
    struct SplatChunk {
        osg::ref_ptr<osg::Vec3Array> centers;
        osg::ref_ptr<osg::DrawElementsUInt> indices;
        osg::ref_ptr<osg::Geometry> geometry;
        osg::Vec3f cube;         ///< grid coordinates of the sub cloud cube this chunk belongs to
        osg::BoundingBox bound;  ///< bound of the splat centers
        float maxRadius;         ///< upper bound on the splat extents (at splat scale 1)
    };

    class LodLevel {
     public:
        LodLevel(): maxSplatRadius(0), splatMode(false) {
            group = new osg::Group;

            addPrimitiveSet();
        }

        /**
         * @brief switch between rendering plain points and gaussian splats,
         * drops all current primitive sets and starts with a fresh one
         */
        void setSplatMode(const bool& enable) {
            splatMode = enable;
            geode.clear();
            color.clear();
            pointsOSG.clear();
            drawArrays.clear();
            pointGeom.clear();
            splatCovA.clear();
            splatCovB.clear();
            splatColors.clear();
            splatIndices.clear();
            maxSplatRadius = 0;
            group->removeChildren(0, group->getNumChildren());
            addPrimitiveSet();
        }

        bool isSplatMode() const {
            return splatMode;
        }

        /**
         * @brief
         * creates a new arraw set and moves "active" variables back in list (they are in the osg graph, no need to access anymore)
         */
        void addPrimitiveSet() {
            geode.push_back(new osg::Geode);
            pointsOSG.push_back(new osg::Vec3Array);

            if (splatMode) {
                splatCovA.push_back(new osg::Vec3Array);
                splatCovB.push_back(new osg::Vec3Array);
                splatColors.push_back(new osg::Vec4ubArray);
                splatIndices.push_back(new osg::DrawElementsUInt(osg::PrimitiveSet::POINTS));

                pointGeom.push_back(GaussianSplats::createGeometry(pointsOSG.back(), splatCovA.back(),
                                                                   splatCovB.back(), splatColors.back(),
                                                                   splatIndices.back()));
            } else {
                color.push_back(new osg::Vec4Array);
                drawArrays.push_back(new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, pointsOSG.back()->size() ));
                pointGeom.push_back(new osg::Geometry);


                pointGeom.back()->setColorArray(color.back());
                pointGeom.back()->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
                pointGeom.back()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
                pointGeom.back()->setVertexArray(pointsOSG.back());
                pointGeom.back()->addPrimitiveSet(drawArrays.back().get());
            }

            geode.back()->addDrawable(pointGeom.back().get());

            //add new geode to graph
            group->addChild(geode.back());
        }

        /**
         * @brief clear the arrays of the active (last) primitive set
         */
        void clearData() {
            pointsOSG.back()->clear();
            if (splatMode) {
                splatCovA.back()->clear();
                splatCovB.back()->clear();
                splatColors.back()->clear();
                splatIndices.back()->clear();
                maxSplatRadius = 0;
            } else {
                color.back()->clear();
            }
        }

        /**
         * @brief set the number of points to draw from the active (last) primitive set,
         * in splat mode the index array is extended (identity, sorted back to front later)
         */
        void setDrawCount(const size_t& count) {
            if (splatMode) {
                osg::DrawElementsUInt* indices = splatIndices.back();
                const size_t oldSize = indices->size();
                indices->resize(count);
                for (size_t i = oldSize; i < count; ++i) {
                    (*indices)[i] = i;
                }
                indices->dirty();
            } else {
                drawArrays.back()->setCount(count);
            }
        }

        /**
         * @brief collect the non empty splat geometries for back to front sorting
         */
        void collectSplatChunks(std::vector<SplatChunk>& chunks, const osg::Vec3f& cube) {
            if (!splatMode) {
                return;
            }
            auto points = pointsOSG.begin();
            auto indices = splatIndices.begin();
            auto geom = pointGeom.begin();
            for (; points != pointsOSG.end(); ++points, ++indices, ++geom) {
                if ((*points)->size()) {
                    SplatChunk chunk;
                    chunk.centers = *points;
                    chunk.indices = *indices;
                    chunk.geometry = *geom;
                    chunk.cube = cube;
                    chunk.maxRadius = maxSplatRadius;
                    for (const osg::Vec3f& center : **points) {
                        chunk.bound.expandBy(center);
                    }
                    chunks.push_back(chunk);
                }
            }
        }

        osg::ref_ptr<osg::Group> getRootNode() {
            return group;
        }

        // osg::ref_ptr<osg::Geode> getGeode() {
        //     return geode.back();
        // }

        osg::ref_ptr<osg::Vec3Array> getPoints() {
            return pointsOSG.back();
        }

        osg::ref_ptr<osg::Vec4Array> getColors() {
            return color.back();
        }

        // splat mode only: covariance xx, xy, xz
        osg::ref_ptr<osg::Vec3Array> getSplatCovA() {
            return splatCovA.back();
        }

        // splat mode only: covariance yy, yz, zz
        osg::ref_ptr<osg::Vec3Array> getSplatCovB() {
            return splatCovB.back();
        }

        // splat mode only: SH degree 0 color and opacity
        osg::ref_ptr<osg::Vec4ubArray> getSplatColors() {
            return splatColors.back();
        }

        osg::ref_ptr<osg::Geometry> getPointGeom() {
            return pointGeom.back();
        }

        osg::ref_ptr<osg::DrawArrays> getDrawArrays() {
            return drawArrays.back();
        }

        float downsample;
        int downsampleSkip;
        // splat mode only: upper bound on the splat extents (at splat scale 1)
        float maxSplatRadius;

        void setPointSize(const double &size) {
            if (splatMode) {
                // splat size is given by the trained covariances (see setSplatScale)
                return;
            }
            osg::ref_ptr<osg::Point> pt = new osg::Point(size);
            for (auto& geom : pointGeom) {
                geom->getOrCreateStateSet()->setAttribute(pt, osg::StateAttribute::ON);
            }
        }

        double getPointSize();

     private:
        osg::ref_ptr<osg::Group> group;
        std::list<osg::ref_ptr<osg::Geode>> geode;

        std::list<osg::ref_ptr<osg::Vec3Array>> pointsOSG;
        std::list<osg::ref_ptr<osg::DrawArrays>> drawArrays;
        std::list<osg::ref_ptr<osg::Geometry>> pointGeom;
        std::list<osg::ref_ptr<osg::Vec4Array>> color;

        // splat mode arrays (used instead of drawArrays/color)
        std::list<osg::ref_ptr<osg::Vec3Array>> splatCovA;
        std::list<osg::ref_ptr<osg::Vec3Array>> splatCovB;
        std::list<osg::ref_ptr<osg::Vec4ubArray>> splatColors;
        std::list<osg::ref_ptr<osg::DrawElementsUInt>> splatIndices;

        bool splatMode;
    };

    class LODCube {
     public:
        LODCube() {
            lodnode = new osg::LOD;
            // pose = new osg::PositionAttitudeTransform;
            // pose->addChild(lodnode);
        }

        void addLodLevel(const float& from, const float& to, const float& downsample) {
            LodLevel level;
            level.downsample = downsample;


            lodlevels.push_back(level);

            lodnode->addChild(level.getRootNode(), from, to);
        }

        LodLevel* getLodLevel(const size_t& index){
            return &(lodlevels[index]);
        }

        std::vector<LodLevel>& getLodLevels() {
            return lodlevels;
        }

        // osg::ref_ptr<osg::PositionAttitudeTransform> pose;
        osg::ref_ptr<osg::LOD> lodnode;

     private:

        std::vector<LodLevel> lodlevels;

    };

    /**
     * @brief structure to split a large poitnt cloud into sub-cubes
     *
     */
    class SubClouds : public std::vector < std::shared_ptr<LODCube> > {
     public:
        SubClouds(const size_t &xsize, const size_t &ysize, const size_t &zsize): xsize(xsize), ysize(ysize), zsize(zsize) {
            osgGroup = new osg::Group;
            this->resize(xsize*ysize*zsize);
            for (auto& lodCube : *this)
            {
                lodCube = std::make_shared<LODCube>();
            }
        }

        void update(){
            osgGroup->removeChildren(0, osgGroup->getNumChildren());
            for (auto& lodCube : *this)
            {
                //only add child if it has contents
                if (lodCube->getLodLevel(0)->getPoints()->size()) {
                    osgGroup->addChild(lodCube->lodnode);
                }

            }
        }

        size_t index(const size_t& x, const size_t& y, const size_t& z) const {
            return x + y*ysize + z*ysize*zsize;
        }

        LODCube &get(int x, int y, int z) {
            return *((*this)[index(x, y, z)]);
        }

        size_t xsize,ysize,zsize;
        osg::ref_ptr<osg::Group> osgGroup;

     private:
        SubClouds(){}

    };

    PCLPointCloudNode();
    virtual ~PCLPointCloudNode();

    void clear() {
        this->removeChildren(0, this->getNumChildren());
        // lodlevels.clear();
        subClouds = std::make_shared<SubClouds>(subCloudsX,subCloudsY,subCloudsZ);
        addChild(subClouds->osgGroup);
        splatChunks.clear();
        ++splatGeneration;
        splatsSorted = false;
    }

    void setSubCloudBoxes(const size_t &xsize, const size_t &ysize, const size_t &zsize) {
        subCloudsX = xsize;
        subCloudsY = ysize;
        subCloudsZ = zsize;
    }

    void addLodLevel(const float& from, const float& to, const float& downsample);


    void dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const DispatchConfig& config, osg::Camera* cam = nullptr);

    void dispatch(const pcl::PCLPointCloud2& pcl_cloud, const DispatchConfig& config, bool show_color, bool show_intensity, osg::Camera* cam = nullptr);

    LodLevel* getDefaultLodLevel() {
        return subClouds->get(0,0,0).getLodLevel(0);
    }

    void setPointSize(const double & size);
    double getPointSize();

    void setColorInterval(const float& interval);

    /**
     * @brief multiplier on the gaussian splat size (1.0 is the trained size)
     */
    void setSplatScale(const float& scale);
    float getSplatScale() const;

    /**
     * @brief called on cull traversal with the current model view matrix and viewport size,
     * keeps the gaussian splats sorted back to front (sorting runs in a background thread)
     * and the sub cloud cubes ordered back to front
     */
    void updateView(const osg::Matrix& modelview, float viewportWidth, float viewportHeight);

 protected:

    void enbableHeightColorShader(osg::Node* parent, osg::Camera* cam);

    /**
     * @brief dispatch a cloud with gaussian splat fields into the sub cube/LOD structure,
     * rendered as splats instead of points.
     *
     * Each cube/lod level is one splat geometry so osg can cull and lod-switch them, splats
     * are sorted back to front within each geometry and the cube geometries are drawn in
     * an exact back to front painter order (see updateView). Splats whose footprint crosses
     * a cube border can still blend in the wrong order (visible at grazing view angles).
     */
    void dispatchSplats(const pcl::PCLPointCloud2& pcl_cloud, const DispatchConfig& config);

    void applySortResults(std::vector<std::vector<GLuint>>&& orders);

    /**
     * @brief expand the bound of each splat geometry by the splat extents, so cubes are not
     * frustum culled while their splats still reach into the view (centers are offscreen,
     * but the splats themselves are large)
     */
    void applySplatBounds();

    void dispatch(const pcl::PointCloud<pcl::PointXYZRGBA>& pcl_cloud, const DispatchConfig& config, bool show_color, bool show_intensity);
    void dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud, const DispatchConfig& config);
    void dispatch(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, const DispatchConfig& config);
    // void dispatch(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const DispatchConfig& config);

    /**
     * @brief template to handle points of the input point cloud, calls the callback for each point, providing the structure to put the point into
     * cb has to handle/compute points and color information and add it to the osg structure,
     * it also receives the index of the point in the input cloud
     *
     * @tparam POINTTYPE
     * @param pc
     * @param downsample
     * @param maxz
     * @param cb
     * @param splatMode switch the lod levels to gaussian splat geometries
     */
    template <class POINTTYPE> std::pair<POINTTYPE, POINTTYPE> traversePoints(const pcl::PointCloud<POINTTYPE>& pc, const DispatchConfig& config, std::function <void(const POINTTYPE&, const size_t&, LodLevel&)> cb, bool calcMinMaxX = false, bool calcMinMaxY = false, bool calcMinMaxZ = false, bool splatMode = false) {

        // float nan = std::numeric_limits<float>::quiet_NaN();
        // POINTTYPE minPt(nan,nan,nan);
        // POINTTYPE maxPt(nan,nan,nan);
        POINTTYPE minPt, maxPt;

        if (pc.size() == 0) {
            return {minPt,maxPt};
        }

        double sizex = 1;
        double sizey = 0;
        double sizez = 0;

        // when we don't need to sort, we will calc minmax later for performance reasons need to cals regardless of option
        if (subClouds->size() > 1) {

            // POINTTYPE minPt, maxPt;
            pcl::getMinMax3D (pc, minPt, maxPt);

            sizex = maxPt.x - minPt.x;
            sizey = maxPt.y - minPt.y;
            sizez = maxPt.z - minPt.z;
        }

        if (splatMode) {
            // remember the point to cube grid mapping, used to draw the cubes back to front
            if (subClouds->size() > 1) {
                splatGridMin.set(minPt.x, minPt.y, minPt.z);
                splatGridScale.set(sizex > 0 ? (subClouds->xsize - 1) / sizex : 0.0,
                                   sizey > 0 ? (subClouds->ysize - 1) / sizey : 0.0,
                                   sizez > 0 ? (subClouds->zsize - 1) / sizez : 0.0);
            } else {
                splatGridMin.set(0, 0, 0);
                splatGridScale.set(0, 0, 0);
            }
        }

        for (auto& lodCube : *subClouds)
        {
            for (auto &lodlevel: lodCube->getLodLevels())
            {
                if (lodlevel.isSplatMode() != splatMode) {
                    lodlevel.setSplatMode(splatMode);
                }
                lodlevel.clearData();
                lodlevel.downsampleSkip = lodlevel.downsample / config.downsample;
            }
        }



        for(size_t i = 0; i < pc.size(); ++i)
        {
            if (!std::isfinite(pc[i].x) || !std::isfinite(pc[i].y) || !std::isfinite(pc[i].z)) {
                continue;
            }
            if (!config.cut || (config.cut && pc[i].z > config.minz && pc[i].z < config.maxz))
            {
                if (subClouds->size() == 1) {
                    if (calcMinMaxX) {
                        if (minPt.x > pc[i].x) {
                            minPt.x = pc[i].x;
                        }
                        if (maxPt.x < pc[i].x) {
                            maxPt.x = pc[i].x;
                        }
                    }
                    if (calcMinMaxY) {
                        if (minPt.y > pc[i].y) {
                            minPt.y = pc[i].y;
                        }
                        if (maxPt.y < pc[i].y) {
                            maxPt.y = pc[i].y;
                        }
                    }
                    if (calcMinMaxZ) {
                        if (minPt.z > pc[i].z) {
                            minPt.z = pc[i].z;
                        }
                        if (maxPt.z < pc[i].z) {
                            maxPt.z = pc[i].z;
                        }
                    }
                }

                // ((pc[i].x-minPt.x) / sizex) percentage on x axis the factor (1.3) make the render cubes smaller
                size_t xindex = 0;
                size_t yindex = 0;
                size_t zindex = 0;
                if (subClouds->size() > 1) {
                    // when subclouds are uses, we need to calculate the index of the subcloud based on point location
                    xindex = ((pc[i].x-minPt.x) / sizex) * (subClouds->xsize-1);
                    yindex = ((pc[i].y-minPt.y) / sizey) * (subClouds->ysize-1);
                    zindex = ((pc[i].z-minPt.z) / sizez) * (subClouds->zsize-1);
                }

                LODCube &cube = subClouds->get(xindex, yindex, zindex);
                std::vector<LodLevel> &lodlevels = cube.getLodLevels();

                for (auto &lodlevel: lodlevels)
                {
                    if (i % lodlevel.downsampleSkip == 0)
                    {
                        POINTTYPE point = pc[i];
                        // splats must stay in a single geometry per level: they are alpha blended
                        // in index order, splitting them would break the back to front sorting
                        if (!splatMode && lodlevel.getPoints()->size() >= 10000) {
                            // 10000 points per primitive set is advised fo better performance, so create a new one
                            // to put new points into it
                            lodlevel.setDrawCount(lodlevel.getPoints()->size());
                            lodlevel.addPrimitiveSet();
                        }
                        // at this point we need global minmax of the cloud
                        cb(point, i, lodlevel);
                    }
                }
            }
        }

        //after sorting, set sizes
        for (auto& lodCube : *subClouds)
        {
            for (auto &lodlevel: lodCube->getLodLevels())
            {
                lodlevel.setDrawCount(lodlevel.getPoints()->size());
            }
        }
        // update cubes in osg graph
        subClouds->update();

        // collect the splat geometries for back to front sorting (empty in point mode)
        splatChunks.clear();
        if (splatMode) {
            for (size_t z = 0; z < subClouds->zsize; ++z) {
                for (size_t y = 0; y < subClouds->ysize; ++y) {
                    for (size_t x = 0; x < subClouds->xsize; ++x) {
                        for (auto &lodlevel: subClouds->get(x, y, z).getLodLevels()) {
                            lodlevel.collectSplatChunks(splatChunks, osg::Vec3f(x, y, z));
                        }
                    }
                }
            }
            applySplatBounds();
        }
        ++splatGeneration;
        splatsSorted = false;

        if (config.cut) {
            // if cut (and minmax was calced before cutting, we need to override)
            // does not really matter
            minPt.z = config.minz;
            maxPt.z = config.maxz;
        }
        return {minPt,maxPt};
    }



 private:

    // std::vector<LodLevel> lodlevels;
    std::shared_ptr<SubClouds> subClouds;
    size_t subCloudsX;
    size_t subCloudsY;
    size_t subCloudsZ;
    float cycle_color_interval;

    double pointsize;

    osg::ref_ptr<osg::Program> program;
    osg::ref_ptr<osg::Shader> fShader;
    osg::ref_ptr<osg::Shader> vShader;
    osg::ref_ptr<osg::Uniform> cycleColorIntervalUniform;

    // gaussian splat state: uniforms for the splat shaders (unused by point rendering)
    osg::ref_ptr<osg::Uniform> viewportUniform;
    osg::ref_ptr<osg::Uniform> splatScaleUniform;
    float splatScale;

    // splat geometries of the current cloud, shared with the sort thread
    std::vector<SplatChunk> splatChunks;
    // mapping from local coordinates to sub cloud cube grid coordinates (see traversePoints)
    osg::Vec3f splatGridMin;
    osg::Vec3f splatGridScale;
    // incremented whenever splatChunks is rebuilt, to discard outdated sort results
    unsigned int splatGeneration;
    unsigned int sortGeneration;
    std::future<std::vector<std::vector<GLuint>>> sortResult;
    osg::Vec3f sortedViewDir;
    osg::Vec3f requestedViewDir;
    bool splatsSorted;

};
