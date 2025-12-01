#pragma once

#include <list>

#include <osg/LOD>
#include <osg/Point>
#include <osg/Geode>
#include <osg/Geometry>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

class PCLPointCloudNode : public osg::Group {
 public:

    struct DispatchConfig {
        DispatchConfig():
            default_feature_color(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)),
            useHeightColoring(false),
            downsample(1),
            cut(false),
            minz(std::numeric_limits<double>::min()),
            maxz(std::numeric_limits<double>::max())
        {}

        DispatchConfig(const osg::Vec4f& default_feature_color, const bool useHeightColoring, const bool cut, const double minz, const double maxz, const double downsample):
            default_feature_color(default_feature_color),
            useHeightColoring(useHeightColoring),
            downsample(downsample),
            cut(cut),
            minz(minz),
            maxz(maxz)
        {}

        osg::Vec4f default_feature_color;
        bool useHeightColoring;
        double downsample;
        bool cut;
        double minz, maxz;
        
    };

    class LodLevel {
     public:
        LodLevel(){
            group = new osg::Group;
            
            addPrimitiveSet();
        }

        /**
         * @brief 
         * creates a new arraw set and moves "active" variables back in list (they are in the osg graph, no need to access anymore)
         */
        void addPrimitiveSet() {
            geode.push_back(new osg::Geode);
            color.push_back(new osg::Vec4Array);
            pointsOSG.push_back(new osg::Vec3Array);

            drawArrays.push_back(new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, pointsOSG.back()->size() ));
            pointGeom.push_back(new osg::Geometry);


            pointGeom.back()->setColorArray(color.back());
            pointGeom.back()->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            pointGeom.back()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            pointGeom.back()->setVertexArray(pointsOSG.back());
            pointGeom.back()->addPrimitiveSet(drawArrays.back().get());

            geode.back()->addDrawable(pointGeom.back().get());

            //add new geode to graph
            group->addChild(geode.back());
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

        osg::ref_ptr<osg::Geometry> getPointGeom() {
            return pointGeom.back();
        }

        osg::ref_ptr<osg::DrawArrays> getDrawArrays() {
            return drawArrays.back();
        }

        float downsample;
        int downsampleSkip;

        void setPointSize(const double &size) {
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

        LODCube &get(int x, int y, int z) {
            size_t index = x + y*ysize + z*ysize*zsize;
            return *((*this)[index]);
        }

        size_t xsize,ysize,zsize;
        osg::ref_ptr<osg::Group> osgGroup;

     private:
        SubClouds(){}

    };

    PCLPointCloudNode();

    void clear() {
        this->removeChildren(0, this->getNumChildren());
        // lodlevels.clear();
        subClouds = std::make_shared<SubClouds>(subCloudsX,subCloudsY,subCloudsZ);
        addChild(subClouds->osgGroup);
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

 protected:
    
    void enbableHeightColorShader(osg::Node* parent, osg::Camera* cam);

    void dispatch(const pcl::PointCloud<pcl::PointXYZRGBA>& pcl_cloud, const DispatchConfig& config, bool show_color, bool show_intensity);
    void dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud, const DispatchConfig& config);
    void dispatch(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, const DispatchConfig& config);
    // void dispatch(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const DispatchConfig& config);

    /**
     * @brief template to handle points of the input point cloud, calls the callback for each point, providing the structure to put the point into
     * cb has to handle/compute points and color information and add it to the osg structure
     * 
     * @tparam POINTTYPE 
     * @param pc 
     * @param downsample 
     * @param maxz 
     * @param cb 
     */
    template <class POINTTYPE> std::pair<POINTTYPE, POINTTYPE> traversePoints(const pcl::PointCloud<POINTTYPE>& pc, const DispatchConfig& config, std::function <void(const POINTTYPE&, LodLevel&)> cb, bool calcMinMaxX = false, bool calcMinMaxY = false, bool calcMinMaxZ = false) {

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

        for (auto& lodCube : *subClouds)
        {
            for (auto &lodlevel: lodCube->getLodLevels())
            {
                lodlevel.getPoints()->clear();
                lodlevel.getColors()->clear();
                lodlevel.downsampleSkip = lodlevel.downsample / config.downsample;
            }
        }

        

        for(size_t i = 0; i < pc.size(); ++i)
        {
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
                        if (lodlevel.getPoints()->size() >= 10000) {
                            // 10000 points per primitive set is advised fo better performance, so create a new one
                            // to put new points into it
                            lodlevel.getDrawArrays()->setCount(lodlevel.getPoints()->size());
                            lodlevel.addPrimitiveSet();
                        }
                        // at this point we need global minmax of the cloud
                        cb(point, lodlevel);
                    }
                }
            }
        }

        //after sorting, set sizes
        for (auto& lodCube : *subClouds)
        {
            for (auto &lodlevel: lodCube->getLodLevels())
            {
                lodlevel.getDrawArrays()->setCount(lodlevel.getPoints()->size());
            }
        }
        // update cubes in osg graph
        subClouds->update();

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

};
