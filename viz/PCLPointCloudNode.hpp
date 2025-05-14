#pragma once

#include <list>

#include <osg/LOD>
#include <osg/Geode>
#include <osg/Geometry>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

class PCLPointCloudNode : public osg::Group {
 public:
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
            // replace old set, they are still referenced in the group
            geode = new osg::Geode;
            color = new osg::Vec4Array;
            pointsOSG = new osg::Vec3Array;

            drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, pointsOSG->size() );
            pointGeom = new osg::Geometry;


            pointGeom->setColorArray(color);
            pointGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            pointGeom->setVertexArray(pointsOSG);
            pointGeom->addPrimitiveSet(drawArrays.get());
            geode->addDrawable(pointGeom.get());

            //add new geode to graph
            group->addChild(geode);
        }

        osg::ref_ptr<osg::Group> getRootNode() {
            return group;
        }

        osg::ref_ptr<osg::Vec3Array> getPoints() {
            return pointsOSG;
        }

        osg::ref_ptr<osg::Vec4Array> getColors() {
            return color;
        }

        osg::ref_ptr<osg::Geometry> getPointGeom() {
            return pointGeom;
        }

        osg::ref_ptr<osg::DrawArrays> getDrawArrays() {
            return drawArrays;
        }

        float downsample;
        int downsampleSkip;

     private:
        osg::ref_ptr<osg::Group> group;

        osg::ref_ptr<osg::Geode> geode;

        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;

        
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

    void dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample);

    void dispatch(const pcl::PCLPointCloud2& pcl_cloud, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample);

    LodLevel* getLodLevel(const size_t& index) {
        return subClouds->get(0,0,0).getLodLevel(index);
    }

 protected:

    void dispatch(const pcl::PointCloud<pcl::PointXYZRGBA>& pcl_cloud, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, double maxz, float downsample);
    void dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud, double maxz, float downsample);
    void dispatch(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, const osg::Vec4f& default_feature_color, double maxz, float downsample);
    void dispatch(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const osg::Vec4f& default_feature_color, double maxz, float downsample);


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
    template <class POINTTYPE> void traversePoints(const pcl::PointCloud<POINTTYPE>& pc, const float& downsample, const float& maxz, std::function <void(const POINTTYPE&, LodLevel&, POINTTYPE &, POINTTYPE &)> cb) {

        POINTTYPE minPt, maxPt;
        pcl::getMinMax3D (pc, minPt, maxPt);

        double sizex = maxPt.x - minPt.x;
        double cubesizex = (double)sizex/(double)(subClouds->xsize-1);

        double sizey = maxPt.y - minPt.y;
        double cubesizey = (double)sizey/(double)(subClouds->ysize-1);

        double sizez = maxPt.z - minPt.z;
        double cubesizez = (double)sizez/(double)(subClouds->zsize-1);

        for (auto& lodCube : *subClouds)
        {
            for (auto &lodlevel: lodCube->getLodLevels())
            {
                lodlevel.getPoints()->clear();
                lodlevel.getColors()->clear();
                lodlevel.downsampleSkip = lodlevel.downsample / downsample;
            }
        }

        for(size_t i = 0; i < pc.size(); ++i)
        {
            if (pc[i].z < maxz)
            {
                // ((pc[i].x-minPt.x) / sizex) percentage on x axis the factor (1.3) make the render cubes smaller
                size_t xindex = ((pc[i].x-minPt.x) / sizex) * (subClouds->xsize-1);
                size_t yindex = ((pc[i].y-minPt.y) / sizey) * (subClouds->ysize-1);
                size_t zindex = ((pc[i].z-minPt.z) / sizez) * (subClouds->zsize-1);

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
                        cb(point, lodlevel, minPt, maxPt);
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
    }



 private:

    // std::vector<LodLevel> lodlevels;
    std::shared_ptr<SubClouds> subClouds;
    size_t subCloudsX;
    size_t subCloudsY;
    size_t subCloudsZ;
    float cycle_color_interval;
    
    osg::ref_ptr<osg::Vec3Array> osg_points;
    osg::ref_ptr<osg::Vec4Array> osg_colors;


};
