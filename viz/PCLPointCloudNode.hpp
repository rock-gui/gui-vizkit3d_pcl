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
    // class LodLevel {
    //  public:
    //     LodLevel(){
    //         group = new osg::Group;
            
    //         addPrimitiveSet();
    //     }

    //     /**
    //      * @brief 
    //      * creates a new arraw set and moves "active" variables back in list (they are in the osg graph, no need to access anymore)
    //      */
    //     void addPrimitiveSet() {
    //         geode.push_back(new osg::Geode);
    //         color.push_back(new osg::Vec4Array);
    //         pointsOSG.push_back(new osg::Vec3Array);

    //         drawArrays.push_back(new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, pointsOSG.back()->size() ));
    //         pointGeom.push_back(new osg::Geometry);


    //         pointGeom.back()->setColorArray(color.back());
    //         pointGeom.back()->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    //         pointGeom.back()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    //         pointGeom.back()->setVertexArray(pointsOSG.back());
    //         pointGeom.back()->addPrimitiveSet(drawArrays.back().get());

    //         geode.back()->addDrawable(pointGeom.back().get());

    //         //add new geode to graph
    //         group->addChild(geode.back());
    //     }

    //     osg::ref_ptr<osg::Group> getRootNode() {
    //         return group;
    //     }

    //     // osg::ref_ptr<osg::Geode> getGeode() {
    //     //     return geode.back();
    //     // }

    //     osg::ref_ptr<osg::Vec3Array> getPoints() {
    //         return pointsOSG.back();
    //     }

    //     osg::ref_ptr<osg::Vec4Array> getColors() {
    //         return color.back();
    //     }

    //     osg::ref_ptr<osg::Geometry> getPointGeom() {
    //         return pointGeom.back();
    //     }

    //     osg::ref_ptr<osg::DrawArrays> getDrawArrays() {
    //         return drawArrays.back();
    //     }

    //     float downsample;
    //     int downsampleSkip;

    //  private:
    //     osg::ref_ptr<osg::Group> group;
    //     std::list<osg::ref_ptr<osg::Geode>> geode;

    //     std::list<osg::ref_ptr<osg::Vec3Array>> pointsOSG;
    //     std::list<osg::ref_ptr<osg::DrawArrays>> drawArrays;
    //     std::list<osg::ref_ptr<osg::Geometry>> pointGeom;
    //     std::list<osg::ref_ptr<osg::Vec4Array>> color;

        
    // };
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

        // osg::ref_ptr<osg::Geode> getGeode() {
        //     return geode.back();
        // }

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
        // return &(lodlevels[index]);
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
                //todo: move to xyz loop below or each lodCube knows its index
                lodlevel.getPoints()->clear();
                lodlevel.getColors()->clear();
                // int cloudsize = (pc.size() * downsample) / subClouds->size(); // just an estimate, boxes might have lower/higer points count but still speeds it up
                // lodlevel.pointsOSG->reserve(cloudsize);
                // lodlevel.color->reserve(cloudsize);
                lodlevel.downsampleSkip = lodlevel.downsample / downsample;
            }
        }

        // for (int z = 0; z < subClouds->zsize;++z) {
        //     for (int y = 0; y < subClouds->ysize;++y) {
        //         for (int x = 0; x < subClouds->xsize;++x) {
        //             LODCube& cube = subClouds->get(x,y,z);
        //             cube.lodnode->setCenterMode(osg::LOD::USER_DEFINED_CENTER);
        //             // osg::Vec3d center (x*cubesizex+cubesizex/2.0,y*cubesizey+cubesizey/2.0,z*cubesizez+cubesizez/2.0);
        //             osg::Vec3d center (0,0,0);
        //             cube.lodnode->setCenter(center);
        //             // cube.lodnode->setRadius(cubesizex); // todo, use max?
        //             // cube.lodnode->setRangeMode()
        //             // cube.pose->setPosition(osg::Vec3d(x*cubesizex+cubesizex/2.0,y*cubesizey+cubesizey/2.0,z*cubesizez+cubesizez/2.0));
        //         }
        //     }
        // }

        for(size_t i = 0; i < pc.size(); ++i)
        {
            if (pc[i].z < maxz) {

                //add skew to have smaller cubes at center
                // double xfactor = (pc[i].x/maxPt.x);
                // double yfactor = (pc[i].y/maxPt.y);
                // double zfactor = (pc[i].z/maxPt.z);

                // const double shiftfactor = 1.9;

                // ((pc[i].x-minPt.x) / sizex) percentage on x axis the factor (1.3) make the render cubes smaller
                size_t xindex = ((pc[i].x-minPt.x) / sizex) * (subClouds->xsize-1);
                size_t yindex = ((pc[i].y-minPt.y) / sizey) * (subClouds->ysize-1);
                size_t zindex = ((pc[i].z-minPt.z) / sizez) * (subClouds->zsize-1);
                

                // if (xindex > subClouds->xsize) {
                //     xindex = subClouds->xsize;
                // }
                // if (yindex > subClouds->ysize) {
                //     yindex = subClouds->ysize;
                // }
                // if (zindex > subClouds->zsize) {
                //     zindex = subClouds->zsize;
                // }

                LODCube &cube = subClouds->get(xindex, yindex, zindex);
                std::vector<LodLevel> &lodlevels = cube.getLodLevels();
                // osg::Vec3d cubepos = cube.pose->getPosition();

                
                for (auto &lodlevel: lodlevels)
                {
                    if (i % lodlevel.downsampleSkip == 0) {

                        POINTTYPE point = pc[i];
                        // point.x -= cubepos.x();
                        // point.y -= cubepos.y();
                        // point.z -= cubepos.z();
                        if (lodlevel.getPoints()->size() >= 10000) {
                            // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
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
                //shrink allocated arrays
                // lodlevel.pointsOSG->reserve(lodlevel.pointsOSG->size());
                // lodlevel.color->reserve(lodlevel.color->size());
            }
        }
        // update cubes in osg graph
        subClouds->update();

        // for (int z = 0; z < subClouds->zsize;++z) {
        //     for (int y = 0; y < subClouds->ysize;++y) {
        //         for (int x = 0; x < subClouds->xsize;++x) {
        //             LODCube& lods = subClouds->get(x,y,z);
        //             printf("%i,%i,%i size: %li\n", x,y,z, lods.getLodLevel(0)->pointsOSG->size());
        //         }
        //     }
        // }
        
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
