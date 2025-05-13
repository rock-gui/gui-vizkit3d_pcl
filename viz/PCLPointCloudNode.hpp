#pragma once

#include <osg/LOD>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

class PCLPointCloudNode : public osg::Group {
 public:
    struct LodLevel {
        float downsample;
        int downsampleSkip;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
        osg::ref_ptr<osg::Geode> geode;
    };


    class LODCube {
     public:
        LODCube() {
            lodnode = new osg::LOD;
            pose = new osg::PositionAttitudeTransform;
            pose->addChild(lodnode);
        }

        void addLodLevel(const float& from, const float& to, const float& downsample) {
            LodLevel level;
            level.downsample = downsample;
            level.pointGeom = new osg::Geometry;
            level.pointsOSG = new osg::Vec3Array;
            level.pointGeom->setVertexArray(level.pointsOSG);
            level.color = new osg::Vec4Array;
            level.pointGeom->setColorArray(level.color);
            level.pointGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            level.pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            level.drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, level.pointsOSG->size() );
            level.pointGeom->addPrimitiveSet(level.drawArrays.get());
            level.geode = new osg::Geode;
            level.geode->addDrawable(level.pointGeom.get());
            
            lodlevels.push_back(level);

            lodnode->addChild(level.geode, from, to);
        }

        LodLevel* getLodLevel(const size_t& index){
            return &(lodlevels[index]);
        }
    
        std::vector<LodLevel>& getLodLevels() {
            return lodlevels;
        }

        osg::ref_ptr<osg::PositionAttitudeTransform> pose;

     private:
        osg::ref_ptr<osg::LOD> lodnode;
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
                //only add child it has content
                if (lodCube->getLodLevel(0)->pointsOSG) {
                    osgGroup->addChild(lodCube->pose);
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


    template <class POINTTYPE> void traversePoints(const pcl::PointCloud<POINTTYPE>& pc, const float& downsample, const float& maxz, std::function <void(const pcl::PointCloud<POINTTYPE>&, LodLevel&, const size_t&, POINTTYPE &, POINTTYPE &)> cb) {

        for (auto& lodCube : *subClouds)
        {
            for (auto &lodlevel: lodCube->getLodLevels())
            {
                lodlevel.pointsOSG->clear();
                lodlevel.color->clear();
                // int cloudsize = (pc.size() * downsample) / subClouds->size(); // just an estimate, boxes might have lower/higer points count but still speeds it up
                // lodlevel.pointsOSG->reserve(cloudsize);
                // lodlevel.color->reserve(cloudsize);
                lodlevel.downsampleSkip = lodlevel.downsample / downsample;
            }
        }
        POINTTYPE minPt, maxPt;
        pcl::getMinMax3D (pc, minPt, maxPt);

        double sizex = maxPt.x - minPt.x;
        double cubesizex = (double)sizex/(double)(subClouds->xsize-1);

        double sizey = maxPt.y - minPt.y;
        double cubesizey = (double)sizey/(double)(subClouds->ysize-1);

        double sizez = maxPt.z - minPt.z;
        double cubesizez = (double)sizez/(double)(subClouds->zsize-1);

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

                for (auto &lodlevel: lodlevels)
                {
                    if (i % lodlevel.downsampleSkip == 0) {
                            cb(pc, lodlevel, i, minPt, maxPt);
                    }
                }
            }
        }

        //after sorting, add sizes
        for (auto& lodCube : *subClouds)
        {
            for (auto &lodlevel: lodCube->getLodLevels())
            {
                lodlevel.drawArrays->setCount(lodlevel.pointsOSG->size());
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
