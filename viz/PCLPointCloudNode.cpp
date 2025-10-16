#include "PCLPointCloudNode.hpp"


#include <vizkit3d/ColorConversionHelper.hpp>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <osg/Uniform>


// https://forum.playcanvas.com/t/world-coordinate-in-fragment-shader/22996/8
// https://learnopengl.com/Getting-started/Shaders
// https://gist.github.com/vicrucann/497fd5839bccba45e58b5ca48feca12f
// https://learnopengl.com/Lighting/Basic-Lighting
// https://www.khronos.org/opengl/wiki/Fragment_Shader
// https://osg-users.openscenegraph.narkive.com/8nXnCbaY/using-modern-shaders-with-osg-setting-vertex-attribute-layout
// https://github.com/openscenegraph/OpenSceneGraph/blob/master/examples/osgsimplegl3/osgsimplegl3.cpp
// https://gist.github.com/bkmeneguello/6047028

const char *vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 osg_Vertex;\n"
    "layout (location = 1) in vec3 osg_Normal;\n"
    "layout (location = 2) in vec4 osg_Color;\n"
    "out vec3 FragPos;\n"
    "out vec3 Normal;\n"
    "out vec4 Color;\n"
    "uniform mat4 modelMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "uniform mat4 modelViewProjectionMatrix;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = modelViewProjectionMatrix * vec4(osg_Vertex, 1.0);\n"
    "    FragPos = vec3(modelMatrix * vec4(osg_Vertex, 1.0));\n"
    "    //Normal = osg_NormalMatrix * osg_Normal;\n"
    "    Normal = normalize(normalMatrix * osg_Normal);\n" // no care about about actual light location (mult by transposed interveted model matrix)
    "    Color = osg_Color;\n"
    "}\n\0";

const char *fragmentShaderSource = "#version 330 core\n"
    "out vec4 out_Color;\n"
    "in vec3 FragPos;\n"
    "in vec3 Normal;\n"
    "in vec4 Color;\n"
    "uniform float cycleColorInterval;\n"
    "// from: http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl\n"
    "vec3 hsv2rgb(vec3 c) {\n"
        "c = vec3(c.x, clamp(c.yz, 0.0, 1.0));\n"
        "vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);\n"
        "vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);\n"
        "return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);\n"
    "}\n"
    "float hue2rgb(float p, float q, float t) {\n"
        "if(t < 0.0) t += 1.0;\n"
        "if(t > 1.0) t -= 1.0;\n"
        "if(t < 1.0/6.0) { return p + (q - p) * 6.0 * t; }\n"
        "if(t < 1.0/2.0) { return q; }\n"
        "if(t < 2.0/3.0) { return p + (q - p) * (2.0/3.0 - t) * 6.0; }\n"
        "return p;"
    "}\n"
    "vec3 hsl2rgb(vec3 c) {\n"
        "float h = c.x;\n"
        "float s = c.y;\n"
        "float l = c.z;\n"
        "vec3 color;\n"
        "if (s == 0){\n"
            "color.r=color.g=color.b=l;\n"
        "}else{\n"
            "float q = l < 0.5 ? l * (1.0 + s) : l + s - l * s;\n"
            "float p = 2.0 * l - q;\n"
            "color.r = hue2rgb(p, q, h + 1.0/3.0);\n"
            "color.g = hue2rgb(p, q, h);\n"
            "color.b = hue2rgb(p, q, h - 1.0/3.0);\n"
        "}\n"
        "return color;\n"
    "}\n"
    "void main()\n"
    "{\n"
    "   float z = FragPos.z;\n"
    "   float int_part;\n"
    "   float hue = (z - floor(z / cycleColorInterval) * cycleColorInterval) / cycleColorInterval;\n"
    "   vec3 hsv = vec3(hue, 1, 0.6);\n"
    "   vec3 rgbcolor = hsl2rgb(hsv);\n"
    "   vec3 lightColor = vec3(1,1,1);\n"
    "   float ambientStrength = 0.7;\n"
    "   vec3 ambient = ambientStrength * lightColor;\n"
    "   vec3 norm = normalize(Normal);\n"
    "   vec3 lightPos = vec3(0.0 , 0.0, 100.0);\n"
    "   vec3 lightDir = normalize(FragPos - lightPos);\n"
    "   float diff = abs(dot(norm, lightDir));\n"
    "   vec3 diffuse = diff*lightColor;\n"
    "   vec3 result = (ambient + diffuse) * rgbcolor;\n"
    "   out_Color = vec4(result,Color.a);\n"
    "}\n\0";


struct ModelViewProjectionMatrixCallback: public osg::Uniform::Callback
{
    ModelViewProjectionMatrixCallback(osg::Camera* camera) :
            _camera(camera) {
    }

    virtual void operator()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
        osg::Matrixd viewMatrix = _camera->getViewMatrix();
        osg::Matrixd modelMatrix = osg::computeLocalToWorld(nv->getNodePath());
        osg::Matrixd modelViewProjectionMatrix = modelMatrix * viewMatrix * _camera->getProjectionMatrix();
        uniform->set(modelViewProjectionMatrix);
    }

    osg::Camera* _camera;
};

struct NormalMatrixCallback: public osg::Uniform::Callback {
	NormalMatrixCallback(osg::Camera* camera) :
			_camera(camera) {
	}

	virtual void operator()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
		osg::Matrixd viewMatrix = _camera->getViewMatrix();
		osg::Matrixd modelMatrix = osg::computeLocalToWorld(nv->getNodePath());
		osg::Matrixd modelViewMatrix = modelMatrix * viewMatrix;

		modelViewMatrix.setTrans(0.0, 0.0, 0.0);

		osg::Matrixd inverse;
		inverse.invert(modelViewMatrix);

		osg::Matrix3 normalMatrix(
				inverse(0,0), inverse(1,0), inverse(2,0),
				inverse(0,1), inverse(1,1), inverse(2,1),
				inverse(0,2), inverse(1,2), inverse(2,2));

		uniform->set(normalMatrix);
	}

	osg::Camera* _camera;
};

struct ModelMatrixCallback: public osg::Uniform::Callback
{
    ModelMatrixCallback(){
    }

    virtual void operator()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
        osg::Matrixd modelMatrix = osg::computeLocalToWorld(nv->getNodePath());
        uniform->set(modelMatrix);
    }

};



PCLPointCloudNode::PCLPointCloudNode():subCloudsX(1),subCloudsY(1),subCloudsZ(1),pointsize(1) {
    subClouds = std::make_shared<SubClouds>(subCloudsX,subCloudsY,subCloudsZ);
    addChild(subClouds->osgGroup);

    program = new osg::Program;
    vShader = new osg::Shader(osg::Shader::VERTEX, vertexShaderSource);
    fShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
    program->addShader(vShader);
    program->addShader(fShader);
    cycleColorIntervalUniform = new osg::Uniform(osg::Uniform::FLOAT, "cycleColorInterval");
    cycleColorIntervalUniform->set((float)5);
}

void PCLPointCloudNode::addLodLevel(const float& from, const float& to, const float& downsample) {
    for (auto& subcloud : *subClouds) {
        subcloud->addLodLevel(from, to, downsample);
    }
}

void PCLPointCloudNode::setPointSize(const double & size) {
    pointsize = size;
    for (auto& lodCube : *subClouds)
    {
        for (auto &lodlevel: lodCube->getLodLevels())
        {
            lodlevel.setPointSize(pointsize);
        }
    }
}

double PCLPointCloudNode::getPointSize() {
    return pointsize;
}

void PCLPointCloudNode::setColorInterval(const float& interval) {
    cycleColorIntervalUniform->set(interval);
}

void PCLPointCloudNode::enbableHeightColorShader(osg::Node* parent, osg::Camera* cam) {
    //setup shader
    if (parent && cam) {
        subClouds->osgGroup->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);

        osg::ref_ptr<osg::Uniform> mvp = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "modelViewProjectionMatrix");
        subClouds->osgGroup->getOrCreateStateSet()->addUniform(mvp);

        mvp->setUpdateCallback(new ModelViewProjectionMatrixCallback(cam));

        osg::ref_ptr<osg::Uniform> model = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "modelMatrix");
        subClouds->osgGroup->getOrCreateStateSet()->addUniform(model);
        model->setUpdateCallback(new ModelMatrixCallback);

        osg::ref_ptr<osg::Uniform> normal = new osg::Uniform(osg::Uniform::FLOAT_MAT3, "normalMatrix");
        subClouds->osgGroup->getOrCreateStateSet()->addUniform(normal);
        normal->setUpdateCallback(new NormalMatrixCallback(cam));

        subClouds->osgGroup->getOrCreateStateSet()->addUniform(cycleColorIntervalUniform);
    }
}

void PCLPointCloudNode::dispatch(const pcl::PCLPointCloud2& pcl_cloud, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, bool useHeightColoring, double maxz, float downsample, osg::Camera* cam) {
    if(pcl::getFieldIndex(pcl_cloud, "rgba") != -1 && (show_color || show_intensity))
    {
        pcl::PointCloud<pcl::PointXYZRGBA> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, show_color, show_intensity, maxz, downsample);
    }
    else if(show_color && pcl::getFieldIndex(pcl_cloud, "rgb") != -1)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, maxz, downsample);
    }
    else if(show_intensity && pcl::getFieldIndex(pcl_cloud, "intensity") != -1)
    {
        pcl::PointCloud<pcl::PointXYZI> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, maxz, downsample);

    }
    else if (useHeightColoring) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, useHeightColoring, maxz, downsample);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, default_feature_color, maxz, downsample);   
    }
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZRGBA>& pc, const osg::Vec4f& default_feature_color, bool show_color, bool show_intensity, double maxz, float downsample) {
    traversePoints<pcl::PointXYZRGBA>(pc, downsample, maxz, [&](const pcl::PointXYZRGBA& point, LodLevel &lodlevel){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        if(!show_color)
            lodlevel.getColors()->push_back(osg::Vec4f(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), point.a/255.0));
        else if(!show_intensity)
            lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
        else
            lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, point.a/255.0));
    });
}


void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pc, double maxz, float downsample) {
    traversePoints<pcl::PointXYZRGB>(pc, downsample, maxz, [&](const pcl::PointXYZRGB& point, LodLevel &lodlevel){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZI>& pc, const osg::Vec4f& default_feature_color, double maxz, float downsample) {        
    traversePoints<pcl::PointXYZI>(pc, downsample, maxz, [&](const pcl::PointXYZI& point, LodLevel &lodlevel){
        osg::Vec4f feature_color = default_feature_color;
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        feature_color.w() = point.intensity;
        lodlevel.getColors()->push_back(feature_color);
    });
}

/**
 * this may be used directly, so we need to take care of useHeightColoring
 */
void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, bool useHeightColoring, double maxz, float downsample, osg::Camera* cam) {
    if (!useHeightColoring) {
        dispatch(pc, default_feature_color, maxz, downsample);
    }else{
        // bool color_cycle_set = false;
        std::pair<pcl::PointXYZ, pcl::PointXYZ> minmax = traversePoints<pcl::PointXYZ>(pc, downsample, maxz, [&](const pcl::PointXYZ& point, LodLevel &lodlevel){
            lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
            lodlevel.getColors()->push_back(default_feature_color);
        },false,false,true);
        enbableHeightColorShader(subClouds->osgGroup, cam);
        setColorInterval(minmax.second.z-minmax.first.z);
    }
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const osg::Vec4f& default_feature_color, double maxz, float downsample) {
    traversePoints<pcl::PointXYZ>(pc, downsample, maxz, [&](const pcl::PointXYZ& point, LodLevel &lodlevel){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        lodlevel.getColors()->push_back(default_feature_color);
    });
}
