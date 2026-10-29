#include "PCLPointCloudNode.hpp"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>

#include <vizkit3d/ColorConversionHelper.hpp>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>

#include <osg/Uniform>
#include <osgUtil/CullVisitor>


// https://forum.playcanvas.com/t/world-coordinate-in-fragment-shader/22996/8
// https://learnopengl.com/Getting-started/Shaders
// https://gist.github.com/vicrucann/497fd5839bccba45e58b5ca48feca12f
// https://learnopengl.com/Lighting/Basic-Lighting
// https://www.khronos.org/opengl/wiki/Fragment_Shader
// https://osg-users.openscenegraph.narkive.com/8nXnCbaY/using-modern-shaders-with-osg-setting-vertex-attribute-layout
// https://github.com/openscenegraph/OpenSceneGraph/blob/master/examples/osgsimplegl3/osgsimplegl3.cpp
// https://gist.github.com/bkmeneguello/6047028

// osg_ModelViewProjectionMatrix and osg_NormalMatrix are supplied by osg itself
// (osg::State::setUseModelViewAndProjectionUniforms). Unlike a manually updated camera-based
// uniform they contain the actual rendering matrices, including the near/far planes fitted to
// the scene bounds each frame - otherwise the clouds get clipped at the cameras default
// far plane when zooming out.
const char *vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 osg_Vertex;\n"
    "layout (location = 1) in vec3 osg_Normal;\n"
    "layout (location = 2) in vec4 osg_Color;\n"
    "out vec3 FragPos;\n"
    "out vec3 Normal;\n"
    "out vec4 Color;\n"
    "uniform mat4 modelMatrix;\n"
    "uniform mat3 osg_NormalMatrix;\n"
    "uniform mat4 osg_ModelViewProjectionMatrix;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = osg_ModelViewProjectionMatrix * vec4(osg_Vertex, 1.0);\n"
    "    FragPos = vec3(modelMatrix * vec4(osg_Vertex, 1.0));\n"
    "    Normal = normalize(osg_NormalMatrix * osg_Normal);\n" // no care about about actual light location (mult by transposed interveted model matrix)
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
    "   // the point geometry carries no normal array, so Normal is the zero vector. normalize(0) is\n"
    "   // undefined: NVIDIA yields 0 (ambient only), Mesa/Intel yields NaN and the points vanish.\n"
    "   float normLength = length(Normal);\n"
    "   vec3 norm = normLength > 0.0 ? Normal / normLength : vec3(0.0);\n"
    "   vec3 lightPos = vec3(0.0 , 0.0, 100.0);\n"
    "   vec3 lightDir = normalize(FragPos - lightPos);\n"
    "   float diff = abs(dot(norm, lightDir));\n"
    "   vec3 diffuse = diff*lightColor;\n"
    "   vec3 result = (ambient + diffuse) * rgbcolor;\n"
    "   out_Color = vec4(result,Color.a);\n"
    "}\n\0";


struct ModelMatrixCallback: public osg::Uniform::Callback
{
    ModelMatrixCallback(){
    }

    virtual void operator()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
        osg::Matrixd modelMatrix = osg::computeLocalToWorld(nv->getNodePath());
        uniform->set(modelMatrix);
    }

};

/**
 * forwards the current model view matrix and viewport to the node on cull,
 * used to keep the gaussian splats sorted back to front
 */
struct SplatCullCallback: public osg::NodeCallback
{
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        if (cv && cv->getModelViewMatrix() && cv->getViewport()) {
            const osg::Viewport* vp = cv->getViewport();
            static_cast<PCLPointCloudNode*>(node)->updateView(*cv->getModelViewMatrix(), vp->width(), vp->height());
        }
        traverse(node, nv);
    }
};



PCLPointCloudNode::PCLPointCloudNode():subCloudsX(1),subCloudsY(1),subCloudsZ(1),pointsize(1),
                                       splatScale(1.0f),splatGeneration(0),sortGeneration(0),splatsSorted(false) {
    subClouds = std::make_shared<SubClouds>(subCloudsX,subCloudsY,subCloudsZ);
    addChild(subClouds->osgGroup);

    program = new osg::Program;
    vShader = new osg::Shader(osg::Shader::VERTEX, vertexShaderSource);
    fShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
    program->addShader(vShader);
    program->addShader(fShader);
    cycleColorIntervalUniform = new osg::Uniform(osg::Uniform::FLOAT, "cycleColorInterval");
    cycleColorIntervalUniform->set((float)5);

    // uniforms for the gaussian splat shaders, on the node itself so they survive clear()
    viewportUniform = new osg::Uniform("u_viewport", osg::Vec2f(1, 1));
    splatScaleUniform = new osg::Uniform("u_scale", splatScale);
    getOrCreateStateSet()->addUniform(viewportUniform);
    getOrCreateStateSet()->addUniform(splatScaleUniform);
    setCullCallback(new SplatCullCallback);
}

PCLPointCloudNode::~PCLPointCloudNode() {
    if (sortResult.valid()) {
        sortResult.wait();
    }
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
        // have osg provide osg_ModelViewProjectionMatrix and osg_NormalMatrix to the shader,
        // using the actual rendering matrices (including the per-frame computed near/far planes)
        if (cam->getGraphicsContext() && cam->getGraphicsContext()->getState()) {
            cam->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);
        }

        subClouds->osgGroup->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);

        osg::ref_ptr<osg::Uniform> model = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "modelMatrix");
        subClouds->osgGroup->getOrCreateStateSet()->addUniform(model);
        model->setUpdateCallback(new ModelMatrixCallback);

        subClouds->osgGroup->getOrCreateStateSet()->addUniform(cycleColorIntervalUniform);
    }
}

void PCLPointCloudNode::dispatch(const pcl::PCLPointCloud2& pcl_cloud, const DispatchConfig& config, bool show_color, bool show_intensity, osg::Camera* cam) {
    if(config.renderSplats && GaussianSplats::isSplatCloud(pcl_cloud))
    {
        dispatchSplats(pcl_cloud, config);
    }
    else if(pcl::getFieldIndex(pcl_cloud, "rgba") != -1 && (show_color || show_intensity))
    {
        pcl::PointCloud<pcl::PointXYZRGBA> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, config, show_color, show_intensity);
    }
    else if(show_color && pcl::getFieldIndex(pcl_cloud, "rgb") != -1)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, config);
    }
    else if(show_intensity && pcl::getFieldIndex(pcl_cloud, "intensity") != -1)
    {
        pcl::PointCloud<pcl::PointXYZI> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, config);

    }
    else if (config.useHeightColoring) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, config, cam);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromPCLPointCloud2(pcl_cloud, pc);
        dispatch(pc, config);   
    }
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZRGBA>& pc, const DispatchConfig& config, bool show_color, bool show_intensity) {
    traversePoints<pcl::PointXYZRGBA>(pc, config, [&](const pcl::PointXYZRGBA& point, const size_t& index, LodLevel &lodlevel){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        if(!show_color)
            lodlevel.getColors()->push_back(osg::Vec4f(config.default_feature_color.x(), config.default_feature_color.y(), config.default_feature_color.z(), point.a/255.0));
        else if(!show_intensity)
            lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
        else
            lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, point.a/255.0));
    });
}


void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZRGB>& pc, const DispatchConfig& config) {
    traversePoints<pcl::PointXYZRGB>(pc, config, [&](const pcl::PointXYZRGB& point, const size_t& index, LodLevel &lodlevel){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        lodlevel.getColors()->push_back(osg::Vec4f(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
    });
}

void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZI>& pc, const DispatchConfig& config) {
    traversePoints<pcl::PointXYZI>(pc, config, [&](const pcl::PointXYZI& point, const size_t& index, LodLevel &lodlevel){
        osg::Vec4f feature_color = config.default_feature_color;
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        feature_color.w() = point.intensity;
        lodlevel.getColors()->push_back(feature_color);
    });
}

/**
 * this may be used directly, so we need to take care of useHeightColoring
 */
void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const DispatchConfig& config, osg::Camera* cam) {
    std::pair<pcl::PointXYZ, pcl::PointXYZ> minmax = traversePoints<pcl::PointXYZ>(pc, config, [&](const pcl::PointXYZ& point, const size_t& index, LodLevel &lodlevel){
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        lodlevel.getColors()->push_back(config.default_feature_color);
    },false,false,true);
    if (config.useHeightColoring) {
        enbableHeightColorShader(subClouds->osgGroup, cam);
        setColorInterval(minmax.second.z-minmax.first.z);
    }
}

// void PCLPointCloudNode::dispatch(const pcl::PointCloud<pcl::PointXYZ>& pc, const DispatchConfig& config) {
//     traversePoints<pcl::PointXYZ>(pc, config, [&](const pcl::PointXYZ& point, LodLevel &lodlevel){
//         lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
//         lodlevel.getColors()->push_back(default_feature_color);
//     });
// }

void PCLPointCloudNode::dispatchSplats(const pcl::PCLPointCloud2& pcl_cloud, const DispatchConfig& config) {
    GaussianSplats::FieldOffsets fields;
    if (!GaussianSplats::getFieldOffsets(pcl_cloud, fields) || !pcl_cloud.width) {
        return;
    }

    // positions for the sub cube/LOD distribution, the splat fields are read from the raw cloud by index
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromPCLPointCloud2(pcl_cloud, pc);

    traversePoints<pcl::PointXYZ>(pc, config, [&](const pcl::PointXYZ& point, const size_t& index, LodLevel &lodlevel){
        const uint8_t* pointData = pcl_cloud.data.data()
                                   + (index / pcl_cloud.width) * pcl_cloud.row_step
                                   + (index % pcl_cloud.width) * pcl_cloud.point_step;
        const GaussianSplats::Splat splat = GaussianSplats::extract(pointData, fields);
        if (!splat.valid) {
            return;
        }
        lodlevel.getPoints()->push_back(osg::Vec3f(point.x, point.y, point.z));
        lodlevel.getSplatCovA()->push_back(splat.covA);
        lodlevel.getSplatCovB()->push_back(splat.covB);
        lodlevel.getSplatColors()->push_back(splat.color);
        // conservative splat extent: 2 sigma of the largest eigenvalue <= 2 sqrt(trace)
        const float trace = splat.covA.x() + splat.covB.x() + splat.covB.z();
        lodlevel.maxSplatRadius = std::max(lodlevel.maxSplatRadius, 2.0f * std::sqrt(std::max(0.0f, trace)));
    }, false, false, false, true);
}

void PCLPointCloudNode::applySplatBounds() {
    for (SplatChunk& chunk : splatChunks) {
        if (!chunk.bound.valid()) {
            continue;
        }
        const float radius = chunk.maxRadius * std::max(1.0f, splatScale);
        const osg::Vec3f padding(radius, radius, radius);
        chunk.geometry->setInitialBound(osg::BoundingBox(chunk.bound._min - padding, chunk.bound._max + padding));
        chunk.geometry->dirtyBound();
    }
}

void PCLPointCloudNode::setSplatScale(const float& scale) {
    splatScale = scale;
    splatScaleUniform->set(scale);
    // larger splats reach further out of their cube bounds
    applySplatBounds();
}

float PCLPointCloudNode::getSplatScale() const {
    return splatScale;
}

void PCLPointCloudNode::updateView(const osg::Matrix& modelview, float viewportWidth, float viewportHeight) {
    viewportUniform->set(osg::Vec2f(viewportWidth, viewportHeight));
    if (splatChunks.empty()) {
        return;
    }

    if (splatChunks.size() > 1) {
        // Draw the cube geometries strictly back to front by assigning render bin numbers.
        // For disjoint grid cells this order is exact: per axis, cells farther from the eye
        // cell never occlude nearer ones on the same side, and cells on opposite sides of the
        // eye never share a view ray. Only splats whose footprint crosses a cube face can
        // still blend in the wrong order.
        const osg::Vec3f eye = osg::Matrix::inverse(modelview).getTrans();
        const osg::Vec3f eyeCell((eye.x() - splatGridMin.x()) * splatGridScale.x(),
                                 (eye.y() - splatGridMin.y()) * splatGridScale.y(),
                                 (eye.z() - splatGridMin.z()) * splatGridScale.z());
        std::vector<osg::Vec3f> farness(splatChunks.size());
        std::vector<size_t> order(splatChunks.size());
        for (size_t i = 0; i < splatChunks.size(); ++i) {
            const osg::Vec3f d = splatChunks[i].cube + osg::Vec3f(0.5f, 0.5f, 0.5f) - eyeCell;
            farness[i].set(std::fabs(d.x()), std::fabs(d.y()), std::fabs(d.z()));
            order[i] = i;
        }
        std::sort(order.begin(), order.end(), [&farness](const size_t& a, const size_t& b) {
            if (farness[a].x() != farness[b].x()) return farness[a].x() > farness[b].x();
            if (farness[a].y() != farness[b].y()) return farness[a].y() > farness[b].y();
            return farness[a].z() > farness[b].z();
        });
        for (size_t rank = 0; rank < order.size(); ++rank) {
            // bins above the opaque (0) and standard transparent (10) bins, drawn in rank order
            splatChunks[order[rank]].geometry->getOrCreateStateSet()->setRenderBinDetails(
                20 + static_cast<int>(rank), "RenderBin");
        }
    }

    // osg matrices are row major (v' = v * M), the third column maps a point to its view space z
    osg::Vec3f viewdir(modelview(0, 2), modelview(1, 2), modelview(2, 2));
    viewdir.normalize();

    if (sortResult.valid() && sortResult.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        std::vector<std::vector<GLuint>> orders = sortResult.get();
        sortResult = std::future<std::vector<std::vector<GLuint>>>();
        if (sortGeneration == splatGeneration) {
            applySortResults(std::move(orders));
            sortedViewDir = requestedViewDir;
        }
    }

    if (!splatsSorted) {
        // first draw of this cloud: sort synchronously to avoid showing unsorted splats
        sortedViewDir = viewdir;
        std::vector<std::vector<GLuint>> orders;
        orders.reserve(splatChunks.size());
        for (SplatChunk& chunk : splatChunks) {
            orders.push_back(GaussianSplats::sortBackToFront(*chunk.centers, viewdir));
        }
        applySortResults(std::move(orders));
    } else if (!sortResult.valid() && viewdir * sortedViewDir < 0.99999f) {
        requestedViewDir = viewdir;
        sortGeneration = splatGeneration;
        std::vector<osg::ref_ptr<const osg::Vec3Array>> centers;
        centers.reserve(splatChunks.size());
        for (SplatChunk& chunk : splatChunks) {
            centers.push_back(chunk.centers);
        }
        sortResult = std::async(std::launch::async, [centers, viewdir]() {
            std::vector<std::vector<GLuint>> orders;
            orders.reserve(centers.size());
            for (const osg::ref_ptr<const osg::Vec3Array>& chunkCenters : centers) {
                orders.push_back(GaussianSplats::sortBackToFront(*chunkCenters, viewdir));
            }
            return orders;
        });
    }
}

void PCLPointCloudNode::applySortResults(std::vector<std::vector<GLuint>>&& orders) {
    splatsSorted = true;
    if (orders.size() != splatChunks.size()) {
        return;
    }
    for (size_t i = 0; i < orders.size(); ++i) {
        if (orders[i].size() != splatChunks[i].indices->size()) {
            continue;
        }
        splatChunks[i].indices->asVector().swap(orders[i]);
        splatChunks[i].indices->dirty();
    }
}
