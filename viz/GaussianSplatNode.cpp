#include "GaussianSplatNode.hpp"

#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstring>

#include <Eigen/Geometry>

#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Program>
#include <osg/Shader>
#include <osgUtil/CullVisitor>

// Splat rendering after "3D Gaussian Splatting for Real-Time Radiance Field Rendering" (Kerbl et al. 2023):
// the 3D covariance of each splat is projected to a 2D screen space covariance (EWA splatting),
// the geometry shader emits a quad spanning the ellipse, the fragment shader evaluates the gaussian.
// Splats are alpha blended back to front.

namespace {

// zero order spherical harmonics coefficient
const float SH_C0 = 0.28209479177387814f;

const GLuint COV_A_ATTRIB = 1;
const GLuint COV_B_ATTRIB = 2;
const GLuint COLOR_ATTRIB = 3;

const char *splatVertexShader = "#version 330 compatibility\n"
    "in vec3 a_covA;\n"  // xx xy xz
    "in vec3 a_covB;\n"  // yy yz zz
    "in vec4 a_color;\n"
    "uniform vec2 u_viewport;\n"
    "uniform float u_scale;\n"
    "out VertexData {\n"
    "    vec4 color;\n"
    "    vec2 axis1;\n"
    "    vec2 axis2;\n"
    "} v_out;\n"
    "void main() {\n"
    "    vec4 cam = gl_ModelViewMatrix * gl_Vertex;\n"
    "    gl_Position = gl_ProjectionMatrix * cam;\n"
    "    v_out.color = a_color;\n"
    "    mat3 cov3d = mat3(a_covA.x, a_covA.y, a_covA.z,\n"
    "                      a_covA.y, a_covB.x, a_covB.y,\n"
    "                      a_covA.z, a_covB.y, a_covB.z) * (u_scale * u_scale);\n"
    "    float fx = gl_ProjectionMatrix[0][0] * 0.5 * u_viewport.x;\n"
    "    float fy = gl_ProjectionMatrix[1][1] * 0.5 * u_viewport.y;\n"
    "    mat3 J;\n"
    "    if (gl_ProjectionMatrix[2][3] != 0.0) {\n"
    // perspective: jacobian of the projection at the splat center
    "        float d = -cam.z;\n"
    "        if (d <= 0.0) { v_out.color.a = 0.0; return; }\n"
    // limit the jacobian for splats far outside the view (as in the reference implementation)
    "        float limx = 1.3 / gl_ProjectionMatrix[0][0];\n"
    "        float limy = 1.3 / gl_ProjectionMatrix[1][1];\n"
    "        float x = clamp(cam.x / d, -limx, limx) * d;\n"
    "        float y = clamp(cam.y / d, -limy, limy) * d;\n"
    "        J = mat3(fx / d, 0.0, 0.0,\n"
    "                 0.0, fy / d, 0.0,\n"
    "                 fx * x / (d * d), fy * y / (d * d), 0.0);\n"
    "    } else {\n"
    "        J = mat3(fx, 0.0, 0.0,  0.0, fy, 0.0,  0.0, 0.0, 0.0);\n"
    "    }\n"
    "    mat3 T = J * mat3(gl_ModelViewMatrix);\n"
    "    mat3 cov2d = T * cov3d * transpose(T);\n"
    // low pass filter: every splat covers at least about one pixel
    "    float a = cov2d[0][0] + 0.3;\n"
    "    float b = cov2d[0][1];\n"
    "    float c = cov2d[1][1] + 0.3;\n"
    "    float mid = 0.5 * (a + c);\n"
    "    float rad = length(vec2(0.5 * (a - c), b));\n"
    "    float lambda1 = mid + rad;\n"
    "    float lambda2 = max(mid - rad, 0.1);\n"
    "    vec2 v1 = abs(b) < 1e-7 ? (a >= c ? vec2(1.0, 0.0) : vec2(0.0, 1.0)) : normalize(vec2(b, lambda1 - a));\n"
    "    vec2 v2 = vec2(-v1.y, v1.x);\n"
    // axes in pixels, scaled so exp(-dot(p,p)) is the gaussian for p in quad coordinates
    "    v_out.axis1 = min(sqrt(2.0 * lambda1), 1024.0) * v1;\n"
    "    v_out.axis2 = min(sqrt(2.0 * lambda2), 1024.0) * v2;\n"
    "}\n";

const char *splatGeometryShader = "#version 330 compatibility\n"
    "layout(points) in;\n"
    "layout(triangle_strip, max_vertices = 4) out;\n"
    "in VertexData {\n"
    "    vec4 color;\n"
    "    vec2 axis1;\n"
    "    vec2 axis2;\n"
    "} v_in[];\n"
    "uniform vec2 u_viewport;\n"
    "out vec4 g_color;\n"
    "out vec2 g_pos;\n"
    "void main() {\n"
    "    vec4 center = gl_in[0].gl_Position;\n"
    "    if (v_in[0].color.a < 1.0 / 255.0 || center.w <= 0.0 || center.z < -center.w) return;\n"
    "    vec2 ndc = center.xy / center.w;\n"
    "    vec2 toNdc = 2.0 / u_viewport;\n"
    "    vec2 axis1 = v_in[0].axis1 * toNdc;\n"
    "    vec2 axis2 = v_in[0].axis2 * toNdc;\n"
    "    vec2 extent = 2.0 * (abs(axis1) + abs(axis2));\n"
    "    if (any(greaterThan(abs(ndc) - extent, vec2(1.0)))) return;\n"
    "    const vec2 corners[4] = vec2[4](vec2(-2.0, -2.0), vec2(2.0, -2.0), vec2(-2.0, 2.0), vec2(2.0, 2.0));\n"
    "    for (int i = 0; i < 4; ++i) {\n"
    "        vec2 pos = ndc + corners[i].x * axis1 + corners[i].y * axis2;\n"
    "        gl_Position = vec4(pos * center.w, center.z, center.w);\n"
    "        g_color = v_in[0].color;\n"
    "        g_pos = corners[i];\n"
    "        EmitVertex();\n"
    "    }\n"
    "    EndPrimitive();\n"
    "}\n";

const char *splatFragmentShader = "#version 330 compatibility\n"
    "in vec4 g_color;\n"
    "in vec2 g_pos;\n"
    "void main() {\n"
    "    float power = -dot(g_pos, g_pos);\n"
    "    if (power < -4.0) discard;\n"
    "    float alpha = exp(power) * g_color.a;\n"
    "    if (alpha < 1.0 / 255.0) discard;\n"
    "    gl_FragColor = vec4(g_color.rgb, alpha);\n"
    "}\n";

class SplatCullCallback : public osg::NodeCallback {
 public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        if (cv && cv->getModelViewMatrix() && cv->getViewport()) {
            const osg::Viewport* vp = cv->getViewport();
            static_cast<GaussianSplatNode*>(node)->updateView(*cv->getModelViewMatrix(), vp->width(), vp->height());
        }
        traverse(node, nv);
    }
};

int fieldOffset(const pcl::PCLPointCloud2& cloud, const std::string& name) {
    for (const pcl::PCLPointField& field : cloud.fields) {
        if (field.name == name && field.datatype == pcl::PCLPointField::FLOAT32) {
            return field.offset;
        }
    }
    return -1;
}

const char* const splatFields[] = {"x", "y", "z",
                                   "scale_0", "scale_1", "scale_2",
                                   "rot_0", "rot_1", "rot_2", "rot_3",
                                   "opacity",
                                   "f_dc_0", "f_dc_1", "f_dc_2"};
const size_t numSplatFields = sizeof(splatFields) / sizeof(splatFields[0]);

}  // namespace

GaussianSplatNode::GaussianSplatNode()
    : geode(new osg::Geode),
      viewportUniform(new osg::Uniform("u_viewport", osg::Vec2f(1, 1))),
      scaleUniform(new osg::Uniform("u_scale", 1.0f)),
      sorted(false),
      splatScale(1.0f) {

    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, splatVertexShader));
    program->addShader(new osg::Shader(osg::Shader::GEOMETRY, splatGeometryShader));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, splatFragmentShader));
    program->addBindAttribLocation("a_covA", COV_A_ATTRIB);
    program->addBindAttribLocation("a_covB", COV_B_ATTRIB);
    program->addBindAttribLocation("a_color", COLOR_ATTRIB);

    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setAttributeAndModes(program, osg::StateAttribute::ON);
    stateset->addUniform(viewportUniform);
    stateset->addUniform(scaleUniform);
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
    // test against the scene, but don't write depth (splats are blended in sorted order)
    stateset->setAttributeAndModes(new osg::Depth(osg::Depth::LESS, 0.0, 1.0, false), osg::StateAttribute::ON);
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    addChild(geode);
    setCullCallback(new SplatCullCallback);
}

GaussianSplatNode::~GaussianSplatNode() {
    if (sortResult.valid()) {
        sortResult.wait();
    }
}

bool GaussianSplatNode::isSplatCloud(const pcl::PCLPointCloud2& cloud) {
    for (size_t i = 0; i < numSplatFields; ++i) {
        if (fieldOffset(cloud, splatFields[i]) < 0) {
            return false;
        }
    }
    return true;
}

void GaussianSplatNode::clear() {
    if (sortResult.valid()) {
        sortResult.wait();
        sortResult = std::future<std::vector<GLuint>>();
    }
    geode->removeDrawables(0, geode->getNumDrawables());
    geometry = nullptr;
    indices = nullptr;
    centers = nullptr;
    sorted = false;
}

void GaussianSplatNode::setData(const pcl::PCLPointCloud2& cloud) {
    clear();
    if (!isSplatCloud(cloud)) {
        return;
    }

    int offsets[numSplatFields];
    for (size_t i = 0; i < numSplatFields; ++i) {
        offsets[i] = fieldOffset(cloud, splatFields[i]);
    }

    const size_t count = static_cast<size_t>(cloud.width) * cloud.height;
    osg::ref_ptr<osg::Vec3Array> positions = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> covA = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec3Array> covB = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec4ubArray> colors = new osg::Vec4ubArray(osg::Array::BIND_PER_VERTEX);
    colors->setNormalize(true);
    positions->reserve(count);
    covA->reserve(count);
    covB->reserve(count);
    colors->reserve(count);

    float values[numSplatFields];
    for (size_t row = 0; row < cloud.height; ++row) {
        for (size_t col = 0; col < cloud.width; ++col) {
            const uint8_t* point = cloud.data.data() + row * cloud.row_step + col * cloud.point_step;
            for (size_t i = 0; i < numSplatFields; ++i) {
                memcpy(&values[i], point + offsets[i], sizeof(float));
            }
            const float opacity = 1.0f / (1.0f + std::exp(-values[10]));
            if (opacity < 1.0f / 255.0f || !std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2])) {
                continue;
            }

            // covariance = R S S^T R^T
            Eigen::Quaternionf rotation(values[6], values[7], values[8], values[9]);
            if (rotation.squaredNorm() < 1e-12f) {
                rotation.setIdentity();
            }
            const Eigen::Vector3f scale(std::exp(values[3]), std::exp(values[4]), std::exp(values[5]));
            const Eigen::Matrix3f m = rotation.normalized().toRotationMatrix() * scale.asDiagonal();
            const Eigen::Matrix3f cov = m * m.transpose();

            auto toByte = [](float shCoeff) {
                const float c = 0.5f + SH_C0 * shCoeff;
                return static_cast<uint8_t>(std::round(std::min(1.0f, std::max(0.0f, c)) * 255.0f));
            };

            positions->push_back(osg::Vec3f(values[0], values[1], values[2]));
            covA->push_back(osg::Vec3f(cov(0, 0), cov(0, 1), cov(0, 2)));
            covB->push_back(osg::Vec3f(cov(1, 1), cov(1, 2), cov(2, 2)));
            colors->push_back(osg::Vec4ub(toByte(values[11]), toByte(values[12]), toByte(values[13]),
                                          static_cast<uint8_t>(std::round(opacity * 255.0f))));
        }
    }

    indices = new osg::DrawElementsUInt(osg::PrimitiveSet::POINTS, positions->size());
    for (GLuint i = 0; i < positions->size(); ++i) {
        (*indices)[i] = i;
    }
    indices->setDataVariance(osg::Object::DYNAMIC);

    geometry = new osg::Geometry;
    geometry->setDataVariance(osg::Object::DYNAMIC);
    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexArray(positions);
    geometry->setVertexAttribArray(COV_A_ATTRIB, covA);
    geometry->setVertexAttribArray(COV_B_ATTRIB, covB);
    geometry->setVertexAttribArray(COLOR_ATTRIB, colors);
    geometry->addPrimitiveSet(indices);
    geode->addDrawable(geometry);

    centers = positions;
}

void GaussianSplatNode::setSplatScale(float scale) {
    splatScale = scale;
    scaleUniform->set(scale);
}

void GaussianSplatNode::updateView(const osg::Matrix& modelview, float viewportWidth, float viewportHeight) {
    viewportUniform->set(osg::Vec2f(viewportWidth, viewportHeight));
    if (!centers.valid() || centers->empty()) {
        return;
    }

    // osg matrices are row major (v' = v * M), the third column maps a point to its view space z
    osg::Vec3f viewdir(modelview(0, 2), modelview(1, 2), modelview(2, 2));
    viewdir.normalize();

    if (sortResult.valid() && sortResult.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        applySortResult(sortResult.get());
        sortedViewDir = requestedViewDir;
    }

    if (!sorted) {
        // first draw: sort synchronously to avoid showing an unsorted cloud
        sortedViewDir = viewdir;
        applySortResult(sortBackToFront(*centers, viewdir));
    } else if (!sortResult.valid() && viewdir * sortedViewDir < 0.99999f) {
        requestedViewDir = viewdir;
        osg::ref_ptr<const osg::Vec3Array> sortCenters = centers;
        sortResult = std::async(std::launch::async, [sortCenters, viewdir]() {
            return sortBackToFront(*sortCenters, viewdir);
        });
    }
}

std::vector<GLuint> GaussianSplatNode::sortBackToFront(const osg::Vec3Array& centers, const osg::Vec3f& viewdir) {
    // 16 bit counting sort on view space z (ascending, so farthest first)
    const size_t count = centers.size();
    std::vector<float> depth(count);
    float minDepth = FLT_MAX;
    float maxDepth = -FLT_MAX;
    for (size_t i = 0; i < count; ++i) {
        depth[i] = centers[i] * viewdir;
        minDepth = std::min(minDepth, depth[i]);
        maxDepth = std::max(maxDepth, depth[i]);
    }

    const float range = maxDepth - minDepth;
    const float toKey = range > 0 ? 65535.0f / range : 0.0f;
    std::vector<uint16_t> keys(count);
    std::vector<uint32_t> start(65536 + 1, 0);
    for (size_t i = 0; i < count; ++i) {
        keys[i] = static_cast<uint16_t>((depth[i] - minDepth) * toKey);
        ++start[keys[i] + 1];
    }
    for (size_t k = 1; k < start.size(); ++k) {
        start[k] += start[k - 1];
    }

    std::vector<GLuint> order(count);
    for (size_t i = 0; i < count; ++i) {
        order[start[keys[i]]++] = i;
    }
    return order;
}

void GaussianSplatNode::applySortResult(std::vector<GLuint>&& order) {
    sorted = true;
    if (!indices.valid() || order.size() != indices->size()) {
        return;
    }
    indices->asVector().swap(order);
    indices->dirty();
}
