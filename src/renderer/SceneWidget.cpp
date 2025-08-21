//
// Created by Cam on 2025-03-18.
//

#include "SceneWidget.h"

#include <QMouseEvent>

#include <cmath>
#include <iostream>

#include "renderer/RenderBuffer.h"
#include "core/Scene.h"

#include "renderer/RenderGeometry.h"


SceneWidget::SceneWidget(QWidget* parent)
    : m_buffer(std::make_shared<RenderBuffer>())
    , m_timer(nullptr)
    , m_interval(1000 / 60)
    , m_defaultProgramIdx(0)
    , m_camera()
    , m_minRadius(1.0f)
    , m_maxRadius(100.0f)
    , m_translationSensitivity(0.002f)
    , m_rotationSensitivity(0.4f)
    , m_zoomSensitivity(0.0005f)
    , m_zoomExponential(0.2f)
    , m_sphere(MeshBuilder::icosphere(1.0, 3))
    , m_axes(MeshBuilder::axes(Axis3D()))
    , QOpenGLWidget(parent)
{
}

SceneWidget::SceneWidget(const std::shared_ptr<Scene>& scene, QWidget* parent)
    : SceneWidget(parent)
{
    scene->addRenderBuffer(m_buffer);
}

SceneWidget::~SceneWidget()
{
    // Ensure the context is current when deleting the buffers.
    makeCurrent();
//    delete geometries; TODO clean
    doneCurrent();
}

void SceneWidget::setScene(const std::shared_ptr<Scene>& scene)
{
    clear(); // TODO remove buffer from older scenes

    scene->addRenderBuffer(m_buffer);

    update();
}

void SceneWidget::mousePressEvent(QMouseEvent *e)
{
    if (e->buttons() == Qt::MouseButton::LeftButton) {
        auto ndc = QPointF{
            2.0f * e->position().x() / QWidget::width() - 1.0f,
            1.0f - 2.0f * e->position().y() / QWidget::height()
        };

        emit mousepick(m_camera.getRay(ndc));
    }

    // Save mouse press position
    m_mouseLastPosition = QVector2D(e->position());
    QOpenGLWidget::mousePressEvent(e);
}

void SceneWidget::mouseMoveEvent(QMouseEvent *e)
{
    if (e->buttons() == Qt::MouseButton::MiddleButton) {
        QVector2D delta = QVector2D(e->position().x(), e->position().y()) - m_mouseLastPosition;

        double yaw = m_camera.getYaw(), pitch = m_camera.getPitch();

        if(QGuiApplication::keyboardModifiers().testFlag(Qt::ShiftModifier)) { // Try moving camera center
            QVector3D horz = -m_camera.horizontal(), vert = m_camera.vertical();

            m_camera.offset(log(m_camera.getRadius()) * m_translationSensitivity * (delta.x() * horz + delta.y() * vert));

        } else { // Rotate the camera about the center instead
            yaw -= m_rotationSensitivity * delta.x();
            pitch = std::clamp(pitch + m_rotationSensitivity * delta.y(), -89.0, 89.0);
            m_camera.setViewingAngle(yaw, pitch);
        }

        emit perspectiveChanged();
    }

    m_mouseLastPosition = QVector2D(e->position());

    update();
}

void SceneWidget::wheelEvent(QWheelEvent *e)
{
    double radius = m_camera.getRadius();
    double exponential = std::min(m_zoomExponential * (radius - m_minRadius), 4.0);
    radius -= m_zoomSensitivity * e->angleDelta().y() * exp(exponential);
    radius = std::clamp(radius, m_minRadius, m_maxRadius);

    m_camera.setRadius(radius);

    emit perspectiveChanged();

    update();
}

void SceneWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0.3, 0.3, 0.3, 1);

    addShaderProgram(R"(..\res\shaders\flat)");
    addShaderProgram(R"(..\res\shaders\color)");

}

// WARNING: Will crash if a nullptr is passed as the mesh
SceneWidget::RenderItem& SceneWidget::getGeometryBuffer(const std::shared_ptr<Mesh>& mesh)
{
    auto it = m_renderMap.find(mesh);
    if (it == m_renderMap.end()) { // Mesh has not been processed yet
        if (!mesh->isInitialized()) throw std::runtime_error("[SceneWidget] Mesh not initialized. Can not prepare render");

        auto item = RenderItem{ (uint32_t)m_geometries.size(),m_defaultProgramIdx + 1 };
        m_renderMap[mesh] = item;
        if (!mesh->faceColorsAssigned()) mesh->setFaceColor(mesh->baseColor());
        m_geometries.push_back(new RenderGeometry(mesh, RenderGeometry::Format::VERTEX_NORMAL_COLOR ));

        return m_renderMap[mesh];
    }

    return it->second;
}

void SceneWidget::addShaderProgram(const std::string& name)
{
    auto *program = new QOpenGLShaderProgram;

    std::cout << name << "|||" << (name + ".frag") << "\n";
    // Compile vertex shader
    if (!program->addShaderFromSourceFile(QOpenGLShader::Vertex, (name + ".vert").c_str()))
        close();

    // Compile fragment shader
    if (!program->addShaderFromSourceFile(QOpenGLShader::Fragment, (name + ".frag").c_str()))
        close();

    // Link shader pipeline
    if (!program->link())
        close();

    // Bind shader pipeline for use
    if (!program->bind())
        close();

    m_programs.push_back(program);
}

void SceneWidget::createDefaultShaderProgram(const std::string& name)
{
    m_defaultProgramIdx = m_programs.size();
    addShaderProgram(name);
}

void SceneWidget::setDefaultShaderProgram(uint32_t idx)
{
    if (idx >= m_programs.size()) return;
    m_defaultProgramIdx = idx;
}

void SceneWidget::show(uint32_t ID, Scene::Model target)
{
    setVisibility(true, ID, target);
}
void SceneWidget::showAll(Scene::Model target)
{
    setVisibility(true, target);
}

void SceneWidget::hide(uint32_t ID, Scene::Model target)
{
    setVisibility(false, ID, target);
}

void SceneWidget::hideAll(Scene::Model target)
{
    setVisibility(false, target);
}

void SceneWidget::setVisibility(bool visible, uint32_t ID, Scene::Model target)
{
    RenderSettings& settings = getSettings(ID);

    switch (target) {
        case Scene::Model::ALL:
            settings.meshVisibility = settings.hullVisibility = settings.boundsVisibility = settings.axesVisibility = visible;
            break;
        case Scene::Model::MESH:
            settings.meshVisibility = visible;
            break;
        case Scene::Model::HULL:
            settings.hullVisibility = visible;
            break;
        case Scene::Model::BOUNDING_SPHERE:
            settings.boundsVisibility = visible;
            break;
        case Scene::Model::AABB: // TODO
            break;
        case Scene::Model::AXES:
            settings.axesVisibility = visible;
            break;
    }
}

void SceneWidget::setVisibility(bool visible, Scene::Model target)
{
    switch (target) {
        case Scene::Model::ALL:
            for (RenderSettings& settings : m_settings)
                settings.meshVisibility = settings.hullVisibility = settings.boundsVisibility = settings.axesVisibility = visible;
            break;
        case Scene::Model::MESH:
            for (RenderSettings& settings : m_settings) settings.meshVisibility = visible;
            break;
        case Scene::Model::HULL:
            for (RenderSettings& settings : m_settings) settings.hullVisibility = visible;
            break;
        case Scene::Model::BOUNDING_SPHERE:
            for (RenderSettings& settings : m_settings) settings.boundsVisibility = visible;
            break;
        case Scene::Model::AABB: // TODO
            break;
        case Scene::Model::AXES:
            for (RenderSettings& settings : m_settings) settings.axesVisibility = visible;
            break;
    }

    update();
}

void SceneWidget::start()
{
    if (m_timer == nullptr) {
        m_timer = new QTimer(this);
        connect(m_timer, &QTimer::timeout, this, &SceneWidget::paint);
    }

    m_timer->start(m_interval);
}
void SceneWidget::pause()
{
    m_timer->stop();
}

void SceneWidget::setUpdateInterval(std::chrono::milliseconds msec)
{
    m_interval = msec;
    if (m_timer != nullptr && m_timer->isActive()) start();
}
void SceneWidget::setTargetFPS(uint32_t target)
{
    setUpdateInterval(std::chrono::milliseconds(1000 / target));
}

void SceneWidget::paint()
{
    update();
}

void SceneWidget::clear()
{
    for (RenderGeometry* rg : m_geometries) {
        delete rg;
    }

    m_geometries.clear();
    m_renderMap.clear();
}

void SceneWidget::updateRenderGeometry(const std::shared_ptr<Mesh>& mesh)
{
    auto item = getGeometryBuffer(mesh);
    // TODO properly delete
    m_geometries[item.geometryIdx] = new RenderGeometry(mesh, mesh->faceColorsAssigned() ? RenderGeometry::Format::VERTEX_NORMAL_COLOR : RenderGeometry::Format::VERTEX_NORMAL);

}

void SceneWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    m_camera.setAspectRatio(qreal(w) / qreal(h ? h : 1));

    emit perspectiveChanged();
}

void SceneWidget::paintGL()
{

    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    glEnable(GL_CULL_FACE);

    const std::vector<RenderBuffer::Item>& items = m_buffer->latest();
    for (const RenderBuffer::Item& item : items) {

        QMatrix4x4 transform;
        for (int j = 0; j < 4; j++) {
            transform.setColumn(j, QVector4D(item.transform[j][0], item.transform[j][1],
                                                        item.transform[j][2], item.transform[j][3]));
        }

        auto settings = getSettings(item.ID);
        if (settings.meshVisibility) render(item.mesh, transform);
        if (settings.hullVisibility) render(item.hull, transform);
        if (settings.axesVisibility) render(m_axes, transform);

        transform.translate(item.bounds.center.x, item.bounds.center.y, item.bounds.center.z);
        transform.scale(item.bounds.radius);
        if (settings.boundsVisibility) render(m_sphere, transform);
    }
}

// Based on the fact that IDs are calculated sequentially by Scene
SceneWidget::RenderSettings& SceneWidget::getSettings(uint32_t ID)
{
    while (ID >= m_settings.size()) m_settings.push_back({ true, false, false, false });
    return m_settings[ID];
}

void SceneWidget::render(const std::shared_ptr<Mesh> &mesh, const QMatrix4x4& transform)
{

    // Identify proper settings to render the mesh
    if (mesh == nullptr) return;
    auto item = getGeometryBuffer(mesh);

    // Handle rendering
    auto *program = m_programs[item.programIdx];
    if (mesh->useOverrideColor()) program = m_programs[0]; // Use a flat shader instead in case no colors are assigned
    program->bind();

    // TODO manage uniforms better
    // Set modelview-projection matrix
    program->setUniformValue("u_transform", transform);
    program->setUniformValue("vp_matrix", m_camera.getViewProjection());
    program->setUniformValue("mvp_matrix", m_camera.getViewProjection() * transform);
    program->setUniformValue("n_matrix", transform.normalMatrix());

    const glm::dvec3& color = mesh->overrideColor();
    program->setUniformValue("out_color", QVector3D(color.r, color.g, color.b));

    // Draw mesh geometry
    m_geometries[item.geometryIdx]->draw(program);
}

Camera& SceneWidget::camera()
{
    return m_camera;
}
