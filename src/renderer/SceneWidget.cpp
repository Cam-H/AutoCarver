//
// Created by Cam on 2025-03-18.
//

#include "SceneWidget.h"

#include <QMouseEvent>

#include <cmath>
#include <iostream>

#include "fileIO/MeshHandler.h"

SceneWidget::SceneWidget(Scene* scene, QWidget* parent)
    : m_scene(scene)
    , m_defaultProgramIdx(0)
    , m_fov(60.0)
    , m_aspect(1.0)
    , m_zNear(1.0)
    , m_zFar(100.0)
    , m_yaw(45.0f)
    , m_pitch(45.0f)
    , m_radius(5.0f)
    , m_minRadius(1.0f)
    , m_maxRadius(100.0f)
    , m_translationSensitivity(0.002f)
    , m_rotationSensitivity(0.4f)
    , m_zoomSensitivity(0.0005f)
    , m_zoomExponential(0.2f)
    , m_center(QVector3D(0, 0, 0))
    , m_eye(m_center + cameraRotated(QVector3D(m_radius, 0, 0)))
    , QOpenGLWidget(parent)
{

}

SceneWidget::~SceneWidget()
{
    // Ensure the context is current when deleting the buffers.
    makeCurrent();
//    delete geometries; TODO clean
    doneCurrent();
}

void SceneWidget::mousePressEvent(QMouseEvent *e)
{
    // Save mouse press position
    m_mouseLastPosition = QVector2D(e->position());
}

void SceneWidget::mouseMoveEvent(QMouseEvent *e)
{
    if (e->buttons() == Qt::MouseButton::MiddleButton) {
        QVector2D delta = QVector2D((float)e->position().x(), (float)e->position().y()) - m_mouseLastPosition;

        if(QGuiApplication::keyboardModifiers().testFlag(Qt::ShiftModifier)) { // Try moving camera center
            QVector3D horz = cameraRotated(QVector3D(0, 0, 1)).normalized();
            QVector3D vert = cameraRotated(QVector3D(0, 1, 0)).normalized();

            m_center += logf(m_radius) * m_translationSensitivity * (delta.x() * horz + delta.y() * vert);

        } else { // Rotate the camera about the center instead
            m_yaw -= m_rotationSensitivity * delta.x();
            m_pitch += m_rotationSensitivity * delta.y();
            m_pitch = std::clamp(m_pitch, -89.0f, 89.0f);
        }
    }

    m_mouseLastPosition = QVector2D(e->position());

    calculateViewProjectionMatrix();

    update();
}

void SceneWidget::wheelEvent(QWheelEvent *e)
{
    float exponential = std::min(m_zoomExponential * (m_radius - m_minRadius), 4.0f);
    m_radius -= m_zoomSensitivity * e->angleDelta().y() * expf(exponential);
    m_radius = std::clamp(m_radius, m_minRadius, m_maxRadius);

    calculateViewProjectionMatrix();

    update();
}

void SceneWidget::calculateViewProjectionMatrix()
{

    // Prepare projection matrix
    m_viewProjection.setToIdentity();
    m_viewProjection.perspective(m_fov, m_aspect, m_zNear, m_zFar);

    // Apply the view matrix to the projection
    m_eye = m_center + cameraRotated(QVector3D(m_radius, 0, 0));
    m_viewProjection.lookAt(m_eye, m_center, QVector3D(0, 1, 0));

}

QVector3D SceneWidget::cameraRotated(QVector3D base) const
{
    base = QQuaternion::fromAxisAndAngle(0, 0, 1, m_pitch) * base; // Apply pitch to eye
    return QQuaternion::fromAxisAndAngle(0, 1, 0, m_yaw) * base; // Apply yaw to eye
}

void SceneWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    addShaderProgram(R"(..\res\shaders\flat)");
}

SceneWidget::RenderItem& SceneWidget::getRender(const std::shared_ptr<Mesh>& mesh, bool defaultVisibility)
{
    auto it = m_renderMap.find(mesh);
    if (it == m_renderMap.end()) { // Mesh has not been processed yet
        auto item = RenderItem{ (uint32_t)m_geometries.size(),m_defaultProgramIdx, defaultVisibility };
        m_renderMap[mesh] = item;
        m_geometries.push_back(new RenderGeometry(mesh, RenderGeometry::Format::VERTEX_NORMAL));
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

void SceneWidget::show(uint32_t idx, Scene::Model target)
{
    std::vector<std::shared_ptr<Mesh>> sources = select(idx, target);
    for (const std::shared_ptr<Mesh>& source : sources) {
        getRender(source).visible = true;
    }

    if (!sources.empty()) update();
}
void SceneWidget::hide(uint32_t idx, Scene::Model target)
{
    std::vector<std::shared_ptr<Mesh>> sources = select(idx, target);
    for (const std::shared_ptr<Mesh>& source : sources) {
        getRender(source).visible = false;
    }

    if (!sources.empty()) update();
}

std::vector<std::shared_ptr<Mesh>> SceneWidget::select(uint32_t idx, Scene::Model target)
{
    if (idx >= m_scene->bodyCount()) return {};

    std::vector<std::shared_ptr<Mesh>> selection;

    switch (target) {
        case Scene::Model::ALL:
            selection.push_back(m_scene->bodies()[idx]->mesh());
            selection.push_back(m_scene->bodies()[idx]->hullMesh());
            break;
        case Scene::Model::MESH:
            selection.push_back(m_scene->bodies()[idx]->mesh());
            break;
        case Scene::Model::HULL:
            selection.push_back(m_scene->bodies()[idx]->hullMesh());
            break;
        case Scene::Model::BOUNDING_SPHERE:
            break;
    }

    return selection;
}

void SceneWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    m_aspect = qreal(w) / qreal(h ? h : 1);

    calculateViewProjectionMatrix();
}

void SceneWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    glEnable(GL_CULL_FACE);


    if (m_scene != nullptr) {
        const std::vector<Body*>& bodies = m_scene->bodies();

        for (Body* body : bodies) {

            // TODO get transform from body
            QMatrix4x4 transform;
//    transform.translate(0.0, 0.0, -5.0);
//    transform.rotate(rotation);

            render(body->mesh(), transform, true);
            render(body->hullMesh(), transform, false);
        }
    }
}

void SceneWidget::render(const std::shared_ptr<Mesh> &mesh, const QMatrix4x4& transform, bool defaultVisibility)
{
    // Identify proper settings to render the mesh
    auto item = getRender(mesh, defaultVisibility);
    if (!item.visible) return;

    // Handle rendering

    m_programs[item.programIdx]->bind();

    // TODO manage uniforms better
    // Set modelview-projection matrix
    m_programs[item.programIdx]->setUniformValue("u_transform", transform);
    m_programs[item.programIdx]->setUniformValue("vp_matrix", m_viewProjection);
    m_programs[item.programIdx]->setUniformValue("mvp_matrix", m_viewProjection * transform);

    m_programs[item.programIdx]->setUniformValue("out_color", QVector3D(1, 1, 0));

    // Draw mesh geometry
    m_geometries[item.geometryIdx]->draw(m_programs[item.programIdx]);
}
