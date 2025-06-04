//
// Created by Cam on 2025-03-18.
//

#include "SceneWidget.h"

#include <QMouseEvent>

#include <cmath>
#include <iostream>


SceneWidget::SceneWidget(QWidget* parent)
    : SceneWidget(nullptr, parent)
{
}

SceneWidget::SceneWidget(const std::shared_ptr<Scene>& scene, QWidget* parent)
    : m_scene(scene)
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

void SceneWidget::setScene(const std::shared_ptr<Scene>& scene)
{
    if (m_scene != scene) clear();
    m_scene = scene;

    update();
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

        float yaw = m_camera.getYaw(), pitch = m_camera.getPitch();

        if(QGuiApplication::keyboardModifiers().testFlag(Qt::ShiftModifier)) { // Try moving camera center
            QVector3D horz = m_camera.horizontal(), vert = m_camera.vertical();

            m_camera.offset(logf(m_camera.getRadius()) * m_translationSensitivity * (delta.x() * horz + delta.y() * vert));

        } else { // Rotate the camera about the center instead
            yaw -= m_rotationSensitivity * delta.x();
            pitch = std::clamp(pitch + m_rotationSensitivity * delta.y(), -89.0f, 89.0f);
            m_camera.setViewingAngle(yaw, pitch);
        }
    }

    m_mouseLastPosition = QVector2D(e->position());

    update();
}

void SceneWidget::wheelEvent(QWheelEvent *e)
{
    float radius = m_camera.getRadius();
    float exponential = std::min(m_zoomExponential * (radius - m_minRadius), 4.0f);
    radius -= m_zoomSensitivity * e->angleDelta().y() * expf(exponential);
    radius = std::clamp(radius, m_minRadius, m_maxRadius);

    m_camera.setRadius(radius);

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
SceneWidget::RenderItem& SceneWidget::getRender(const std::shared_ptr<Mesh>& mesh, bool defaultVisibility)
{
    auto it = m_renderMap.find(mesh);
    if (it == m_renderMap.end()) { // Mesh has not been processed yet
        auto item = RenderItem{ (uint32_t)m_geometries.size(),m_defaultProgramIdx + mesh->faceColorsAssigned(), defaultVisibility };
        m_renderMap[mesh] = item;
        m_geometries.push_back(new RenderGeometry(mesh, mesh->faceColorsAssigned() ? RenderGeometry::Format::VERTEX_NORMAL_COLOR : RenderGeometry::Format::VERTEX_NORMAL));
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
    show(select(idx, target));
}
void SceneWidget::showAll(Scene::Model target)
{
    show(selectAll(target));
}
void SceneWidget::show(const std::vector<std::shared_ptr<Mesh>>& selection)
{
    for (const std::shared_ptr<Mesh>& mesh : selection) {
        getRender(mesh).visible = true;
    }

    if (!selection.empty()) update();
}

void SceneWidget::hide(uint32_t idx, Scene::Model target)
{
    hide(select(idx, target));
}
void SceneWidget::hideAll(Scene::Model target)
{
    hide(selectAll(target));
}
void SceneWidget::hide(const std::vector<std::shared_ptr<Mesh>>& selection)
{
    for (const std::shared_ptr<Mesh>& mesh : selection) {
        getRender(mesh).visible = false;
    }

    if (!selection.empty()) update();
}

std::vector<std::shared_ptr<Mesh>> SceneWidget::select(uint32_t idx, Scene::Model target)
{
    if (m_scene == nullptr || idx >= m_scene->bodyCount()) return {};

    std::vector<std::shared_ptr<Mesh>> selection;

    switch (target) {
        case Scene::Model::ALL:
            selection.push_back(m_scene->bodies()[idx]->mesh());
            if (m_scene->bodies()[idx]->hullMesh() != nullptr) selection.push_back(m_scene->bodies()[idx]->hullMesh());
            break;
        case Scene::Model::MESH:
            selection.push_back(m_scene->bodies()[idx]->mesh());
            break;
        case Scene::Model::HULL:
            if (m_scene->bodies()[idx]->hullMesh() != nullptr) selection.push_back(m_scene->bodies()[idx]->hullMesh());
            break;
        case Scene::Model::BOUNDING_SPHERE:
            if (m_scene->bodies()[idx]->bSphereMesh() != nullptr) selection.push_back(m_scene->bodies()[idx]->bSphereMesh());
            break;
    }

    return selection;
}

std::vector<std::shared_ptr<Mesh>> SceneWidget::selectAll(Scene::Model target)
{
    if (m_scene == nullptr) return {};

    std::vector<std::shared_ptr<Mesh>> selection;

    switch (target) {
        case Scene::Model::ALL:
            for (const std::shared_ptr<RigidBody>& body : m_scene->bodies()) selection.push_back(body->mesh());
            for (const std::shared_ptr<RigidBody>& body : m_scene->bodies()) {
                if (body->hullMesh() != nullptr) selection.push_back(body->hullMesh());
            }
            break;
        case Scene::Model::MESH:
            for (const std::shared_ptr<RigidBody>& body : m_scene->bodies()) selection.push_back(body->mesh());
            break;
        case Scene::Model::HULL:
            for (const std::shared_ptr<RigidBody>& body : m_scene->bodies()) {
                if (body->hullMesh() != nullptr) selection.push_back(body->hullMesh());
            }
            break;
        case Scene::Model::BOUNDING_SPHERE:
            for (const std::shared_ptr<RigidBody>& body : m_scene->bodies()) {
                if (body->hullMesh() != nullptr) selection.push_back(body->bSphereMesh());
            }
            break;
    }

    return selection;
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
    m_geometries.clear();
    m_renderMap.clear();
}

void SceneWidget::updateRenderGeometry(const std::shared_ptr<Mesh>& mesh)
{
    auto item = getRender(mesh);
    // TODO properly delete
    m_geometries[item.geometryIdx] = new RenderGeometry(mesh, mesh->faceColorsAssigned() ? RenderGeometry::Format::VERTEX_NORMAL_COLOR : RenderGeometry::Format::VERTEX_NORMAL);

}

void SceneWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    m_camera.setAspectRatio(qreal(w) / qreal(h ? h : 1));
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
        const std::vector<std::shared_ptr<RigidBody>>& bodies = m_scene->bodies();

        for (const std::shared_ptr<RigidBody>& body : bodies) {
            glm::mat4x4 trans = body->getTransform();

            QMatrix4x4 transform;
            for (int i = 0; i < 4; i++) transform.setColumn(i, QVector4D(trans[i][0], trans[i][1], trans[i][2], trans[i][3]));

            // For robot - Wrong TODO
//            for (int i = 0; i < 4; i++) transform.setRow(i, QVector4D(trans[i][0], trans[i][1], trans[i][2], trans[i][3]));

//            std::cout << "QTransform:\n"
//                    << transform.row(0)[0] << " " << transform.row(0)[1] << " " << transform.row(0)[2] << " " << transform.row(0)[3] << "\n"
//                    << transform.row(1)[0] << " " << transform.row(1)[1] << " " << transform.row(1)[2] << " " << transform.row(1)[3] << "\n"
//                    << transform.row(2)[0] << " " << transform.row(2)[1] << " " << transform.row(2)[2] << " " << transform.row(2)[3] << "\n"
//                    << transform.row(3)[0] << " " << transform.row(3)[1] << " " << transform.row(3)[2] << " " << transform.row(3)[3] << "\n";

//            std::cout << "GLM Transform:\n"
//                    << trans[0][0] << " " << trans[0][1] << " " << trans[0][2] << " " << trans[0][3] << "\n"
//                    << trans[1][0] << " " << trans[1][1] << " " << trans[1][2] << " " << trans[1][3] << "\n"
//                    << trans[2][0] << " " << trans[2][1] << " " << trans[2][2] << " " << trans[2][3] << "\n"
//                    << trans[3][0] << " " << trans[3][1] << " " << trans[3][2] << " " << trans[3][3] << "\n";
//    transform.translate(0.0, 0.0, -5.0);
//    transform.rotate(rotation);

            render(body->mesh(), transform, true);
            render(body->hullMesh(), transform, false);
            render(body->bSphereMesh(), transform, false);

        }
    }

}

void SceneWidget::render(const std::shared_ptr<Mesh> &mesh, const QMatrix4x4& transform, bool defaultVisibility)
{
    // Identify proper settings to render the mesh
    if (mesh == nullptr) return;
    auto item = getRender(mesh, defaultVisibility);
    if (!item.visible) return;

    // Handle rendering
    auto *program = m_programs[item.programIdx];
    if (mesh->colorOverrideEnabled()) program = m_programs[0]; // Use a flat shader instead in case of color override
    program->bind();

    // TODO manage uniforms better
    // Set modelview-projection matrix
    program->setUniformValue("u_transform", transform);
    program->setUniformValue("vp_matrix", m_camera.getViewProjection());
    program->setUniformValue("mvp_matrix", m_camera.getViewProjection() * transform);
    program->setUniformValue("n_matrix", transform.normalMatrix());

    const glm::vec3& color = mesh->colorOverrideEnabled() ? mesh->colorOverride() : mesh->baseColor();
    program->setUniformValue("out_color", QVector3D(color.r, color.g, color.b));

    // Draw mesh geometry
    m_geometries[item.geometryIdx]->draw(program);
}

Camera& SceneWidget::camera()
{
    return m_camera;
}
