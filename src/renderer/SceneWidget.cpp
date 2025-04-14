//
// Created by Cam on 2025-03-18.
//

#include "SceneWidget.h"

#include <QMouseEvent>

#include <cmath>
#include <iostream>

SceneWidget::SceneWidget(const std::shared_ptr<Scene>& scene, QWidget* parent)
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
            QVector3D vert = cameraRotated(UP_VECTOR).normalized();

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
    m_viewProjection.lookAt(m_eye, m_center, UP_VECTOR);

}

QVector3D SceneWidget::cameraRotated(QVector3D base) const
{
    base = QQuaternion::fromAxisAndAngle(0, 0, 1, m_pitch) * base; // Apply pitch to eye
    return QQuaternion::fromAxisAndAngle(UP_VECTOR, m_yaw) * base; // Apply yaw to eye
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
    if (idx >= m_scene->bodyCount()) return {};

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
            break;
    }

    return selection;
}

std::vector<std::shared_ptr<Mesh>> SceneWidget::selectAll(Scene::Model target)
{
    std::vector<std::shared_ptr<Mesh>> selection;

    switch (target) {
        case Scene::Model::ALL:
            for (const std::shared_ptr<Body>& body : m_scene->bodies()) selection.push_back(body->mesh());
            for (const std::shared_ptr<Body>& body : m_scene->bodies()) {
                if (body->hullMesh() != nullptr) selection.push_back(body->hullMesh());
            }
            break;
        case Scene::Model::MESH:
            for (const std::shared_ptr<Body>& body : m_scene->bodies()) selection.push_back(body->mesh());
            break;
        case Scene::Model::HULL:
            for (const std::shared_ptr<Body>& body : m_scene->bodies()) {
                if (body->hullMesh() != nullptr) selection.push_back(body->hullMesh());
            }
            break;
        case Scene::Model::BOUNDING_SPHERE:
            break;
    }

    return selection;
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
        const std::vector<std::shared_ptr<Body>>& bodies = m_scene->bodies();

        for (const std::shared_ptr<Body>& body : bodies) {
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

    m_programs[item.programIdx]->bind();

    // TODO manage uniforms better
    // Set modelview-projection matrix
    m_programs[item.programIdx]->setUniformValue("u_transform", transform);
    m_programs[item.programIdx]->setUniformValue("vp_matrix", m_viewProjection);
    m_programs[item.programIdx]->setUniformValue("mvp_matrix", m_viewProjection * transform);
    m_programs[item.programIdx]->setUniformValue("n_matrix", transform.normalMatrix());

    const glm::vec3& base = mesh->baseColor();
    m_programs[item.programIdx]->setUniformValue("out_color", QVector3D(base.r, base.g, base.b));

    // Draw mesh geometry
    m_geometries[item.geometryIdx]->draw(m_programs[item.programIdx]);
}
