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
    mousePressPosition = QVector2D(e->position());
}

void SceneWidget::mouseReleaseEvent(QMouseEvent *e)
{
    // Mouse release position - mouse press position
    QVector2D diff = QVector2D(e->position()) - mousePressPosition;

    // Rotation axis is perpendicular to the mouse position difference
    // vector
    QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

    // Accelerate angular speed relative to the length of the mouse sweep
    qreal acc = diff.length() / 100.0;

    // Calculate new rotation axis as weighted sum
    rotationAxis = (rotationAxis * angularSpeed + n * acc).normalized();

    // Increase angular speed
    angularSpeed += acc;
}

void SceneWidget::timerEvent(QTimerEvent *)
{
    // Decrease angular speed (friction)
    angularSpeed *= 0.99;

    // Stop rotation when speed goes below threshold
    if (angularSpeed < 0.01) {
        angularSpeed = 0.0;
    } else {
        // Update rotation
        rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;

        // Request an update
        update();
    }
}

void SceneWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    addShaderProgram(R"(..\res\shaders\flat)");

//    if (m_scene != nullptr) {
//        const std::vector<Body*>& bodies = m_scene->bodies();
//
//        //
//        for (auto body : bodies) {
//            getRender(body->mesh());
//        }
////        m_geometries.push_back(new RenderGeometry(MeshHandler::loadAsMeshBody(R"(..\res\meshes\devil.obj)"), RenderGeometry::Format::VERTEX_NORMAL));
//    }


    // Use QBasicTimer because its faster than QTimer
    timer.start(12, this);
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
}
void SceneWidget::hide(uint32_t idx, Scene::Model target)
{
    std::vector<std::shared_ptr<Mesh>> sources = select(idx, target);
    for (const std::shared_ptr<Mesh>& source : sources) {
        getRender(source).visible = false;
    }
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
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    const qreal zNear = 1.0, zFar = 100.0, fov = 60.0;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
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
            render(body->mesh(), true);
            render(body->hullMesh(), false);
        }
    }
}

void SceneWidget::render(const std::shared_ptr<Mesh> &mesh, bool defaultVisibility)
{
    // Identify proper settings to render the mesh
    auto item = getRender(mesh, defaultVisibility);
    if (!item.visible) return;

    // Handle rendering

    m_programs[item.programIdx]->bind();


    // TODO get rotation from body
    // Calculate model view transformation
    QMatrix4x4 matrix;
    matrix.translate(0.0, 0.0, -5.0);
    matrix.rotate(rotation);

    // TODO manage uniforms better
    // Set modelview-projection matrix
    m_programs[item.programIdx]->setUniformValue("u_transform", matrix);
    m_programs[item.programIdx]->setUniformValue("vp_matrix", projection);
    m_programs[item.programIdx]->setUniformValue("mvp_matrix", projection * matrix);

    m_programs[item.programIdx]->setUniformValue("out_color", QVector3D(1, 1, 0));

    // Draw mesh geometry
    m_geometries[item.geometryIdx]->draw(m_programs[item.programIdx]);
}
