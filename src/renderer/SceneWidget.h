//
// Created by Cam on 2025-03-18.
//

#ifndef AUTOCARVER_SCENEWIDGET_H
#define AUTOCARVER_SCENEWIDGET_H

#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QTimer>

#include <vector>
#include <unordered_map>

#include "core/Scene.h"

#include "renderer/RenderGeometry.h"
class RenderGeometry;
#include "Camera.h"

class SceneWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
Q_OBJECT

public:
    using QOpenGLWidget::QOpenGLWidget;
    SceneWidget(QWidget* parent = nullptr);
    SceneWidget(const std::shared_ptr<Scene>& scene = nullptr, QWidget* parent = nullptr);
    ~SceneWidget();

    void setScene(const std::shared_ptr<Scene>& scene);

    void addShaderProgram(const std::string& name);
    void createDefaultShaderProgram(const std::string& name);
    void setDefaultShaderProgram(uint32_t idx);

    void show(uint32_t idx, Scene::Model target = Scene::Model::ALL);
    void hide(uint32_t idx, Scene::Model target = Scene::Model::ALL);

    void showAll(Scene::Model target = Scene::Model::ALL);
    void hideAll(Scene::Model target = Scene::Model::ALL);


    void start();
    void pause();

    void setUpdateInterval(std::chrono::milliseconds msec);
    void setTargetFPS(uint32_t target);

    void clear();

    Camera& camera();

protected:

    struct RenderItem {
        uint32_t geometryIdx;
        uint32_t programIdx;
        bool visible; // TODO separate if render items are reused for multiple bodies
    };

    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void updateRenderGeometry(const std::shared_ptr<Mesh>& mesh);

private:

    void paint();

    void render(const std::shared_ptr<Mesh>& mesh, const QMatrix4x4& transform, bool defaultVisibility);

    void show(const std::vector<std::shared_ptr<Mesh>>& selection);
    void hide(const std::vector<std::shared_ptr<Mesh>>& selection);

    std::vector<std::shared_ptr<Mesh>> select(uint32_t idx, Scene::Model target);
    std::vector<std::shared_ptr<Mesh>> selectAll(Scene::Model target);

    RenderItem& getRender(const std::shared_ptr<Mesh>& mesh, bool defaultVisibility = true);

protected:

    std::shared_ptr<Scene> m_scene;

private:

    QTimer *m_timer;
    std::chrono::milliseconds m_interval;

    std::unordered_map<std::shared_ptr<Mesh>, RenderItem> m_renderMap;

    std::vector<QOpenGLShaderProgram*> m_programs;
    uint32_t m_defaultProgramIdx;

    std::vector<RenderGeometry*> m_geometries;


    /* ******* CAMERA CONTROLS ******** */

    Camera m_camera;

    float m_minRadius;
    float m_maxRadius;

    // Camera sensitivity
    float m_translationSensitivity;
    float m_rotationSensitivity;
    float m_zoomSensitivity;
    float m_zoomExponential;

    // Inputs
    QVector2D m_mouseLastPosition;

};


#endif //AUTOCARVER_SCENEWIDGET_H