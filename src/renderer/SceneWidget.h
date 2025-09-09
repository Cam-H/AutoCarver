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

#include "geometry/primitives/Ray.h"
#include "core/Scene.h"

class RenderBuffer;
class RenderGeometry;
#include "Camera.h"

class SceneWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
Q_OBJECT

signals:
    void perspectiveChanged();
    void mousepick(Ray ray);

public:
    using QOpenGLWidget::QOpenGLWidget;
    SceneWidget(QWidget* parent = nullptr);
    SceneWidget(const std::shared_ptr<Scene>& scene, QWidget* parent = nullptr);
    ~SceneWidget();

    void setScene(const std::shared_ptr<Scene>& scene);

    void addShaderProgram(const std::string& name);
    void createDefaultShaderProgram(const std::string& name);
    void setDefaultShaderProgram(uint32_t idx);

    void show(uint32_t ID, Scene::Model target = Scene::Model::ALL);
    void hide(uint32_t ID, Scene::Model target = Scene::Model::ALL);
    void setVisibility(bool visible, uint32_t ID, Scene::Model target = Scene::Model::ALL);

    void showAll(Scene::Model target = Scene::Model::ALL);
    void hideAll(Scene::Model target = Scene::Model::ALL);
    void setVisibility(bool visible, Scene::Model target = Scene::Model::ALL);

    void setTitle(const std::string& title);

    void start();
    void pause();

    void setUpdateInterval(std::chrono::milliseconds msec);
    void setTargetFPS(uint32_t target);

    void clear();

    void updateRenderGeometry(const std::shared_ptr<Mesh>& mesh);

    Camera& camera();

protected:

    struct RenderSettings {
        bool meshVisibility;
        bool hullVisibility;
        bool boundsVisibility;
        bool axesVisibility;
    };

    struct RenderItem {
        uint32_t geometryIdx;
        uint32_t programIdx;
    };

    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;


private:

    void paint();

    void render(const std::shared_ptr<Mesh>& mesh, const QMatrix4x4& transform);

    RenderSettings& getSettings(uint32_t ID);
    RenderItem& getGeometryBuffer(const std::shared_ptr<Mesh>& mesh);

    std::shared_ptr<Mesh> hullMesh(const std::shared_ptr<Mesh>& mesh);

protected:

    std::shared_ptr<RenderBuffer> m_buffer;

private:

    QString m_title;

    QTimer *m_timer;
    std::chrono::milliseconds m_interval;

    std::unordered_map<std::shared_ptr<Mesh>, RenderItem> m_renderMap;
    std::vector<RenderSettings> m_settings;

    std::vector<QOpenGLShaderProgram*> m_programs;
    uint32_t m_defaultProgramIdx;

    std::vector<RenderGeometry*> m_geometries;

    std::shared_ptr<Mesh> m_sphere;
    std::shared_ptr<Mesh> m_axes;

    std::unordered_map<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> m_hulls;

    /* ******* CAMERA CONTROLS ******** */

    Camera m_camera;

    double m_minRadius;
    double m_maxRadius;

    // Camera sensitivity
    double m_translationSensitivity;
    double m_rotationSensitivity;
    double m_zoomSensitivity;
    double m_zoomExponential;

    // Inputs
    QVector2D m_mouseLastPosition;

};


#endif //AUTOCARVER_SCENEWIDGET_H