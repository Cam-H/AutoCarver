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
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#include <vector>
#include <unordered_map>

#include "core/Scene.h"

#include "renderer/RenderGeometry.h"
class RenderGeometry;

class SceneWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
Q_OBJECT

public:
    using QOpenGLWidget::QOpenGLWidget;
    SceneWidget(Scene* scene = nullptr, QWidget* parent = nullptr);
    ~SceneWidget();

    void addShaderProgram(const std::string& name);
    void createDefaultShaderProgram(const std::string& name);
    void setDefaultShaderProgram(uint32_t idx);

//    void showAll();
//    void hideAll();

    void show(uint32_t idx, Scene::Model target = Scene::Model::ALL);
    void hide(uint32_t idx, Scene::Model target = Scene::Model::ALL);

protected:

    struct RenderItem {
        uint32_t geometryIdx;
        uint32_t programIdx;
        bool visible; // TODO separate if render items are reused for multiple bodies
    };

    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void timerEvent(QTimerEvent *e) override;

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:

    void render(const std::shared_ptr<Mesh>& mesh, bool defaultVisibility);

    std::vector<std::shared_ptr<Mesh>> select(uint32_t idx, Scene::Model target);

    RenderItem& getRender(const std::shared_ptr<Mesh>& mesh, bool defaultVisibility = true);

private:

    std::unordered_map<std::shared_ptr<Mesh>, RenderItem> m_renderMap;

    Scene* m_scene;

    QBasicTimer timer;

    std::vector<QOpenGLShaderProgram*> m_programs;
    uint32_t m_defaultProgramIdx;

    std::vector<RenderGeometry*> m_geometries;

    QMatrix4x4 projection;

    QVector2D mousePressPosition;
    QVector3D rotationAxis;
    qreal angularSpeed = 0;
    QQuaternion rotation;
};


#endif //AUTOCARVER_SCENEWIDGET_H