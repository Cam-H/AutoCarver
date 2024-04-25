////
//// Created by cameronh on 23/04/24.
////
//
//#ifndef AUTOCARVER_SCENEVIEWWIDGET_H
//#define AUTOCARVER_SCENEVIEWWIDGET_H
//
//#include <QBasicTimer>
//#include <QKeyEvent>
//#include <QOpenGLWidget>
//#include <QOpenGLFunctions>
//#include <QOpenGLShaderProgram>
//
//#include "../renderer/Camera.h"
//#include "../renderer/MonitorCameraController.h"
//#include "../renderer/VertexArray.h"
//
//class SceneViewWidget : public QOpenGLWidget, protected QOpenGLFunctions
//{
//Q_OBJECT
//
//public:
//    using QOpenGLWidget::QOpenGLWidget;
//
//    SceneViewWidget();
//    ~SceneViewWidget();
//
//protected:
//
//    void initializeGL() override;
//    void initShaders();
//
//    void resizeGL(int width, int height) override;
//
//    void paintGL() override;
//
////    void mouseMoveEvent(QMouseEvent *e) override;
////    void keyPressEvent(QKeyEvent *e) override;
////    void keyReleaseEvent(QKeyEvent *e) override;
////    void timerEvent(QTimerEvent *e) override;
//
//public slots:
//    void cameraMoved();
//
//private:
//    QBasicTimer m_timer;
//    QOpenGLShaderProgram m_program;
//
//    VertexArray *arr = nullptr;
//
//    Camera m_camera;
//    MonitorCameraController m_controller;
//
//};
//
//
//#endif //AUTOCARVER_SCENEVIEWWIDGET_H
