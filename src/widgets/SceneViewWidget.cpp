////
//// Created by cameronh on 23/04/24.
////
//
//#include "SceneViewWidget.h"
//
//#include <QStackedLayout>
//#include <QPushButton>
//
//#include <iostream>
//
//SceneViewWidget::SceneViewWidget()
//    : m_camera(QVector3D(0, 0, 0)),
//      m_controller(&m_camera)
//{
//
//    QStackedLayout *layout = new QStackedLayout(this);
//    layout->addWidget(&m_controller);
//    setLayout(layout);
//
//    resize(640, 480);
//
//    QObject::connect(&m_camera, &Camera::poseChanged, this, &SceneViewWidget::cameraMoved);
//}
//
//SceneViewWidget::~SceneViewWidget()
//{
//    makeCurrent();
//    // TODO delete stuff
//    delete arr;
//    doneCurrent();
//}
//
//void SceneViewWidget::initializeGL()
//{
//    initializeOpenGLFunctions();
//
//    glClearColor(0, 0, 0, 1);
//
//    initShaders();
//
//    arr = new VertexArray;
//
//}
//
//void SceneViewWidget::initShaders()
//{
//    // Compile vertex shader
//    if (!m_program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/res/vshader.glsl")) close();
//
//    // Compile fragment shader
//    if (!m_program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/res/fshader.glsl")) close();
//
//    // Link shader pipeline
//    if (!m_program.link()) close();
//
//    // Bind shader pipeline
//    if (!m_program.bind()) close();
//}
//
//void SceneViewWidget::resizeGL(int width, int height)
//{
//    qreal aspect = qreal(width) / qreal(height ? height : 1);
//    m_camera.setAspectRatio(aspect);
//}
//
//void SceneViewWidget::paintGL()
//{
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//    glEnable(GL_DEPTH_TEST);
//
//    glEnable(GL_CULL_FACE);
//
//    m_program.bind();
//
//    QMatrix4x4 matrix;
//    matrix.translate(0.0, 0.0, -5.0);
//
//    // Set model-view-projection matrix
//    m_program.setUniformValue("mvp_matrix", m_camera.viewProjectionMatrix() * matrix);
//
//    // Draw geometry
//    arr->draw(&m_program);
//
//}
//
//void SceneViewWidget::cameraMoved()
//{
//    update();
//}