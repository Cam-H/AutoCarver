#include <QApplication>
#include <QPushButton>

#include <QSurfaceFormat>
#include <QLabel>
#include <QSlider>
#include <QLine>

#include <QGuiApplication>


#include <QPainter>
#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qcameralens.h>

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QCommandLinkButton>
#include <QtGui/QScreen>

#include <Qt3DRender/qtechnique.h>
#include <Qt3DRender/qmaterial.h>
#include <Qt3DRender/qeffect.h>
#include <Qt3DRender/qtexture.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/qsceneloader.h>
#include <Qt3DRender/qpointlight.h>

#include <Qt3DCore/qaspectengine.h>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/qt3dwindow.h>

#include <iostream>

#include "../renderer/MonitorCameraController.h"

#include "NotScene.h"

#ifndef QT_NO_OPENGL
#include "../widgets/SceneViewWidget.h"
#endif


int main(int argc, char *argv[]) {
//    std::cout << argc << " " << argv[0] << "\n";
    QApplication app(argc, argv);

    // Root entity
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();
    NotScene *scene = new NotScene(rootEntity, argv[1]);

//    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    scene->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
    QWidget *container = QWidget::createWindowContainer(scene);
    QSize screenSize = scene->screen()->size();
    container->setMinimumSize(QSize(200, 100));
    container->setMaximumSize(screenSize);

    QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->setAlignment(Qt::AlignTop);
    hLayout->addWidget(container, 1);
    hLayout->addLayout(vLayout);

    widget->setWindowTitle(QStringLiteral("Auto Carver"));

    // Camera
    Qt3DRender::QCamera *cameraEntity = scene->camera();

    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(0, 0, 20.0f));
    cameraEntity->setUpVector(QVector3D(0, 1, 0));
    cameraEntity->setViewCenter(QVector3D(0, 0, 0));


    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(cameraEntity->position());
    lightEntity->addComponent(lightTransform);

    // For camera controls
    MonitorCameraController *camController = new MonitorCameraController(rootEntity);
    camController->setCamera(cameraEntity);
    camController->linkContainer(container);

    // Set root object of the scene
    scene->setRootEntity(rootEntity);

    // Create control widgets

    QSlider *rxSlider = new QSlider(Qt::Orientation::Horizontal, widget);
    rxSlider->setMinimumWidth(200);
    rxSlider->setMinimum(-100);
    QSlider *rySlider = new QSlider(Qt::Orientation::Horizontal, widget);
    rySlider->setMinimum(-100);
    QSlider *rzSlider = new QSlider(Qt::Orientation::Horizontal, widget);
    rzSlider->setMinimum(-100);

    QSlider *cutSlider = new QSlider(Qt::Orientation::Horizontal, widget);
    cutSlider->setMinimum(-40);
    cutSlider->setMaximum(100);

    QObject::connect(rxSlider, &QSlider::valueChanged, scene, &NotScene::apexX);
    QObject::connect(rySlider, &QSlider::valueChanged, scene, &NotScene::apexY);
    QObject::connect(rzSlider, &QSlider::valueChanged, scene, &NotScene::apexZ);
    QObject::connect(cutSlider, &QSlider::valueChanged, scene, &NotScene::cut);


    QCheckBox *cutPlaneShowCB = new QCheckBox(widget);
    cutPlaneShowCB->setChecked(false);
    cutPlaneShowCB->setText(QStringLiteral("Show cut plane"));
    QObject::connect(cutPlaneShowCB, &QCheckBox::stateChanged, scene, &NotScene::show);

    vLayout->addWidget(rxSlider);
    vLayout->addWidget(rySlider);
    vLayout->addWidget(rzSlider);
    vLayout->addWidget(rzSlider);

    vLayout->addWidget(cutSlider);

    vLayout->addWidget(cutPlaneShowCB);

    // Show window
    widget->show();
    widget->resize(1200, 800);

    return app.exec();
}
