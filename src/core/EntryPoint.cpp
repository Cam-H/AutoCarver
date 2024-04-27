#include <QApplication>
#include <QPushButton>

#include <QSurfaceFormat>
#include <QLabel>

#include <QGuiApplication>

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
#include "Scene.h"

#ifndef QT_NO_OPENGL
#include "../widgets/SceneViewWidget.h"
#endif

int main(int argc, char *argv[]) {
//    std::cout << argc << " " << argv[0] << "\n";
    QApplication app(argc, argv);
    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
    QWidget *container = QWidget::createWindowContainer(view);
    QSize screenSize = view->screen()->size();
    container->setMinimumSize(QSize(200, 100));
    container->setMaximumSize(screenSize);

    QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->setAlignment(Qt::AlignTop);
    hLayout->addWidget(container, 1);
    hLayout->addLayout(vLayout);

    widget->setWindowTitle(QStringLiteral("Auto Carver"));

    // Root entity
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

    // Camera
    Qt3DRender::QCamera *cameraEntity = view->camera();

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

    Scene *scene = new Scene(rootEntity);

    // Set root object of the scene
    view->setRootEntity(rootEntity);

    // Create control widgets
    QCommandLinkButton *info = new QCommandLinkButton();
    info->setText(QStringLiteral("Qt3D ready-made meshes"));
    info->setDescription(QString::fromLatin1("Qt3D provides several ready-made meshes, like torus, cylinder, cone, "
                                             "cube, plane and sphere."));
    info->setIconSize(QSize(0,0));

    QCheckBox *torusCB = new QCheckBox(widget);
    torusCB->setChecked(true);
    torusCB->setText(QStringLiteral("Torus"));

    QCheckBox *coneCB = new QCheckBox(widget);
    coneCB->setChecked(true);
    coneCB->setText(QStringLiteral("Cone"));

    QCheckBox *cylinderCB = new QCheckBox(widget);
    cylinderCB->setChecked(true);
    cylinderCB->setText(QStringLiteral("Cylinder"));

    QCheckBox *cuboidCB = new QCheckBox(widget);
    cuboidCB->setChecked(true);
    cuboidCB->setText(QStringLiteral("Cuboid"));

    QCheckBox *planeCB = new QCheckBox(widget);
    planeCB->setChecked(true);
    planeCB->setText(QStringLiteral("Plane"));

    QCheckBox *sphereCB = new QCheckBox(widget);
    sphereCB->setChecked(true);
    sphereCB->setText(QStringLiteral("Sphere"));

    vLayout->addWidget(info);
    vLayout->addWidget(torusCB);
    vLayout->addWidget(coneCB);
    vLayout->addWidget(cylinderCB);
    vLayout->addWidget(cuboidCB);
    vLayout->addWidget(planeCB);
    vLayout->addWidget(sphereCB);

    // Show window
    widget->show();
    widget->resize(1200, 800);

    return app.exec();
}
