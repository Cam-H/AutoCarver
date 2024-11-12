#include <QApplication>

#include <QLabel>
#include <QPushButton>
#include <QCheckBox>

#include <QPainter>
#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qentity.h>

#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtGui/QScreen>

#include <Qt3DRender/qmaterial.h>
#include <Qt3DRender/qtexture.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/qpointlight.h>

#include <Qt3DCore/qaspectengine.h>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/qt3dwindow.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <QRandomGenerator>

#include "geometry/Body.h"
#include "fileIO/MeshLoader.h"

#include "core/Scene.h"
#include "core/Sculpture.h"

uint32_t m_bodyIndex = 0;
Scene *m_scene;

bool m_displayHulls = false;

Qt3DCore::QEntity *root;
Qt3DExtras::Qt3DWindow *view;

void updateHullDisplay();
[[noreturn]] void update();

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    std::string source = "..\\res\\";
    std::vector<std::string> paths = {
            source + "cube.obj",
            source + "HollowCylinder.obj",
            source + "emblem.obj",
            source + "stone.obj",
            source + "devil.obj",
            source + "spot.obj",
            source + "beshon.obj",
            source + "spider.obj",
            source + "horse.obj",
            source + "hornbug.obj",
            source + "caterpillar.obj",
            source + "teddy.obj",
            source + "ogre.obj",
            source + "bunny.obj",
            source + "dragon.ply"
    };

    // Root entity
    root = new Qt3DCore::QEntity();

    view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
    QWidget *container = QWidget::createWindowContainer(view);
    QSize screenSize = view->screen()->size();
    container->setMinimumSize(QSize(200, 100));
    container->setMaximumSize(screenSize);

    // Set root object of the scene
    view->setRootEntity(root);

    QWidget *widget = new QWidget;
    QVBoxLayout *vLayout = new QVBoxLayout(widget);
    QHBoxLayout *hLayout = new QHBoxLayout();
    vLayout->setAlignment(Qt::AlignTop);
    vLayout->addWidget(container, 1);
    vLayout->addLayout(hLayout);

    widget->setWindowTitle(QStringLiteral("Auto Carver - Convex Hull Algorithm Testing"));

    // Camera
    Qt3DRender::QCamera *cameraEntity = view->camera();

    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(0, 0, 20.0f));
    cameraEntity->setUpVector(QVector3D(0, 1, 0));
    cameraEntity->setViewCenter(QVector3D(0, 0, 0));


    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(root);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(0, 10, 0));
    lightEntity->addComponent(lightTransform);


    auto prevButton = new QPushButton("<", container);
    prevButton->setMaximumWidth(50);
    hLayout->addWidget(prevButton);

    QObject::connect(prevButton, &QPushButton::clicked, [&]() {
        m_scene->hide(m_bodyIndex);

        if (m_bodyIndex > 0) m_bodyIndex--;
        m_scene->show(m_bodyIndex, Scene::Model::MESH);

        updateHullDisplay();
    });


    auto hullButton = new QCheckBox("Show convex hull", container);
    hLayout->addWidget(hullButton);

    QObject::connect(hullButton, &QCheckBox::clicked, [&](bool checked) {
        m_displayHulls = checked;

        updateHullDisplay();
    });


    auto nextButton = new QPushButton(">", container);
    nextButton->setMaximumWidth(50);
    hLayout->addWidget(nextButton);

    QObject::connect(nextButton, &QPushButton::clicked, [&]() {
        m_scene->hide(m_bodyIndex);

        if (m_bodyIndex < m_scene->bodyCount() - 1) m_bodyIndex++;
        m_scene->show(m_bodyIndex, Scene::Model::MESH);

        updateHullDisplay();
    });

    m_scene = new Scene();
    m_scene->linkRenderer(root, view);

    // Prepare all test bodies
    for (const std::string& path : paths) {
        auto mesh = MeshLoader::loadAsMeshBody(path, 10.0f);
        // TODO| Note: normals not calculated properly if sufficiently small mesh is loaded

        // Use sculpture positioning function to conveniently transform the mesh
        auto temp = Sculpture(mesh, 12.0f, 12.0f);
        mesh->translate(0, -temp.height() / 2, 0);

        m_scene->createBody(mesh);
    }

    m_scene->hideAll();

    // Show current body
    if (m_bodyIndex < m_scene->bodyCount()) m_scene->show(m_bodyIndex, Scene::Model::MESH);


    auto thread = std::thread(update);

    // Show window
    widget->show();
    widget->resize(700, 700);

    return app.exec();
}

void updateHullDisplay()
{
    if (m_bodyIndex < m_scene->bodyCount()) {
        if (m_displayHulls) m_scene->show(m_bodyIndex, Scene::Model::HULL);
        else m_scene->hide(m_bodyIndex, Scene::Model::HULL);
    }
}

[[noreturn]] void update()
{
    float theta = 0, phi = 0;

    while (true) {

        QQuaternion quat = QQuaternion::fromAxisAndAngle(1, 0, 0, M_PI / 2) * QQuaternion::fromAxisAndAngle(0, 1, 0, M_PI);
        m_scene->rotateBody(m_bodyIndex, quat.scalar(), quat.x(), quat.y(), quat.z());

        theta += M_PI;
        phi += M_PI / 16;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}