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
#include <Qt3DExtras/QPhongMaterial>


#include <thread>
#include <chrono>
#include <iostream>
#include <QRandomGenerator>

#include "geometry/Tesselation.h"
#include "geometry/GeometryBuilder.h"
#include "geometry/Body.h"
#include "fileIO/MeshLoader.h"

uint32_t m_bodyIndex = 0;
std::vector<Body*> m_bodies;

bool m_displayHulls = false;

Qt3DCore::QEntity *root;
Qt3DExtras::Qt3DWindow *view;

void updateHullDisplay();
[[noreturn]] void update();

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    std::string source = "..\\res\\";
    std::vector<std::pair<std::string, float>> paths = {
            {source + "cube.obj", 2.0f},
            {source + "HollowCylinder.obj", 2.0f},
            {source + "emblem.obj", 1.4f},
            {source + "stone.obj", 2.0f},
            {source + "devil.obj", 2.0f},
            {source + "spot.obj", 4.0f},
            {source + "beshon.obj", 4.0f},
            {source + "spider.obj", 0.06f},
            {source + "horse.obj", 50.0f},
            {source + "hornbug.obj", 4.0f},
            {source + "caterpillar.obj", 0.05f},
            {source + "teddy.obj", 0.3f},
            {source + "ogre.obj", 0.3f},
            {source + "bunny.obj", 0.6f},
            {source + "dragon.ply", 40.0f}


    };
//    std::vector<std::pair<std::string, float>> paths = {
//            {source + "cube.obj", 2.0f}
//    };

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
        if (m_bodyIndex < m_bodies.size()) m_bodies[m_bodyIndex]->hide();
        if (m_bodyIndex > 0) m_bodyIndex--;
        if (m_bodyIndex < m_bodies.size()) m_bodies[m_bodyIndex]->show(Body::Model::MESH);

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
        if (m_bodyIndex < m_bodies.size()) m_bodies[m_bodyIndex]->hide();
        if (m_bodyIndex < m_bodies.size() - 1) m_bodyIndex++;
        if (m_bodyIndex < m_bodies.size()) m_bodies[m_bodyIndex]->show(Body::Model::MESH);

        updateHullDisplay();
    });


    // Prepare all test bodies
    for (const std::pair<std::string, float>& path : paths) {
        auto mesh = std::make_shared<Mesh>(MeshLoader::loadAsMeshBody(path.first, path.second));
//        mesh->setBaseColor(QColor::fromRgbF(1, 1, 0.2));
//        for (uint32_t i = 0; i < mesh->triangleCount(); i += 2) {
//            mesh->setFaceColor(i, QColor::fromRgbF(0, 0, 1));
//        }
        auto body = new Body(mesh);
        body->hull(); // Induce calculation of convex hull
        body->setRenderer(root, view);
        body->hide();

        //TODO
//        body->mesh().setBaseColor(QColor::fromRgbF(1, 1, 0.2));
//        for (uint32_t i = 0; i < body->mesh().triangleCount(); i += 2) {
//            body->mesh().setFaceColor(i, QColor::fromRgbF(0, 0, 1));
//        }

        m_bodies.push_back(body);
    }

    // Show current body
    if (m_bodyIndex < m_bodies.size()) m_bodies[m_bodyIndex]->show(Body::Model::MESH);


    auto thread = std::thread(update);

    // Show window
    widget->show();
    widget->resize(700, 700);

    return app.exec();
}

void updateHullDisplay()
{
    if (m_bodyIndex < m_bodies.size()) {
        if (m_displayHulls) m_bodies[m_bodyIndex]->show(Body::Model::HULL);
        else m_bodies[m_bodyIndex]->hide(Body::Model::HULL);
    }
}

[[noreturn]] void update()
{
    float theta = 0, phi = 0;

    while (true) {

        if (m_bodyIndex < m_bodies.size()) {
            QQuaternion quat = QQuaternion::fromAxisAndAngle(1, 0, 0, M_PI / 2) * QQuaternion::fromAxisAndAngle(0, 1, 0, M_PI);
            m_bodies[m_bodyIndex]->rotate(quat.scalar(), quat.x(), quat.y(), quat.z());
        }

        theta += M_PI;
        phi += M_PI / 16;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}