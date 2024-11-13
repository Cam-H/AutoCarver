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
#include "fileIO/MeshHandler.h"
#include "core/Sculpture.h"
#include "core/SculptProcess.h"

uint32_t m_processIndex = 0;
std::vector<SculptProcess*> m_processes;
bool m_displaySculptures = true, m_displayHulls = false;

Qt3DCore::QEntity *root;
Qt3DExtras::Qt3DWindow *view;

Qt3DRender::QCamera *m_camera;


void updateSculptureDisplay();
[[noreturn]] void update();

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    std::string source = "..\\res\\";
    std::vector<std::string> paths = {
//            source + "cube.obj",
            source + "devil.obj"
//            source + "beshon.obj",
//            source + "spot.obj",
//            source + "teddy.obj",
//            source + "ogre.obj",
//            source + "bunny.obj",
//            source + "dragon.ply",
//            source + "HollowCylinder.obj",
//            source + "emblem.obj",
//            source + "stone.obj",
//            source + "spider.obj",
//            source + "horse.obj",
//            source + "hornbug.obj",
//            source + "caterpillar.obj"
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
    m_camera = view->camera();

    m_camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    m_camera->setPosition(QVector3D(0, 4, 20.0f));
    m_camera->setUpVector(QVector3D(0, 1, 0));
    m_camera->setViewCenter(QVector3D(0, 4, 0));


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
//        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->hide();
//        if (m_processIndex > 0) m_processIndex--;
//        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->show(Body::Model::MESH);

        updateSculptureDisplay();
    });


    auto sculptureButton = new QCheckBox("Show sculpture", container);
    sculptureButton->setChecked(true);
    hLayout->addWidget(sculptureButton);

    QObject::connect(sculptureButton, &QCheckBox::clicked, [&](bool checked) {
        m_displaySculptures = checked;

        updateSculptureDisplay();
    });

    auto hullButton = new QCheckBox("Show convex hull", container);
    hLayout->addWidget(hullButton);

    QObject::connect(hullButton, &QCheckBox::clicked, [&](bool checked) {
        m_displayHulls = checked;

        updateSculptureDisplay();
    });

    auto stepButton = new QPushButton("Next step", container);
    hLayout->addWidget(stepButton);

    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        if (m_processIndex < m_processes.size()) {
            m_processes[m_processIndex]->next();
        }
    });


    auto nextButton = new QPushButton(">", container);
    nextButton->setMaximumWidth(50);
    hLayout->addWidget(nextButton);

    QObject::connect(nextButton, &QPushButton::clicked, [&]() {
//        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->hide();
//        if (m_processIndex < m_processes.size() - 1) m_processIndex++;
//        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->show(Body::Model::MESH);

        updateSculptureDisplay();
    });


    // Prepare all test bodies
    for (const std::string& path : paths) {
        auto model = MeshHandler::loadAsMeshBody(path);

        m_processes.push_back(new SculptProcess(model));
        m_processes[m_processes.size() - 1]->linkRenderer(root, view);
//        auto body = new Sculpture(model);
//        body->hull(); // Induce calculation of convex hull
//        body->setRenderer(root, view);
//        body->hide();

        m_processes[m_processes.size() - 1]->hideAll();

    }

    // Show current body
    if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->showAll();


    auto thread = std::thread(update);

    // Show window
    widget->show();
    widget->resize(700, 700);

    return app.exec();
}

void updateSculptureDisplay()
{
    if (m_processIndex < m_processes.size()) {
        if (m_displaySculptures) m_processes[m_processIndex]->show(0);
        else m_processes[m_processIndex]->hide(0);

        if (m_displayHulls) m_processes[m_processIndex]->show(1, Scene::Model::HULL);
        else m_processes[m_processIndex]->hide(1, Scene::Model::HULL);
    }
}

[[noreturn]] void update()
{
    float theta = 0, phi = 0;

    while (true) {

        float r = 20, y = 1;
        m_camera->setPosition(QVector3D(r * cos(theta), y, r * sin(theta)));
        m_camera->setUpVector(QVector3D(0, 1, 0));
        m_camera->setViewCenter(QVector3D(0, y, 0));
//        if (m_bodyIndex < m_bodies.size()) {
//            QQuaternion quat = QQuaternion::fromAxisAndAngle(0, 1, 0, M_PI);
//            m_bodies[m_bodyIndex]->rotate(quat.scalar(), quat.x(), quat.y(), quat.z());
//        }

        theta += M_PI / 64;
        phi += M_PI / 16;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}