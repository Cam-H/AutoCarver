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
#include "geometry/MeshBuilder.h"

uint32_t m_processIndex = 0;
std::vector<SculptProcess*> m_processes;
bool m_displaySculptures = true, m_displayHulls = false;

Qt3DCore::QEntity *root;
Qt3DExtras::Qt3DWindow *view;

Qt3DRender::QCamera *m_camera;

std::vector<std::string> m_paths;
bool m_enableSculptures = true;


void updateSculptureDisplay();
[[noreturn]] void update();

void plan(SculptProcess* process)
{
    process->skipConvexTrim(true);
    process->processCut(false);
    process->plan();
}

void prepare(uint32_t idx)
{
    std::cout << "Prepare\n";
    auto model = MeshHandler::loadAsMeshBody(m_paths[idx]);
    m_processes.push_back(new SculptProcess(model));
    m_processes[idx]->linkRenderer(root, view);
    m_processes[idx]->showAll();

    plan(m_processes[idx]);
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    std::string source = "..\\res\\";
    m_paths = {
//            source + "cube.obj",
//            source + "colplate.obj"
            source + "pommel.obj",
            source + "devil.obj",
            source + "beshon.obj",
            source + "spot.obj",
            source + "teddy.obj",
            source + "ogre.obj"
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


//    for (uint32_t i = 0; i < 13; i++) {
//        auto col = MeshHandler::loadAsMeshBody("..\\out\\step" + std::to_string(i) + ".obj");
//        MeshHandler::exportMesh(MeshBuilder::cleaned(col->vertices(), col->faces()), "..\\out\\step" + std::to_string(i) + "C.obj");
//    }
auto vertices = new float[] {
        -0.318407, 0.735962, -0.00818602,
        -0.136813, 0.995899, -0.178975,
        -0.142492, 0.93009, -0.5,
        -0.321246, 0.644504, -0.5,
        -0.5, 0.358919, -0.5,
        -0.5, 0.476025, 0.162603

};

auto faces = new uint32_t[] {
    1, 2, 3, 0,
    4, 5, 0, 3
};

auto sizes = new uint32_t[] {4, 4};
//auto col = std::make_shared<Mesh>(vertices, 6, faces, sizes, 2);
//    auto col = MeshHandler::loadAsMeshBody("..\\out\\plane7.obj");
//    MeshHandler::exportMesh(col, "..\\out\\plane7T.obj");
//    MeshHandler::exportMesh(MeshBuilder::cleaned(col), "..\\out\\plane7C.obj");

//        MeshHandler::exportMesh(MeshBuilder::cleaned(col), "..\\out\\step4C.obj");
//    MeshHandler::exportMesh(MeshHandler::loadAsMeshBody("..\\out\\plane3.obj"), "..\\out\\plane3C.obj");
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
        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->hideAll();
        if (m_processIndex > 0) m_processIndex--;
        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->showAll();

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

        // Allows meshes to be prepared at user demand, rather than all at once
        if (m_enableSculptures && m_processes.size() < m_paths.size() && m_processIndex + 1 >= m_processes.size()) {
            prepare(m_processes.size());
        }

        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->hideAll();
        if (m_processIndex < m_processes.size() - 1) m_processIndex++;
        if (m_processIndex < m_processes.size()) m_processes[m_processIndex]->showAll();

        updateSculptureDisplay();

    });

    // Show the first body
    prepare(m_processIndex);


    auto thread = std::thread(update);

    // Show window
    widget->show();
    widget->resize(700, 700);

    return app.exec();
}

void updateSculptureDisplay()
{
    if (m_processIndex < m_processes.size()) {
        if (m_displaySculptures) m_processes[m_processIndex]->show(1);
        else m_processes[m_processIndex]->hide(1);

        if (m_displayHulls) m_processes[m_processIndex]->show(0, Scene::Model::HULL);
        else m_processes[m_processIndex]->hide(0, Scene::Model::HULL);
    }
}

[[noreturn]] void update()
{
    float theta = 0, phi = 0;

    while (true) {

        float r = 4, y = 1;
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