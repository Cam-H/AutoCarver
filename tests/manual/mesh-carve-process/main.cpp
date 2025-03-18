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
#include <Qt3DRender/QRenderCapture>

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
#include <QViewport>
#include <QCameraSelector>
#include <QRenderSurfaceSelector>
#include <QFirstPersonCameraController>
#include <QLayer>
#include <QLayerFilter>
#include <QPhongMaterial>
#include <QSphereMesh>

#include "geometry/Body.h"
#include "fileIO/MeshHandler.h"
#include "core/Sculpture.h"
#include "core/SculptProcess.h"
#include "geometry/MeshBuilder.h"

#include "renderer/RenderCapture.h"

uint32_t m_processIndex = 0;
std::vector<SculptProcess*> m_processes;
bool m_displaySculptures = true, m_displayHulls = false;

Qt3DCore::QEntity *root;
Qt3DExtras::Qt3DWindow *view;

Qt3DRender::QCamera *m_camera;
Qt3DRender::QCamera *m_cam2;

Qt3DRender::QLayer* m_testLayer = nullptr;

std::vector<std::string> m_paths;
bool m_enableSculptures = true;

RenderCapture* m_rc;

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

//    ((RenderEntity*)m_processes[idx]->sculpture())->m_renders[0]->addComponent(m_testLayer);
//    ((RenderEntity*)m_processes[idx]->sculpture())->m_renders[0]->addComponent(new Qt3DRender::QLayer);

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
    view->defaultFrameGraph()->setClearColor(Qt::gray);
    QWidget *container = QWidget::createWindowContainer(view);
    QSize screenSize = view->screen()->size();
    container->setMinimumSize(QSize(500, 500));
    container->setMaximumSize(screenSize);


//    auto vs = new Qt3DRender::QViewport();
//    auto *cameraSelector = new Qt3DRender::QCameraSelector(vs);
//    auto *cam2 = new Qt3DRender::QCamera(cameraSelector);
//    cam2->setPosition(QVector3D(0, 5, 0));
//    cam2->setViewCenter(QVector3D(0, 0, 0));
//    cameraSelector->setCamera(cam2);
//    view->setActiveFrameGraph(vs);

//    m_camera->setPosition(QVector3D(r * cos(theta), y, r * sin(theta)));
//    m_camera->setUpVector(QVector3D(0, 1, 0));
//    m_camera->setViewCenter(QVector3D(0, y, 0));


    // Set root object of the scene
    view->setRootEntity(root);

//    auto *renderSurfaceSelector = new Qt3DRender::QRenderSurfaceSelector;
//    renderSurfaceSelector->setSurface(view);
//
//    auto vs = new Qt3DRender::QViewport(renderSurfaceSelector);
////    vs->setNormalizedRect(QRectF(0, 0, 1, 1));
//
//    auto *cameraSelector = new Qt3DRender::QCameraSelector(vs);
//
//    auto clearBuffers = new Qt3DRender::QClearBuffers(cameraSelector);
//    clearBuffers->setBuffers(Qt3DRender::QClearBuffers::AllBuffers);
//    clearBuffers->setClearColor(Qt::cyan);
//
//    m_cam2 = new Qt3DRender::QCamera(cameraSelector);
////    cam2->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
////    cam2->setPosition(QVector3D(0, 5, 0));
////    cam2->setViewCenter(QVector3D(0, 0, 0));
////    cam2->setPosition(QVector3D(0, 5, 5.0f));
////    cam2->setUpVector(QVector3D(0, 1, 0));
////    cam2->setViewCenter(QVector3D(0, 0, 0));
//    m_cam2->setProjectionType(Qt3DRender::QCameraLens::OrthographicProjection);
//    m_cam2->setPosition(QVector3D(0,5,5));
//    m_cam2->setViewCenter(QVector3D(0,0,0));
//    m_cam2->lens()->setOrthographicProjection(-1, 1, -1, 1, 10, -10);
//    cameraSelector->setCamera(m_cam2);
//
//    auto cameraController = new Qt3DExtras::QFirstPersonCameraController(root);
//    cameraController->setCamera(m_cam2);
//
    m_testLayer = new Qt3DRender::QLayer;
//    m_testLayer->setRecursive(true);
    auto layerFilter = new Qt3DRender::QLayerFilter(root);// renderSurfaceSelector
    layerFilter->setFilterMode(Qt3DRender::QLayerFilter::DiscardAnyMatchingLayers);
    layerFilter->addLayer(m_testLayer);


//    view->activeFrameGraph()->setParent(renderSurfaceSelector);
//    view->setActiveFrameGraph(renderSurfaceSelector);

//    Qt3DRender::QRenderCapture* capture = new Qt3DRender::QRenderCapture;
//    view->activeFrameGraph()->setParent(capture);
//    view->setActiveFrameGraph(capture);

    QWidget *widget = new QWidget;
    QVBoxLayout *vLayout = new QVBoxLayout(widget);
    vLayout->setAlignment(Qt::AlignTop);

    QHBoxLayout *hMainLayout = new QHBoxLayout();
    hMainLayout->addWidget(container);
//    hMainLayout->addWidget(vs);
    vLayout->addLayout(hMainLayout, 1);

    QHBoxLayout *hLayout = new QHBoxLayout();
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




    QLabel *imageLabel = new QLabel();
    imageLabel->setBackgroundRole(QPalette::Base);
    imageLabel->setSizePolicy(QSizePolicy::Ignored,
                              QSizePolicy::Ignored);
    imageLabel->setScaledContents(true);
    imageLabel->resize(100,
                       100);
    imageLabel->setMinimumSize(QSize(100, 100));
    imageLabel->setMaximumSize(QSize(600, 600));
    hMainLayout->addWidget(imageLabel);

//    m_rc = new RenderCapture(capture, imageLabel);

    auto captureButton = new QPushButton("Capture", container);
    hLayout->addWidget(captureButton);

    QObject::connect(captureButton, &QPushButton::clicked, [&]() {
        m_rc->capture();
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

    auto s1 = new Qt3DCore::QEntity(root);
    auto transform1 = new Qt3DCore::QTransform;
    transform1->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    auto material1 = new Qt3DExtras::QPhongMaterial;
    material1->setAmbient(Qt::red);
    auto spheremesh1 = new Qt3DExtras::QSphereMesh;
    spheremesh1->setRadius(1.0);
    spheremesh1->setSlices(32);
    spheremesh1->setRings(32);
    s1->addComponent(material1);
    s1->addComponent(spheremesh1);
    s1->addComponent(transform1);
    s1->addComponent(m_testLayer);

    // Show window
    widget->show();
    widget->resize(1200, 700);

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

        // Have the camera orbit the origin
        float r = 4, y = 1;
        m_camera->setPosition(QVector3D(r * cos(theta), y, r * sin(theta)));
        m_camera->setUpVector(QVector3D(0, 1, 0));
        m_camera->setViewCenter(QVector3D(0, y, 0));

//        m_cam2->setPosition(QVector3D(r * cos(theta), y + r * sin(theta), 0));
//        m_cam2->setUpVector(QVector3D(0, 1, 0));
//        m_cam2->setViewCenter(QVector3D(0, y, 0));
//        if (m_bodyIndex < m_bodies.size()) {
//            QQuaternion quat = QQuaternion::fromAxisAndAngle(0, 1, 0, M_PI);
//            m_bodies[m_bodyIndex]->rotate(quat.scalar(), quat.x(), quat.y(), quat.z());
//        }

        theta += M_PI / 64;
        phi += M_PI / 16;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}