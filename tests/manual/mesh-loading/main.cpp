#include <QApplication>

#include <QLabel>


#include <QPainter>
#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qentity.h>

#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
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

#include <Qt3DRender/QMesh>
#include <QPhongMaterial>
#include <QPerVertexColorMaterial>


#include <thread>
#include <chrono>
#include <iostream>
#include <QCuboidMesh>
#include <QRandomGenerator>

#include "core/Timer.h"
#include "geometry/Tesselation.h"
#include "geometry/GeometryBuilder.h"
#include "geometry/Body.h"
#include "fileIO/MeshLoader.h"

static std::string s_filepath;
Qt3DRender::QMesh* s_mesh;
[[noreturn]] void spin(Qt3DCore::QTransform *transform);

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Root entity
    Qt3DCore::QEntity *root = new Qt3DCore::QEntity();

    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
    QWidget *container = QWidget::createWindowContainer(view);
    QSize screenSize = view->screen()->size();
    container->setMinimumSize(QSize(200, 100));
    container->setMaximumSize(screenSize);

    // Set root object of the scene
    view->setRootEntity(root);

    QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->setAlignment(Qt::AlignTop);
    hLayout->addWidget(container, 1);
    hLayout->addLayout(vLayout);

    widget->setWindowTitle(QStringLiteral("Auto Carver - Mesh Loading"));

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
    lightTransform->setTranslation(cameraEntity->position());
    lightEntity->addComponent(lightTransform);

    // Mesh
    auto *meshTransform = new Qt3DCore::QTransform();
    meshTransform->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));

    auto *meshEntity = new Qt3DCore::QEntity(root);
    s_mesh = new Qt3DRender::QMesh();
//    auto *cuboid = new Qt3DExtras::QCuboidMesh();

    if (argc > 2 && std::string(argv[2]) == "true") {
        s_filepath = argv[1];

        QObject::connect(s_mesh, &Qt3DRender::QMesh::statusChanged, view, [](Qt3DRender::QMesh::Status status) {
            std::cout << "Mesh Status: " << status << "\n";
            if (status != 3) return;

            ScopedTimer timer("Geometry reconstruction");

            Tesselation tessel = MeshLoader::loadAsTesselation(s_filepath);

            std::cout << "Tesselation: " << tessel.getVertexCount() << " vertices, " << tessel.getTriangleCount() << " triangles\n";
            Body body(tessel);
            tessel = body.tesselation();

            auto colors = std::vector<QVector3D>(tessel.getTriangleCount());
            for (auto & color : colors) {
                color = {QRandomGenerator::global()->bounded(100) / 100.0f
                        , QRandomGenerator::global()->bounded(100) / 100.0f
                        , QRandomGenerator::global()->bounded(100) / 100.0f
                };
            }

            auto geo = GeometryBuilder::convert(tessel, colors);

            geo->setParent(s_mesh);
            s_mesh->setGeometry(geo);
        });

        auto meshMaterial = new Qt3DExtras::QPerVertexColorMaterial();
        meshEntity->addComponent(meshMaterial);

    } else {
        s_mesh->setSource(QUrl::fromLocalFile(argv[1]));

        auto meshMaterial = new Qt3DExtras::QPhongMaterial();
        meshMaterial->setDiffuse(QColor(QRgb(0xa69929)));
    }

    meshEntity->addComponent(s_mesh);
    meshEntity->addComponent(meshTransform);

    auto thread = std::thread(spin, meshTransform);

    // Show window
    widget->show();
    widget->resize(1200, 800);

    return app.exec();
}

[[noreturn]] void spin(Qt3DCore::QTransform *transform)
{
    float theta = 0, phi = 0;

    while (true) {

        transform->setRotationX(theta);
        transform->setRotationY(phi);

        theta += M_PI;
        phi += M_PI / 16;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}


