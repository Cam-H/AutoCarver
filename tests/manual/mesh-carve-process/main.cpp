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

#include "mcut/mcut.h"
//#include "mio/mio.h"

#include "core/Timer.h"
#include "core/Scene.h"
#include "geometry/Tesselation.h"
#include "geometry/GeometryBuilder.h"
#include "geometry/Body.h"
#include "fileIO/MeshLoader.h"
#include "geometry/Mesh.h"
#include "geometry/MeshBuilder.h"

#include <reactphysics3d/reactphysics3d.h>

//root;
static std::string s_filepath;

static Scene s_scene;
Body* m_floor = nullptr;
static Body* s_body = nullptr;

//static Mesh s_body(nullptr, 0, nullptr, 0);
//static rp3d::RigidBody *s_physBody;
//Qt3DRender::QMesh* s_mesh;
[[noreturn]] void update();

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Root entity
    auto root = new Qt3DCore::QEntity();

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

//    auto *cuboid = new Qt3DExtras::QCuboidMesh();

    if (argc > 2 && std::string(argv[2]) == "true") {
        s_filepath = argv[1];

//        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~\n";
//        Mesh mesh = MeshBuilder::box(10, 5, 1);
//        for (uint32_t i = 0; i < mesh.vertexCount(); i++) {
//            std::cout << i << ": " << mesh.vertices()[3 * i] << " " << mesh.vertices()[3 * i + 1] << " " << mesh.vertices()[3 * i + 2] << "\n";
//        }
//        for (uint32_t i = 0; i < mesh.triangleCount(); i++) {
//            std::cout << i << ": " << mesh.indices()[3 * i] << " " << mesh.indices()[3 * i + 1] << " " << mesh.indices()[3 * i + 2] << "\n";
//        }

//        s_world = s_physicsCommon.createPhysicsWorld();

        // Add floor
        rp3d::Vector3 position(0, -3, 0);
        rp3d::Quaternion orientation = rp3d::Quaternion::identity();
        orientation = rp3d::Quaternion::fromEulerAngles(0, 0, M_PI / 16);
        rp3d::Transform transform(position, orientation);
//        rp3d::RigidBody* floorBody = s_world->createRigidBody(transform);
//        floorBody->addCollider(s_physicsCommon.createBoxShape(rp3d::Vector3(100, 0.2f, 100)), rp3d::Transform::identity());
//        floorBody->setType(rp3d::BodyType::STATIC);

//        transform = rp3d::Transform::identity();
//        s_physBody = s_world->createRigidBody(transform);
//        s_body.generateColliders(s_physicsCommon, s_physBody);

//        s_physBody = s_world->createRigidBody(rp3d::Transform::identity());
//        rp3d::Collider *collider = s_physBody->addCollider(s_physicsCommon.createBoxShape(rp3d::Vector3(0.5f, 0.5f, 0.5f)), rp3d::Transform::identity());
//        collider->getMaterial().setBounciness(0.01f);
//        floorBody->setType(rp3d::BodyType::STATIC);


        m_floor = s_scene.createBody(MeshBuilder::box(18, 6, 1));
        m_floor->translate(0, -5, 0);
        m_floor->rotate(M_PI / 32, 0, 0, 1);
        m_floor->rotate(M_PI / 128, 1, 0, 0);

//        m_floor->mesh().generate(root, view);

//        s_body = s_scene.createBody(MeshBuilder::box(2, 2, 2), rp3d::BodyType::DYNAMIC);

        s_body = s_scene.createBody(s_filepath, rp3d::BodyType::DYNAMIC);

//        s_body->mesh().setBaseColor(QColor::fromRgbF(1, 0, 0));
//        for (uint32_t i = 0; i < s_body->mesh().triangleCount(); i += 2) {
//            s_body->mesh().setFaceColor(i, QColor::fromRgbF(0, 0, 1));
//        }
//        s_body->mesh().generate(root, view);

//        QObject::connect(s_mesh, &Qt3DRender::QMesh::statusChanged, view, [](Qt3DRender::QMesh::Status status) {
//            std::cout << "Mesh Status: " << status << "\n";
//            if (status != 3) return;
//
//            s_scene.createBody(MeshBuilder::box(10, 5, 1));
//            s_body = s_scene.createBody(s_filepath, rp3d::BodyType::DYNAMIC);
//
//        });



    } else {
//        s_mesh->setSource(QUrl::fromLocalFile(argv[1]));

        auto meshMaterial = new Qt3DExtras::QPhongMaterial();
        meshMaterial->setDiffuse(QColor(QRgb(0xa69929)));
    }


    auto thread = std::thread(update);

    // Show window
    widget->show();
    widget->resize(1200, 800);

    return app.exec();
}

[[noreturn]] void update()
{
    float theta = 0, phi = 0;
//    const decimal timeStep = 1.0f / 60.0f;

    while (true) {

//        transform->setRotationX(theta);
//        transform->setRotationY(phi);
//
//        theta += M_PI;
//        phi += M_PI / 16;

//        if (m_floor != nullptr) m_floor->translate(0, 0.2f, 0);

        // Get the updated position of the body
        if (s_body != nullptr) {
            s_scene.update(20 / 1000.0f);

            // Display the position of the body
//            std::cout << "Body Position: (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}


