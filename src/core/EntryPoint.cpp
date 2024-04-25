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

#include <Qt3DRender/qmesh.h>
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

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QPhongMaterial>

#include <Qt3DExtras/qt3dwindow.h>

#include "../renderer/MonitorCameraController.h"

#ifndef QT_NO_OPENGL
#include "../widgets/SceneViewWidget.h"
#endif

int main(int argc, char *argv[]) {

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

    // Scenemodifier
    {
        // Torus shape data
        //! [0]
        Qt3DExtras::QTorusMesh *m_torus = new Qt3DExtras::QTorusMesh();
        m_torus->setRadius(1.0f);
        m_torus->setMinorRadius(0.4f);
        m_torus->setRings(100);
        m_torus->setSlices(20);
        //! [0]

        // TorusMesh Transform
        //! [1]
        Qt3DCore::QTransform *torusTransform = new Qt3DCore::QTransform();
        torusTransform->setScale(2.0f);
        torusTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 1.0f, 0.0f), 25.0f));
        torusTransform->setTranslation(QVector3D(5.0f, 4.0f, 0.0f));
        //! [1]

        //! [2]
        Qt3DExtras::QPhongMaterial *torusMaterial = new Qt3DExtras::QPhongMaterial();
        torusMaterial->setDiffuse(QColor(QRgb(0xbeb32b)));
        //! [2]

        {
            // Torus
            //! [3]
            Qt3DCore::QEntity *m_torusEntity = new Qt3DCore::QEntity(rootEntity);
            m_torusEntity->addComponent(m_torus);
            m_torusEntity->addComponent(torusMaterial);
            m_torusEntity->addComponent(torusTransform);
            //! [3]
        }

        // Cone shape data
        Qt3DExtras::QConeMesh *cone = new Qt3DExtras::QConeMesh();
        cone->setTopRadius(0.5);
        cone->setBottomRadius(1);
        cone->setLength(3);
        cone->setRings(50);
        cone->setSlices(20);

        // ConeMesh Transform
        Qt3DCore::QTransform *coneTransform = new Qt3DCore::QTransform();
        coneTransform->setScale(1.5f);
        coneTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 45.0f));
        coneTransform->setTranslation(QVector3D(0.0f, 4.0f, -1.5));

        Qt3DExtras::QPhongMaterial *coneMaterial = new Qt3DExtras::QPhongMaterial();
        coneMaterial->setDiffuse(QColor(QRgb(0x928327)));

        // Cone
        {
            Qt3DCore::QEntity *m_coneEntity = new Qt3DCore::QEntity(rootEntity);
            m_coneEntity->addComponent(cone);
            m_coneEntity->addComponent(coneMaterial);
            m_coneEntity->addComponent(coneTransform);
        }

        // Cylinder shape data
        Qt3DExtras::QCylinderMesh *cylinder = new Qt3DExtras::QCylinderMesh();
        cylinder->setRadius(1);
        cylinder->setLength(3);
        cylinder->setRings(100);
        cylinder->setSlices(20);

        // CylinderMesh Transform
        Qt3DCore::QTransform *cylinderTransform = new Qt3DCore::QTransform();
        cylinderTransform->setScale(1.5f);
        cylinderTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 45.0f));
        cylinderTransform->setTranslation(QVector3D(-5.0f, 4.0f, -1.5));

        Qt3DExtras::QPhongMaterial *cylinderMaterial = new Qt3DExtras::QPhongMaterial();
        cylinderMaterial->setDiffuse(QColor(QRgb(0x928327)));

        // Cylinder
        {
            Qt3DCore::QEntity *m_cylinderEntity = new Qt3DCore::QEntity(rootEntity);
            m_cylinderEntity->addComponent(cylinder);
            m_cylinderEntity->addComponent(cylinderMaterial);
            m_cylinderEntity->addComponent(cylinderTransform);
        }

        // Cuboid shape data
        Qt3DExtras::QCuboidMesh *cuboid = new Qt3DExtras::QCuboidMesh();

        // CuboidMesh Transform
        Qt3DCore::QTransform *cuboidTransform = new Qt3DCore::QTransform();
        cuboidTransform->setScale(4.0f);
        cuboidTransform->setTranslation(QVector3D(5.0f, -4.0f, 0.0f));

        Qt3DExtras::QPhongMaterial *cuboidMaterial = new Qt3DExtras::QPhongMaterial();
        cuboidMaterial->setDiffuse(QColor(QRgb(0x665423)));

        //Cuboid
        {
            Qt3DCore::QEntity *m_cuboidEntity = new Qt3DCore::QEntity(rootEntity);
            m_cuboidEntity->addComponent(cuboid);
            m_cuboidEntity->addComponent(cuboidMaterial);
            m_cuboidEntity->addComponent(cuboidTransform);
        }

        // Plane shape data
        Qt3DExtras::QPlaneMesh *planeMesh = new Qt3DExtras::QPlaneMesh();
        planeMesh->setWidth(2);
        planeMesh->setHeight(2);

        // Plane mesh transform
        Qt3DCore::QTransform *planeTransform = new Qt3DCore::QTransform();
        planeTransform->setScale(1.3f);
        planeTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 45.0f));
        planeTransform->setTranslation(QVector3D(0.0f, -4.0f, 0.0f));

        Qt3DExtras::QPhongMaterial *planeMaterial = new Qt3DExtras::QPhongMaterial();
        planeMaterial->setDiffuse(QColor(QRgb(0xa69929)));

        // Plane
        {
            Qt3DCore::QEntity *m_planeEntity = new Qt3DCore::QEntity(rootEntity);
            m_planeEntity->addComponent(planeMesh);
            m_planeEntity->addComponent(planeMaterial);
            m_planeEntity->addComponent(planeTransform);
        }

        // Sphere shape data
        Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh();
        sphereMesh->setRings(20);
        sphereMesh->setSlices(20);
        sphereMesh->setRadius(2);

        // Sphere mesh transform
        Qt3DCore::QTransform *sphereTransform = new Qt3DCore::QTransform();

        sphereTransform->setScale(1.3f);
        sphereTransform->setTranslation(QVector3D(-5.0f, -4.0f, 0.0f));

        Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial();
        sphereMaterial->setDiffuse(QColor(QRgb(0xa69929)));

        // Sphere
        Qt3DCore::QEntity *m_sphereEntity = new Qt3DCore::QEntity(rootEntity);
        m_sphereEntity->addComponent(sphereMesh);
        m_sphereEntity->addComponent(sphereMaterial);
        m_sphereEntity->addComponent(sphereTransform);
    }

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
