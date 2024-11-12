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

#ifndef QT_NO_OPENGL
#include "../widgets/SceneViewWidget.h"
#endif

#include "mcut/mcut.h"
#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <string>

#define my_assert(cond)                                                                            \
	if(!(cond))                                                                                    \
	{                                                                                              \
		fprintf(stderr, "MCUT error: %s\n", #cond);                                                \
		std::abort();                                                                              \
	}

int main(int argc, char *argv[]) {
    std::cout << "See manual tests. \n";

    McFloat srcMeshVertices[] = {
            -5, -5, 5,  // vertex 0
            5, -5, 5,   // vertex 1
            5, 5, 5,    // vertex 2
            -5, 5, 5,   // vertex 3
            -5, -5, -5, // vertex 4
            5, -5, -5,  // vertex 5
            5, 5, -5,   // vertex 6
            -5, 5, -5   // vertex 7
    };

    McUint32 srcMeshFaces[] = {
            0, 1, 2, 3, // face 0
            7, 6, 5, 4, // face 1
            1, 5, 6, 2, // face 2
            0, 3, 7, 4, // face 3
            3, 2, 6, 7, // face 4
            4, 5, 1, 0  // face 5
    };

    McUint32 srcMeshFaceSizes[] = { 4, 4, 4, 4, 4, 4};

    McUint32 srcMeshVertexCount = 8;
    McUint32 srcMeshFaceCount = 6;

    // the cut mesh (a quad formed of two triangles)

    McFloat cutMeshVertices[] = {
            -20, -4, 0, // vertex 0
            0, 20, 20,  // vertex 1
            20, -4, 0,  // vertex 2
            0, 20, -20  // vertex 3
    };

    McUint32 cutMeshFaces[] = {
            0, 1, 2, // face 0
            0, 2, 3  // face 1
    };

    // McUint32 cutMeshFaceSizes[] = { 3, 3};

    McUint32 cutMeshVertexCount = 4;
    McUint32 cutMeshFaceCount = 2;

    // Create the context
    McContext context = MC_NULL_HANDLE;
    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);
    my_assert (status == MC_NO_ERROR);

    // Do cut
    status = mcDispatch(
            context,
            MC_DISPATCH_VERTEX_ARRAY_FLOAT,
            srcMeshVertices,
            srcMeshFaces,
            srcMeshFaceSizes,
            srcMeshVertexCount,
            srcMeshFaceCount,
            cutMeshVertices,
            cutMeshFaces,
            nullptr, // cutMeshFaceSizes, // no need to give 'cutMeshFaceSizes' parameter since the cut-mesh is a triangle mesh
            cutMeshVertexCount,
            cutMeshFaceCount);
    my_assert (status == MC_NO_ERROR);

    std::cout << "Dispatch complete\n";

    // Query the number of available connected components after the cut
    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);
    my_assert (status == MC_NO_ERROR);

    if (connectedComponentCount == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(EXIT_FAILURE);
    }

    connectedComponents.resize(connectedComponentCount); // allocate for the amount we want to get

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);
    my_assert (status == MC_NO_ERROR);

    //  Query the data of each connected component
    for (McInt32 i = 0; i < (McInt32)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i];
        McSize numBytes = 0;

        // Vertices

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &numBytes);
        my_assert (status == MC_NO_ERROR);

        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McFloat) * 3ull));
        std::vector<McFloat> ccVertices(ccVertexCount * 3u);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, numBytes, (McVoid*)ccVertices.data(), NULL);
        my_assert (status == MC_NO_ERROR);

        // Faces

        numBytes = 0;

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceIndices;
        ccFaceIndices.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, (McVoid*)ccFaceIndices.data(), NULL);
        my_assert (status == MC_NO_ERROR);

        // Face sizes (vertices per face)

        numBytes = 0;

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceSizes;
        ccFaceSizes.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, (McVoid*)ccFaceSizes.data(), NULL);
        my_assert (status == MC_NO_ERROR);

        std::cout << "Connected component: " << ccVertexCount << " " << ccFaceSizes.size() << "\n";
    }

    std::cout << "Preparing to free memory\n";

//     Free individual component memory
    status = mcReleaseConnectedComponents(context, 0, NULL);
    my_assert (status == MC_NO_ERROR);

    std::cout << "Individual components freed\n";

    // Free context memory
    status = mcReleaseContext(context);
    my_assert (status == MC_NO_ERROR);

    std::cout << "?\n";

    return 0;

//    std::cout << argc << " " << argv[0] << "\n";
//    QApplication app(argc, argv);
//
//    // Root entity
//    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();
////    NotScene *scene = new NotScene(rootEntity, argv[1]);
//
////    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
//    scene->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
//    QWidget *container = QWidget::createWindowContainer(scene);
//    QSize screenSize = scene->screen()->size();
//    container->setMinimumSize(QSize(200, 100));
//    container->setMaximumSize(screenSize);
//
//    QWidget *widget = new QWidget;
//    QHBoxLayout *hLayout = new QHBoxLayout(widget);
//    QVBoxLayout *vLayout = new QVBoxLayout();
//    vLayout->setAlignment(Qt::AlignTop);
//    hLayout->addWidget(container, 1);
//    hLayout->addLayout(vLayout);
//
//    widget->setWindowTitle(QStringLiteral("Auto Carver"));
//
//    // Camera
//    Qt3DRender::QCamera *cameraEntity = scene->camera();
//
//    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
//    cameraEntity->setPosition(QVector3D(0, 0, 20.0f));
//    cameraEntity->setUpVector(QVector3D(0, 1, 0));
//    cameraEntity->setViewCenter(QVector3D(0, 0, 0));
//
//
//    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
//    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
//    light->setColor("white");
//    light->setIntensity(1);
//    lightEntity->addComponent(light);
//    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
//    lightTransform->setTranslation(cameraEntity->position());
//    lightEntity->addComponent(lightTransform);
//
//    // For camera controls
//    MonitorCameraController *camController = new MonitorCameraController(rootEntity);
//    camController->setCamera(cameraEntity);
//    camController->linkContainer(container);
//
//    // Set root object of the scene
//    scene->setRootEntity(rootEntity);
//
//    // Create control widgets
//
//    QSlider *rxSlider = new QSlider(Qt::Orientation::Horizontal, widget);
//    rxSlider->setMinimumWidth(200);
//    rxSlider->setMinimum(-100);
//    QSlider *rySlider = new QSlider(Qt::Orientation::Horizontal, widget);
//    rySlider->setMinimum(-100);
//    QSlider *rzSlider = new QSlider(Qt::Orientation::Horizontal, widget);
//    rzSlider->setMinimum(-100);
//
//    QSlider *cutSlider = new QSlider(Qt::Orientation::Horizontal, widget);
//    cutSlider->setMinimum(-40);
//    cutSlider->setMaximum(100);
//
//    QObject::connect(rxSlider, &QSlider::valueChanged, scene, &NotScene::apexX);
//    QObject::connect(rySlider, &QSlider::valueChanged, scene, &NotScene::apexY);
//    QObject::connect(rzSlider, &QSlider::valueChanged, scene, &NotScene::apexZ);
//    QObject::connect(cutSlider, &QSlider::valueChanged, scene, &NotScene::cut);
//
//
//    QCheckBox *cutPlaneShowCB = new QCheckBox(widget);
//    cutPlaneShowCB->setChecked(false);
//    cutPlaneShowCB->setText(QStringLiteral("Show cut plane"));
//    QObject::connect(cutPlaneShowCB, &QCheckBox::stateChanged, scene, &NotScene::show);
//
//    vLayout->addWidget(rxSlider);
//    vLayout->addWidget(rySlider);
//    vLayout->addWidget(rzSlider);
//    vLayout->addWidget(rzSlider);
//
//    vLayout->addWidget(cutSlider);
//
//    vLayout->addWidget(cutPlaneShowCB);
//
//    // Show window
//    widget->show();
//    widget->resize(1200, 800);
//
//    return app.exec();
}
