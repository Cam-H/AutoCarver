#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>
#include <QPushButton>
#include <QVBoxLayout>
#include <QSpinBox>

#include <QFile>
#include <QDir>

#ifndef QT_NO_OPENGL
#include "fileIO/MeshHandler.h"
#include "geometry/VertexArray.h"
#include "core/Scene.h"
#include "geometry/MeshBuilder.h"

#include "renderer/UiLoader.h"
#include "renderer/SceneWidget.h"
#include "geometry/primitives/Plane.h"
#include "geometry/collision/Collision.h"

#endif


QWidget *window = nullptr;

std::shared_ptr<Scene> scene = nullptr;
SceneWidget *sceneWidget = nullptr;

std::shared_ptr<Mesh> sphere = nullptr;

std::shared_ptr<Body> body = nullptr;

VertexArray cloud("../res/cloud.bin");
HullBuilder builder(cloud.vertices());

Plane cutPlane({ 0.0236198, 0.796194, -0.0335051 }, { -0.168133, 0.956477, 0.2385 });

static QWidget *loadUiFile(QWidget *parent)
{
    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/convex-hull/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void updateMesh(const std::shared_ptr<Mesh>& mesh)
{
    body->setMesh(mesh);
    scene->update();
    sceneWidget->update();
}

void reset()
{
    builder = HullBuilder(cloud.vertices());
    builder.clean();

    builder.initialize();
}

void prepare()
{
//    cloud.print();

    reset();

    scene->clear();

    double span = cloud.span({ 1, 0, 0 });
    span = std::max(span, cloud.span({ 0, 1, 0 }));
    span = std::max(span, cloud.span({ 0, 0, 1 }));

    sphere = MeshBuilder::icosphere(0.01 * span, 3);

    for (const glm::dvec3& vertex : builder.getVertices()) {
//        if (cutPlane.isAbove(vertex)) {
//            auto point = scene->createVisual(sphere);
//            point->setPosition(vertex);
//        }

        auto point = scene->createVisual(sphere);
        point->setPosition(vertex);
    }

//    hull.print();
//    auto intersection = Collision::intersection(hull, cutPlane);
//    std::cout << "Intersections: " << intersection.size() << "\n";
//    int idx = 0;
//    for (const glm::dvec3& vertex : intersection) {
//        std::cout << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
//        auto point = scene->createVisual(sphere);
//        point->setPosition(vertex);
//        point->scale(1.0 + 1.0 * idx++ / intersection.size());
//    }

    body = scene->createVisual(nullptr);
    updateMesh(std::make_shared<Mesh>(builder.getHull()));

//    scene->createVisual(MeshBuilder::plane(cutPlane, 1.0));
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    std::cout << "Cloud: " << cloud.vertexCount() << "\n";

    auto resetButton = window->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        reset();
        updateMesh(std::make_shared<Mesh>(builder.getHull()));
    });

//    auto invertButton = window->findChild<QPushButton*>("invertButton");
//    QObject::connect(invertButton, &QPushButton::clicked, [&]() {
//        cutPlane.invert();
//    });

    auto nextButton = window->findChild<QPushButton*>("nextButton");
    QObject::connect(nextButton, &QPushButton::clicked, [&]() {
//        auto fragments = Collision::fragments(hull, cutPlane);
//        updateMesh(std::make_shared<Mesh>(fragments.second));
        builder.step();
        updateMesh(std::make_shared<Mesh>(builder.getHull()));

        std::cout << "Iteration: " << builder.getIteration() << ", complete: " << builder.finished() << "\n";
    });

    auto completeButton = window->findChild<QPushButton*>("completeButton");
    QObject::connect(completeButton, &QPushButton::clicked, [&]() {
        builder.solve();
        updateMesh(std::make_shared<Mesh>(builder.getHull()));
    });

    auto *loadButton = window->findChild<QPushButton*>("loadButton");
    QObject::connect(loadButton, &QPushButton::clicked, [&]() {
        const QString fileName = QFileDialog::getOpenFileName(nullptr, "Select Model",
                                                     "../res/meshes", "Model Files (*.obj)");
        if (!fileName.isEmpty()) {
            cloud = VertexArray(fileName.toStdString());
            prepare();
        }

    });

    scene = std::make_shared<Scene>();

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(2, 0, 0));
    sceneWidget->setScene(scene);

    prepare();

    window->show();

    return app.exec();
}
