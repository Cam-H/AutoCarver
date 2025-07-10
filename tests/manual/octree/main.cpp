#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QTextEdit>
#include <QLineEdit>
#include <QSpinBox>
#include <QTableWidget>
#include <QGraphicsWidget>

#include <QFile>
#include <QDir>
#include <vector>

#ifndef QT_NO_OPENGL
#include "geometry/Sphere.h"
#include "geometry/ConvexHull.h"
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "geometry/Octree.h"
#include "core/Scene.h"
#include "robot/ArticulatedWrist.h"
#include "renderer/LineChartWidget.h"

#include "renderer/UiLoader.h"
#include "robot/planning/Trajectory.h"

#endif

std::shared_ptr<Scene> scene = nullptr;
SceneWidget *sceneWidget = nullptr;

std::shared_ptr<Octree> oct = nullptr;

std::shared_ptr<RigidBody> render = nullptr;
std::shared_ptr<RigidBody> collider = nullptr;

std::array<float, 6> pos = { 0, 0, 0, 0, 0, 0 };

#include <QRandomGenerator>

std::shared_ptr<Mesh> randomMesh()
{
    static QRandomGenerator rng;

    // Generate a cloud of count vertices within a cube around the origin
    uint32_t count = rng.global()->bounded(8, 12);
    std::vector<glm::vec3> cloud(count);
    auto *ptr = (float*)cloud.data();

    for (uint32_t j = 0; j < 3 * count; j++) *ptr++ = (float)rng.global()->bounded(2.0) - 1.0f;

    return std::make_shared<Mesh>(ConvexHull(cloud), false);
}

static QWidget *loadUiFile(QWidget *parent)
{
//    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/octree/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void sceneUpdate()
{
    sceneWidget->update();
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    auto *widget = loadUiFile(nullptr);
    if (widget == nullptr) return -1;

#ifndef QT_NO_OPENGL

    scene = std::make_shared<Scene>();

    oct = std::make_shared<Octree>(5);

    auto count = 0;
    for (auto i : *oct)
        std::cout << count++ << "| " << i << "\n";

    // Create a cutting plane
    auto model = MeshBuilder::mesh(oct);
    if (model != nullptr) {
        model->setBaseColor({1, 0.5f, 1 });
        render = scene->createBody(model);
    }

    auto colliderMesh = MeshBuilder::icosphere(1.0f, 3);
    collider = scene->createBody(colliderMesh);

    sceneWidget = widget->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(10, -1, 0));
    sceneWidget->setScene(scene);

    // Handle position field update
    auto *jiw = widget->findChild<QWidget*>("inputWidget");
    std::array<std::string, 6> names = { "xField", "yField", "zField", "rxField", "ryField", "rzField" };
    for (uint32_t i = 0; i < 6; i++) {
        auto *jointField = jiw->findChild<QDoubleSpinBox*>(names[i].c_str());
        QObject::connect(jointField, &QDoubleSpinBox::valueChanged, [i](float value) {
            pos[i] = value;

            const glm::vec3 position = { pos[0], pos[1], pos[2] };
            collider->setPosition(position);
            collider->setRotation({ pos[3], pos[4], pos[5] });

            Sphere sphere = Sphere(position, 1.0f);
            bool collision = oct->collides(sphere);
            if (collision) {
                oct->applyDifference(sphere);
//                render->setMesh(MeshBuilder::mesh(oct));
            }
            std::cout << "Collision: " << collision << "\n";
            sceneWidget->update();
        });

        pos[i] = (float)jointField->value();
    }

    collider->setPosition({ pos[0], pos[1], pos[2] });
    collider->setRotation({ pos[3], pos[4], pos[5] });

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    // Handle Octree reset button
    auto *resetButton = widget->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        oct->reset();

        if (render != nullptr) render->setMesh(MeshBuilder::mesh(oct));
    });

    // Handle mesh randomization button
    auto *randomButton = widget->findChild<QPushButton*>("randomButton");
    QObject::connect(randomButton, &QPushButton::clicked, [&]() {
        collider->setMesh(randomMesh(), true);
//        base->setMesh(MeshBuilder::box(6), true);

//        sceneWidget->update();
    });

    widget->show();

    scene->start();

    scene->connect(&sceneUpdate);
//    sceneWidget->start();

    return app.exec();
}
