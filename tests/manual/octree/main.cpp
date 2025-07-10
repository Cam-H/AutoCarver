#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>
#include <QMainWindow>
#include <QPushButton>
#include <QRadioButton>
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

bool updateMesh = false;

std::array<float, 6> pos = { 0, 0, 0, 0, 0, 0 };
uint32_t type = 0;

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
        std::cout << count++ << "| " << i.index << " " << (uint32_t)i.status << "\n";

    // Create a cutting plane
    auto model = MeshBuilder::mesh(oct);
    if (model != nullptr) {
        model->setBaseColor({1, 1, 1 });
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
            switch (type) {
                case 0:
                {
                    static bool collision = false;
                    bool test = oct->collides(sphere) && updateMesh;
                    if (test != collision) {
                        auto mesh = MeshBuilder::mesh(oct);
                        if (test) mesh->setFaceColor({ 1, 0, 0});
                        else mesh->setFaceColor({ 1, 1, 1});
                        render->setMesh(mesh);
                        collision = test;
                    }
                }

                    break;
                case 1:
                    if (oct->unite(sphere) && updateMesh) render->setMesh(MeshBuilder::mesh(oct));
                    break;
                case 2:
                    if (oct->subtract(sphere) && updateMesh) render->setMesh(MeshBuilder::mesh(oct));
                    break;
                case 3:
                    if (oct->intersect(sphere) && updateMesh) render->setMesh(MeshBuilder::mesh(oct));
                    break;
                default: std::cout << "Unknown type: " << type << "\n";
            }

//            std::cout << oct->memoryFootprint() << "\n";
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

    auto *noneRButton = widget->findChild<QRadioButton*>("noneRButton");
    QObject::connect(noneRButton, &QRadioButton::pressed, [&]() {
        type = 0;
    });

    auto *uniteRButton = widget->findChild<QRadioButton*>("uniteRButton");
    QObject::connect(uniteRButton, &QRadioButton::pressed, [&]() {
        type = 1;
    });

    auto *differenceRButton = widget->findChild<QRadioButton*>("differenceRButton");
    QObject::connect(differenceRButton, &QRadioButton::pressed, [&]() {
        type = 2;
    });

    auto *intersectRButton = widget->findChild<QRadioButton*>("intersectRButton");
    QObject::connect(intersectRButton, &QRadioButton::pressed, [&]() {
        type = 3;
    });

    widget->show();

    scene->start();

    scene->connect(&sceneUpdate);
//    sceneWidget->start();

    return app.exec();
}
