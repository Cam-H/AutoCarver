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
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "core/Scene.h"
#include "robot/ArticulatedWrist.h"
#include "renderer/LineChartWidget.h"

#include "renderer/UiLoader.h"
#include "robot/planning/Trajectory.h"

#endif

std::shared_ptr<Scene> scene = nullptr;
SceneWidget *sceneWidget = nullptr;

std::shared_ptr<RigidBody> plane;
std::shared_ptr<RigidBody> base;

std::array<float, 6> pos = { 0, 0, 0, 0, 0, 0 };

#include <QRandomGenerator>

std::shared_ptr<Mesh> randomMesh()
{
    static QRandomGenerator rng;

    // Generate a cloud of count vertices within a cube around the origin
    uint32_t count = rng.global()->bounded(4, 12);
    auto *cloud = new float[3 * count], *ptr = cloud;
    for (uint32_t j = 0; j < 3 * count; j++) *ptr++ =  (float)rng.global()->bounded(2.0) - 1.0f;

    return std::make_shared<Mesh>(ConvexHull(cloud, count), false);
}

static QWidget *loadUiFile(QWidget *parent)
{
//    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/convex-slice/main.ui");
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

    // Create a cutting plane
    auto model = MeshBuilder::plane(10, glm::vec3(), glm::vec3(0, 1, 0));
    model->setBaseColor({1, 1, 1 });
    plane = scene->createBody(model);
    plane->setLayer(0);

    // Create the floor
    model = MeshBuilder::box(40, 40, 5);
    model->translate(0, -8, 0);
//    model->rotate(1, 0, 0, -M_PI / 16);
    model->setBaseColor({0.1, 0.4, 0.1});
    scene->createBody(model);

    // Create the base to cut from
    model = MeshBuilder::box(3, 3, 3);
    model->rotate(1, 0, 0, M_PI / 3);
//    model->translate(0.5, 0.5, 0.5);
    model->setBaseColor({1, 0, 1});
    base = scene->createBody(model);

    auto c = base->centroid();
    auto t = base->inertiaTensor();
//    std::cout << "Trait: " << base->volume() << "m3 " << base->mass() << "kg (" << c.x << ", " << c.y << ", " << c.z << ")\n";
//    std::cout << t[0][0] << " " << t[1][0] << " " << t[2][0] << "\n"
//              << t[0][1] << " " << t[1][1] << " " << t[2][1] << "\n"
//              << t[0][2] << " " << t[1][2] << " " << t[2][2] << "\n";

    sceneWidget = widget->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(15, -1, 0));
    sceneWidget->setScene(scene);

    // Handle position field update
    auto *jiw = widget->findChild<QWidget*>("jointInputWidget");
    std::array<std::string, 6> names = { "xField", "yField", "zField", "rxField", "ryField", "rzField" };
    for (uint32_t i = 0; i < 6; i++) {
        auto *jointField = jiw->findChild<QDoubleSpinBox*>(names[i].c_str());
        QObject::connect(jointField, &QDoubleSpinBox::valueChanged, [i](float value) {
            pos[i] = value;

            plane->setPosition({ pos[0], pos[1], pos[2] });
            plane->setRotation({ pos[3], pos[4], pos[5] });
            sceneWidget->update();
        });
    }

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    // Handle mesh slice button
    auto *sliceButton = widget->findChild<QPushButton*>("sliceButton");
    QObject::connect(sliceButton, &QPushButton::clicked, [&]() {
        auto fragments = base->hull().fragments(plane->position(), plane->up());
        base->setMesh(std::make_shared<Mesh>(fragments.first), true);

        if (!fragments.second.empty()) {
            auto debris = scene->createBody(fragments.second, RigidBody::Type::DYNAMIC);
//            debris->zero();
//            debris->setVelocity({0, -0.1, 0});
        }

//        sceneWidget->update();
    });

    // Handle mesh randomization button
    auto *randomButton = widget->findChild<QPushButton*>("randomButton");
    QObject::connect(randomButton, &QPushButton::clicked, [&]() {
        base->setMesh(randomMesh(), true);
//        sceneWidget->update();
    });

    widget->show();

    scene->start();

    scene->connect(&sceneUpdate);
//    sceneWidget->start();

    return app.exec();
}
