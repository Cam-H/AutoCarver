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
#include "core/SculptProcess.h"
#include "robot/ArticulatedWrist.h"
#include "renderer/LineChartWidget.h"

#include "renderer/UiLoader.h"
#include "robot/planning/Trajectory.h"

#endif

std::shared_ptr<Scene> scene = nullptr;
SceneWidget *sceneWidget = nullptr;

std::shared_ptr<Body> plane;
std::shared_ptr<Body> base;

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
    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/convex-slice/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
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

    auto model = MeshBuilder::plane(10, glm::vec3(), glm::vec3(0, 1, 0));
    model->setBaseColor({1, 1, 1 });
    plane = scene->createBody(model);

    model = MeshBuilder::box(1, 1, 1);
    model->setBaseColor({1, 0, 1});
    base = scene->createBody(model);

    sceneWidget = widget->findChild<SceneWidget*>("sceneWidget");
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
        auto hull = base->hull().fragment(plane->position(), plane->up());
        base->setMesh(std::make_shared<Mesh>(hull), true);
        sceneWidget->update();
    });

    // Handle mesh randomization button
    auto *randomButton = widget->findChild<QPushButton*>("randomButton");
    QObject::connect(randomButton, &QPushButton::clicked, [&]() {
        base->setMesh(randomMesh(), true);
        sceneWidget->update();
    });

    widget->show();


    return app.exec();
}
