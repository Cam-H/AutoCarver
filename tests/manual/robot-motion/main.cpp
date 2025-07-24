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
#include <QFile>
#include <QDir>


#ifndef QT_NO_OPENGL
#include "renderer/UiLoader.h"
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "core/SculptProcess.h"
#include "robot/ArticulatedWrist.h"
#include "geometry/Axis3D.h"

#endif

QWidget *window = nullptr;

std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget *sceneWidget = nullptr;
std::shared_ptr<Robot> robot = nullptr;

std::vector<QSpinBox*> jointFields;
std::vector<QDoubleSpinBox*> posFields;

Axis3D axes;
double theta = M_PI / 64;

static QWidget *loadUiFile(QWidget *parent)
{
    QFile file("../tests/manual/robot-motion/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void updateJointFields()
{
    for (uint32_t j = 0; j < 6; j++) {
        jointFields[j]->blockSignals(true);
        jointFields[j]->setValue(robot->getJointValueDg(j));
        jointFields[j]->blockSignals(false);
    }
}

void updatePositionFields()
{
    axes = robot->getEOATAxes();

    auto position = robot->getEOATPosition();
    for (uint32_t j = 0; j < 3; j++) {
        posFields[j]->blockSignals(true);
        posFields[j]->setValue(position[j]);
        posFields[j]->blockSignals(false);
    }
}

void updateAxes()
{
    robot->moveTo(axes);
    sceneWidget->update();

    updateJointFields();
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    std::string source = R"(..\res\meshes\devil.obj)";
    auto model = MeshHandler::loadAsMeshBody(source);

    model->setBaseColor({1, 0, 1});


    scene = std::make_shared<SculptProcess>(model);
    robot = scene->createRobot(std::make_shared<ArticulatedWrist>(0.8, 2, 2, 1));
    robot->translate({ -2, 0, 0 });

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->setScene(scene);

    robot->setJointValueDg(1, 135);
    robot->setJointValueDg(2, -45);
    axes = robot->getEOATAxes();

    robot->update();
    sceneWidget->update();

    jointFields = {
            window->findChild<QSpinBox*>("j0Field"),
            window->findChild<QSpinBox*>("j1Field"),
            window->findChild<QSpinBox*>("j2Field"),
            window->findChild<QSpinBox*>("j3Field"),
            window->findChild<QSpinBox*>("j4Field"),
            window->findChild<QSpinBox*>("j5Field")
    };

    posFields = {
            window->findChild<QDoubleSpinBox*>("xField"),
            window->findChild<QDoubleSpinBox*>("yField"),
            window->findChild<QDoubleSpinBox*>("zField")
    };

    // Joint angle control
    for (uint8_t i = 0; i < 6; i++) {
        auto *field = jointFields[i];

        field->setValue(std::round(robot->getJointValueDg(i)));
        QObject::connect(field, &QSpinBox::valueChanged, [field, i](int value) {
            robot->setJointValueDg(i, value);
            robot->update();
            sceneWidget->update();

            field->setValue(std::round(robot->getJointValueDg(i)));

            updatePositionFields();
        });
    }


    // EOAT position
    for (uint8_t i = 0; i < 3; i++) {
        auto *field = posFields[i];
        auto position = robot->getEOATPosition();
        field->setValue(position[i]);

        QObject::connect(field, &QDoubleSpinBox::valueChanged, [field, i](double value) {
            glm::dvec3 position = robot->getEOATPosition();
            position[i] = value;

            robot->moveTo(position);
            sceneWidget->update();

            field->blockSignals(true);
            field->setValue(robot->getEOATPosition()[i]);
            field->blockSignals(false);

            // Update joint fields to match
            updateJointFields();
        });
    }

    auto decXButton = window->findChild<QPushButton*>("decXButton");
    QObject::connect(decXButton, &QPushButton::clicked, [&]() {
        axes.rotateX(-theta);
        updateAxes();
    });

    auto incXButton = window->findChild<QPushButton*>("incXButton");
    QObject::connect(incXButton, &QPushButton::clicked, [&]() {
        axes.rotateX(theta);
        updateAxes();
    });

    auto decYButton = window->findChild<QPushButton*>("decYButton");
    QObject::connect(decYButton, &QPushButton::clicked, [&]() {
        axes.rotateY(-theta);
        updateAxes();
    });

    auto incYButton = window->findChild<QPushButton*>("incYButton");
    QObject::connect(incYButton, &QPushButton::clicked, [&]() {
        axes.rotateY(theta);
        updateAxes();
    });

    auto decZButton = window->findChild<QPushButton*>("decZButton");
    QObject::connect(decZButton, &QPushButton::clicked, [&]() {
        axes.rotateZ(-theta);
        updateAxes();
    });

    auto incZButton = window->findChild<QPushButton*>("incZButton");
    QObject::connect(incZButton, &QPushButton::clicked, [&]() {
        axes.rotateZ(theta);
        updateAxes();
    });


    auto showMeshButton = window->findChild<QCheckBox*>("showMeshButton");
    QObject::connect(showMeshButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::MESH);
        else sceneWidget->hideAll(Scene::Model::MESH);
    });

    auto showHullButton = window->findChild<QCheckBox*>("showHullButton");
    QObject::connect(showHullButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::HULL);
        else sceneWidget->hideAll(Scene::Model::HULL);
    });

    auto showSphereButton = window->findChild<QCheckBox*>("showSphereButton");
    QObject::connect(showSphereButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::BOUNDING_SPHERE);
        else sceneWidget->hideAll(Scene::Model::BOUNDING_SPHERE);
    });



    window->show();

    return app.exec();
}
