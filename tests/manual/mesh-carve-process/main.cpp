#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>

#include <QFile>
#include <QDir>


#ifndef QT_NO_OPENGL
#include "renderer/UiLoader.h"
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "core/SculptProcess.h"
#include "renderer/RenderCapture.h"
#include "robot/ArticulatedWrist.h"
#include "core/Timer.h"

#endif

QWidget *window = nullptr;

std::unique_ptr<std::thread> updateThread;

std::shared_ptr<Robot> robot = nullptr;
std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget* sceneWidget = nullptr;

QPushButton *stepButton = nullptr, *skipButton = nullptr;

static QWidget *loadUiFile(QWidget *parent)
{
    QFile file("../tests/manual/mesh-carve-process/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
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

    std::string source = "../res/meshes/ogre.obj";
    auto model = MeshHandler::loadAsMeshBody(source);
    model->setBaseColor({1, 0, 1});


    scene = std::make_shared<SculptProcess>(model);
    scene->enableConvexTrim(false);
//    scene->setContinuous(true);

    robot = scene->createRobot(new ArticulatedWrist(0.2, 1.2, 1.2, 1));
    robot->translate({1, 0, 0});
    robot->update();

    scene->setSculptingRobot(robot);//

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
//    sceneWidget->camera().setPosition(QVector3D(15, 0, 0));
    sceneWidget->setScene(scene);


    auto sculptureButton = window->findChild<QCheckBox*>("sculptureButton");
    QObject::connect(sculptureButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->show(0, Scene::Model::ALL);
        else sceneWidget->hide(0, Scene::Model::ALL);
        sceneWidget->update();
    });

    auto hullButton = window->findChild<QCheckBox*>("hullButton");
    QObject::connect(hullButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->show(0, Scene::Model::HULL);
        else sceneWidget->hide(0, Scene::Model::HULL);
        sceneWidget->update();
    });

    stepButton = window->findChild<QPushButton*>("stepButton");
    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        stepButton->setEnabled(false);
        skipButton->setEnabled(false);
        scene->proceed();
        sceneWidget->update();
    });

    skipButton = window->findChild<QPushButton*>("skipButton");
    QObject::connect(skipButton, &QPushButton::clicked, [&]() {
        scene->skip();
        sceneWidget->update();
    });

    auto captureButton = window->findChild<QPushButton*>("captureButton");
    QObject::connect(captureButton, &QPushButton::clicked, [&]() {
//        m_rc->capture();
        if (sceneWidget != nullptr) {
            auto image = sceneWidget->grabFramebuffer();
//            auto img = new uint8_t[image.width()][image.height()];

            image.save(("../out/capture.png"));
        }
    });


    // Handle updating robot
    updateThread = std::make_unique<std::thread>([](){
        Timer rateTimer;
        bool idle = false;

        while (true) {
            scene->step((float)rateTimer.getElapsedSeconds());
            rateTimer.reset();

            if (scene->simulationActive() || (!idle && scene->simulationIdle())) {
                sceneWidget->update();
            } else {
                stepButton->setEnabled(true);
                skipButton->setEnabled(true);
            }

            idle = scene->simulationIdle();

            std::this_thread::sleep_for(std::chrono::nanoseconds(80000000));
        }
    });

    window->show();

    return app.exec();
}
