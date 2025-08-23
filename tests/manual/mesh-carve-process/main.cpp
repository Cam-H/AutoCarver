#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>
#include <QMainWindow>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QSpinBox>
#include <QComboBox>

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
std::shared_ptr<RigidBody> eoat = nullptr;
SceneWidget* sceneWidget = nullptr;

QCheckBox *sculptureButton = nullptr, *modelButton = nullptr;

QCheckBox *contButton = nullptr, *releaseButton = nullptr;
QDoubleSpinBox *timeField = nullptr;
QPushButton *stepButton = nullptr, *skipButton = nullptr;

QSpinBox* sliceLimitField = nullptr;

static QWidget *loadUiFile(QWidget *parent)
{
    QFile file("../tests/manual/mesh-carve-process/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void setOrder(int index) {
    switch (index) {
        case 0: scene->setSlicingOrder(SculptProcess::ConvexSliceOrder::TOP_DOWN); break;
        case 1: scene->setSlicingOrder(SculptProcess::ConvexSliceOrder::BOTTOM_UP); break;
        default: std::cout << "Unhandled order\n";
    }
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

    std::string source = "../res/meshes/devil.obj";
//    std::string source = "../res/meshes/teddy.obj";
//    std::string source = "../res/meshes/bunny.obj";
    auto model = MeshHandler::loadAsMeshBody(source);
    model->setBaseColor({1, 0, 1});

    scene = std::make_shared<SculptProcess>(model);
//    scene->enableConvexTrim(false);
//    scene->setContinuous(true);

    robot = scene->createRobot(std::make_shared<ArticulatedWrist>(0.2, 1.2, 1.2, 0.35));
    robot->setJointValueDg(1, 110);
    robot->setJointValueDg(2, 20);
    robot->setJointValueDg(4, -130);
    robot->translate({ 2, 1, 0 });
    robot->rotate({ 0, 1, 0 }, M_PI);
    robot->setLinkMesh(0, MeshHandler::loadAsMeshBody(R"(..\res\meshes\RobotBase.obj)"));

    robot->setLinkMesh(6, MeshHandler::loadAsMeshBody(R"(..\res\meshes\BladeAttachment.obj)"));

    auto eoatMesh = MeshHandler::loadAsMeshBody("../res/meshes/Blade.obj");
    eoat = scene->createBody(eoatMesh);
    eoat->setName("BLADE");
    eoat->prepareColliderVisuals();
    robot->setEOAT(eoat, false);

    scene->setSculptingRobot(robot);//
    robot->update();

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
//    sceneWidget->camera().setPosition(QVector3D(15, 0, 0));
    sceneWidget->setScene(scene);


    sculptureButton = window->findChild<QCheckBox*>("sculptureButton");
    QObject::connect(sculptureButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->show(scene->getSculpture()->getID(), Scene::Model::MESH);
        else sceneWidget->hide(scene->getSculpture()->getID(), Scene::Model::MESH);
        sceneWidget->update();
    });

    modelButton = window->findChild<QCheckBox*>("modelButton");
    sceneWidget->setVisibility(modelButton->isChecked(), scene->getModel()->getID(), Scene::Model::MESH);
    QObject::connect(modelButton, &QCheckBox::clicked, [&](bool checked) {
        sceneWidget->setVisibility(checked, scene->getModel()->getID(), Scene::Model::MESH);
        sceneWidget->update();
    });

    auto decompButton = window->findChild<QCheckBox*>("decompButton");
    scene->getSculpture()->applyCompositeColors(decompButton->isChecked());
    QObject::connect(decompButton, &QCheckBox::clicked, [&](bool checked) {
        scene->getSculpture()->applyCompositeColors(checked);
        sceneWidget->updateRenderGeometry(scene->getSculpture()->mesh());
        sceneWidget->update();
    });

    auto hullButton = window->findChild<QCheckBox*>("hullButton");
    QObject::connect(hullButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::HULL);
        else sceneWidget->hideAll(Scene::Model::HULL);
        sceneWidget->update();
    });

    auto axesButton = window->findChild<QCheckBox*>("axesButton");
    sceneWidget->setVisibility(axesButton->isChecked(), eoat->getID(), Scene::Model::AXES);
    QObject::connect(axesButton, &QCheckBox::clicked, [&](bool checked) {
        sceneWidget->setVisibility(checked, eoat->getID(), Scene::Model::AXES);
        sceneWidget->update();
    });

    auto collisionButton = window->findChild<QCheckBox*>("collisionButton");
    scene->enableCollisionColoring(collisionButton->isChecked());
    QObject::connect(collisionButton, &QCheckBox::clicked, [&](bool checked) {
        scene->enableCollisionColoring(checked);
    });

    contButton = window->findChild<QCheckBox*>("contButton");
    QObject::connect(contButton, &QCheckBox::clicked, [&](bool checked) {
        scene->setContinuous(checked);
    });

    timeField = window->findChild<QDoubleSpinBox*>("timeField");
    QObject::connect(timeField, &QDoubleSpinBox::valueChanged, [&](double value) {
        scene->setTimeScaling(value);
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

    auto newModelButton = window->findChild<QPushButton*>("newModelButton");
    QObject::connect(newModelButton, &QPushButton::clicked, [&]() {
        const QString fileName = QFileDialog::getOpenFileName(nullptr, "Select Model",
                                                              "../res/meshes", "Model Files (*.obj)");

        if (!fileName.isEmpty()) {

            auto mesh = MeshHandler::loadAsMeshBody(fileName.toStdString());
            if (mesh != nullptr) {
                scene->setTarget(mesh);
                sculptureButton->setChecked(true);
                modelButton->setChecked(false);

                sceneWidget->clear();
            } else std::cout << "Failed to load model. Can not set target\n";
        }
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

    auto resetButton = window->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        scene->reset();
        sceneWidget->update();
    });

    auto linkButton = window->findChild<QCheckBox*>("linkButton");
    scene->enableActionLinking(linkButton->isChecked());
    QObject::connect(linkButton, &QCheckBox::clicked, [&](bool checked) {
        scene->enableActionLinking(checked);
    });

    auto testCollisionButton = window->findChild<QCheckBox*>("testCollisionButton");
    scene->enableCollisionTesting(testCollisionButton->isChecked());
    QObject::connect(testCollisionButton, &QCheckBox::clicked, [&](bool checked) {
        scene->enableCollisionTesting(checked);
    });

    auto mergeButton = window->findChild<QCheckBox*>("mergeButton");
    scene->getSculpture()->enableHullMerging(testCollisionButton->isChecked());
    QObject::connect(mergeButton, &QCheckBox::clicked, [&](bool checked) {
        scene->getSculpture()->enableHullMerging(checked);
    });

    releaseButton = window->findChild<QCheckBox*>("releaseButton");
    scene->enableFragmentRelease(releaseButton->isChecked());
    QObject::connect(releaseButton, &QCheckBox::clicked, [&](bool checked) {
        scene->enableFragmentRelease(checked);
    });

    auto simulateCutButton = window->findChild<QCheckBox*>("simulateCutButton");
    scene->enableCutSimulation(simulateCutButton->isChecked());
    releaseButton->setEnabled(simulateCutButton->isChecked());
    QObject::connect(simulateCutButton, &QCheckBox::clicked, [&](bool checked) {
        scene->enableCutSimulation(checked);
        releaseButton->setEnabled(checked);
    });

    auto convexTrimButton = window->findChild<QCheckBox*>("convexTrimButton");
    scene->enableConvexTrim(convexTrimButton->isChecked());
    QObject::connect(convexTrimButton, &QCheckBox::clicked, [&](bool checked) {
        scene->enableConvexTrim(checked);
    });

    auto outlineRefinementButton = window->findChild<QCheckBox*>("outlineRefinementButton");
    scene->enableSilhouetteRefinement(outlineRefinementButton->isChecked());
    QObject::connect(outlineRefinementButton, &QCheckBox::clicked, [&](bool checked) {
        scene->enableSilhouetteRefinement(checked);
    });

    auto sliceOrderBox = window->findChild<QComboBox*>("sliceOrderBox");
    setOrder(sliceOrderBox->currentIndex());
    QObject::connect(sliceOrderBox, &QComboBox::activated, [&](int index) {
        setOrder(index);
    });

    sliceLimitField = window->findChild<QSpinBox*>("sliceLimitField");
    scene->setActionLimit(sliceLimitField->value());
    std::cout << "SLF: " << sliceLimitField->value() << "\n";
    QObject::connect(sliceLimitField, &QSpinBox::valueChanged, [&](int value) {
        scene->setActionLimit(value);
    });

    auto sliceLimitButton = window->findChild<QCheckBox*>("sliceLimitButton");
    scene->enableActionLimit(sliceLimitButton->isChecked());
    QObject::connect(sliceLimitButton, &QCheckBox::clicked, [&](bool checked) {
        sliceLimitField->setEnabled(checked);
        scene->enableActionLimit(checked);
    });


    scene->start();

    // Handle updating robot
    updateThread = std::make_unique<std::thread>([](){
        bool idle = false, complete = false;

        while (true) {
            if (scene->simulationActive() || (!idle && scene->simulationIdle()) || (!complete && scene->simulationComplete())) {
                sceneWidget->update();
//                std::cout << "Collision: " << scene->test(scene->getTurntable()->getEOAT()) << "\n";

                complete = scene->simulationComplete();
            } else {
                stepButton->setEnabled(true);
                skipButton->setEnabled(true);
            }

            idle = scene->simulationIdle();

            std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));
        }
    });

    window->show();

    return app.exec();
}
