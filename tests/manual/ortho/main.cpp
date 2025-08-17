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
#include "core/Scene.h"
#include "core/Sculpture.h"
#include "physics/RigidBody.h"
#include "geometry/MeshBuilder.h"
#include "robot/ArticulatedWrist.h"

#include "renderer/UiLoader.h"
#include "renderer/RenderCapture.h"
#include "renderer/EdgeDetect.h"
#include "renderer/SceneWidget.h"
#include "renderer/PolygonWidget.h"

#endif


QWidget *window = nullptr;

QLabel *modelView = nullptr;
QLabel *processView = nullptr;

Profile profile;
PolygonWidget* polygonWidget = nullptr;

std::shared_ptr<Scene> scene = nullptr;
SceneWidget *sceneWidget = nullptr;

QCheckBox *showModelSetting = nullptr;
QCheckBox *showSculptureSetting = nullptr;
QCheckBox *showHullSetting = nullptr;
QPushButton *stepButton = nullptr;

std::shared_ptr<Sculpture> body = nullptr;
std::shared_ptr<RigidBody> silhouette = nullptr;

std::shared_ptr<Mesh> mesh = nullptr;
//std::shared_ptr<Mesh> hull = nullptr;

EdgeDetect* detector = nullptr;

QPixmap* px;

static QWidget *loadUiFile(QWidget *parent)
{
    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/ortho/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void updateImage()
{
    glm::dvec3 offset = -mesh->boundedOffset();
    double scalar = body->scalar();

    std::cout << scalar << " " << offset.x << " " << offset.y << " " << offset.z << "!\n";

    detector->update();

    profile = detector->profile();
    profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);
//    profile.rotateAbout({ 0, 1, 0 }, -body->rotation());
//        profile.centerScale(scalar);
//    profile.scale(scalar);
//    profile.translate(offset);

//    auto sc = profile.spanCenter();
//    profile.translate(-sc);
//    profile.translate(sc + glm::vec2{0.01f, 0});


//    auto extrude = MeshBuilder::extrude(profile.projected3D(), profile.normal(), 4);
//
//    if (extrude != nullptr) {
//        extrude->setBaseColor({1, 0, 1});
////        extrude->translate(-2.0f * profile.normal() + mesh->boundedOffset());
//        MeshHandler::exportMesh(extrude, "border.obj");
//
//        if (silhouette == nullptr) silhouette = scene->createBody(extrude);
//        else silhouette->setMesh(extrude, false);
//    }

    polygonWidget->setPolygon(&profile);
    polygonWidget->center();

    detector->sink().save(QString("image.png"));

    px = new QPixmap;
    px->convertFromImage(detector->source());
    modelView->setPixmap(*px);

    px = new QPixmap;
    px->convertFromImage(detector->sink());
    processView->setPixmap(*px);

    sceneWidget->update();
}

void updateBodyVisibility()
{
    if (showSculptureSetting->isChecked()) sceneWidget->show(0, Scene::Model::MESH);
    else sceneWidget->hide(0);

    sceneWidget->update();
}

void updateModelVisibility()
{
    if (showModelSetting->isChecked()) sceneWidget->show(1, Scene::Model::MESH);
    else sceneWidget->hide(1);

    sceneWidget->update();
}

void updateColoring()
{
    body->applyCompositeColors(showHullSetting->isChecked());

    sceneWidget->updateRenderGeometry(body->mesh());
    sceneWidget->update();
}

void refine()
{
    bool external = profile.isNextExternal();
    auto indices = profile.refine();

    std::cout << "RSTATE: " << external << " " << profile.complete() << " " << profile.vertexCount() << "\n";
    auto angles = profile.angles(indices), clearance = profile.clearance(indices);
    std::cout << "Angles: [" << (180/M_PI)*angles.first << " " << (180/M_PI)*angles.second << "], Clearance: ["
    << " " << clearance.first << " " << clearance.second << "]\n";

    auto border = profile.projected3D( std::vector<uint32_t>{ indices.I0, indices.I1, indices.I2 });

    if (!border.empty()) {
        body->queueSection(border[0], border[1], border[2], profile.normal(), external);
        body->applySection();

//        auto extrude = MeshBuilder::extrude(border, profile.normal(), 1);
//        extrude->translate(0.5f * -profile.normal());
//        extrude->setFaceColor({ 0, 1, 1 });
//        scene->createBody(extrude);
    }
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    modelView = window->findChild<QLabel*>("modelView");
    processView = window->findChild<QLabel*>("processView");
    polygonWidget = window->findChild<PolygonWidget*>("polygonWidget");


    showModelSetting = window->findChild<QCheckBox*>("showModelSetting");
    QObject::connect(showModelSetting, &QCheckBox::clicked, [&]() {
        updateModelVisibility();
        sceneWidget->update();
    });

    showSculptureSetting = window->findChild<QCheckBox*>("showSculptureSetting");
    QObject::connect(showSculptureSetting, &QCheckBox::clicked, [&]() {
        if (!showSculptureSetting->isChecked()) {
            showHullSetting->setCheckState(Qt::CheckState::Unchecked);
        }

        updateBodyVisibility();
    });

    showHullSetting = window->findChild<QCheckBox*>("showHullSetting");
    QObject::connect(showHullSetting, &QCheckBox::clicked, [&]() {
        if (showHullSetting->isChecked()) {
            showSculptureSetting->setCheckState(Qt::CheckState::Checked);
            updateBodyVisibility();
        }

        updateColoring();
    });

    auto resetButton = window->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        body->restoreAsHull();

        updateBodyVisibility();

        updateImage();
    });

    stepButton = window->findChild<QPushButton*>("stepButton");
    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        detector->capture()->camera().rotate(5);
        detector->capture()->focus();
        updateImage();
    });

    auto skipButton = window->findChild<QPushButton*>("skipButton");
    QObject::connect(skipButton, &QPushButton::clicked, [&]() {
        profile.skip();
        polygonWidget->repaint();
    });

    auto refineButton = window->findChild<QPushButton*>("refineButton");
    QObject::connect(refineButton, &QPushButton::clicked, [&]() {
        refine();

        updateBodyVisibility();

        sceneWidget->update();
        polygonWidget->repaint();
    });

    auto completeButton = window->findChild<QPushButton*>("completeButton");
    QObject::connect(completeButton, &QPushButton::clicked, [&]() {
        while (!profile.complete()) {
            refine();
        }

        updateBodyVisibility();

        sceneWidget->update();
        polygonWidget->repaint();
    });

    auto mergeButton = window->findChild<QPushButton*>("mergeButton");
    QObject::connect(mergeButton, &QPushButton::clicked, [&]() {
        if (body->tryMerge()) {
            std::cout << "Hulls merged!\n";
            updateBodyVisibility();
            sceneWidget->updateRenderGeometry(body->mesh());
            sceneWidget->update();
        }
    });

    auto formButton = window->findChild<QPushButton*>("formButton");
    QObject::connect(formButton, &QPushButton::clicked, [&]() {
        if (body->form()) {
            updateBodyVisibility();
            sceneWidget->updateRenderGeometry(body->mesh());
            sceneWidget->update();
        }
    });

    auto *epsilonField = window->findChild<QSpinBox*>("epsilonField");
    QObject::connect(epsilonField, &QSpinBox::valueChanged, [](int value) {
        detector->setEpsilon((double)value);
        updateImage();
    });

    auto *newMeshButton = window->findChild<QPushButton*>("newMeshButton");
    QObject::connect(newMeshButton, &QPushButton::clicked, [&]() {
        const QString fileName = QFileDialog::getOpenFileName(nullptr, "Select Model",
                                                     "../res/meshes", "Model Files (*.obj)");

        if (!fileName.isEmpty()) {
            mesh = MeshHandler::loadAsMeshBody(fileName.toStdString());

            scene->clear();
            body = std::make_shared<Sculpture>(mesh, 1.0f, 1.0f);
            showModelSetting->setCheckState(Qt::CheckState::Checked);
            showSculptureSetting->setCheckState(Qt::CheckState::Checked);
            body->applyCompositeColors(showHullSetting->isChecked());
            scene->prepareBody(body);
            scene->prepareBody(body->model());

            body->print();
            body->model()->print();

//            body->restoreAsHull();

//            body->setMesh(mesh, false);
            sceneWidget->clear();

            detector->setMesh(mesh);
            detector->capture()->focus();

            updateImage();
        }

    });


    mesh = MeshHandler::loadAsMeshBody(R"(..\res\meshes\devil.obj)");
    mesh->center();


    scene = std::make_shared<Scene>();
    body = std::make_shared<Sculpture>(mesh, 1.0f, 1.0f);
    scene->prepareBody(body);
    scene->prepareBody(body->model());


    detector = new EdgeDetect(mesh);

    mesh->print();
    body->model()->mesh()->print();

    detector->setSize(400);
    detector->setEpsilon((double)epsilonField->value());

    detector->capture()->camera().setViewingAngle(-180 * body->rotation() / M_PI, 0);
    detector->capture()->focus();

    body->print();
    body->model()->print();


    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(5, 0, 0));
    sceneWidget->setScene(scene);

    window->show();

    updateImage();


    return app.exec();
}
