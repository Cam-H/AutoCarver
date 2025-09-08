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
std::shared_ptr<RigidBody> cut = nullptr;

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
    if (profile.complete()) return;

    auto operation = profile.next(0.108, 0.011);
    if (operation.valid) {

        auto lVertices = std::vector<glm::dvec2>{
                operation.start,
                operation.splitVertex(),
                operation.end
        };

        auto margin = profile.clearance();
        if (margin.first > 10 && margin.second > 10) {
            lVertices[0] += operation.BA * 2.0;
            lVertices[2] += operation.BC * 2.0;
        }

        auto border = std::vector<glm::dvec3>{
                profile.projected3D(lVertices[0]),
                profile.projected3D(lVertices[1]),
                profile.projected3D(lVertices[2])
        };

        body->queueSection(border[0], border[1], border[2], profile.normal());
        body->applySection();

        auto extrude = MeshBuilder::extrude(border, profile.normal(), 1);
        extrude->translate(0.5 * -profile.normal());
        extrude->setFaceColor({ 0, 1, 1 });
        if (cut == nullptr) cut = scene->createBody(extrude);
        else cut->setMesh(extrude);

        profile.refine();
    } else {
        std::cout << "Invalid operation!\n";
        profile.skip();
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
        scene->update();
    });

    showSculptureSetting = window->findChild<QCheckBox*>("showSculptureSetting");
    QObject::connect(showSculptureSetting, &QCheckBox::clicked, [&]() {
        if (!showSculptureSetting->isChecked()) {
            showHullSetting->setCheckState(Qt::CheckState::Unchecked);
        }

        updateBodyVisibility();
        scene->update();
    });

    showHullSetting = window->findChild<QCheckBox*>("showHullSetting");
    QObject::connect(showHullSetting, &QCheckBox::clicked, [&]() {
        if (showHullSetting->isChecked()) {
            showSculptureSetting->setCheckState(Qt::CheckState::Checked);
            updateBodyVisibility();
        }

        updateColoring();
        scene->update();
    });

    auto resetButton = window->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        body->restoreAsHull();

        updateBodyVisibility();

        updateImage();
        scene->update();
    });

    stepButton = window->findChild<QPushButton*>("stepButton");
    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        detector->capture()->camera().rotate(5);
        detector->capture()->focus();
        updateImage();
        scene->update();
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

        scene->update();
        sceneWidget->update();
        polygonWidget->repaint();
    });

    auto completeButton = window->findChild<QPushButton*>("completeButton");
    QObject::connect(completeButton, &QPushButton::clicked, [&]() {
        while (!profile.complete()) {
            refine();
        }

        updateBodyVisibility();

        scene->update();
        sceneWidget->update();
        polygonWidget->repaint();
    });

    auto mergeButton = window->findChild<QPushButton*>("mergeButton");
    QObject::connect(mergeButton, &QPushButton::clicked, [&]() {
        if (body->tryMerge()) {
            std::cout << "Hulls merged!\n";
            updateBodyVisibility();
            scene->update();

            sceneWidget->updateRenderGeometry(body->mesh());
            sceneWidget->update();
        }
    });

    auto formButton = window->findChild<QPushButton*>("formButton");
    QObject::connect(formButton, &QPushButton::clicked, [&]() {
        if (body->form()) {
            updateBodyVisibility();
            scene->update();

            sceneWidget->updateRenderGeometry(body->mesh());
            sceneWidget->update();
        }
    });

    auto *epsilonField = window->findChild<QSpinBox*>("epsilonField");
    QObject::connect(epsilonField, &QSpinBox::valueChanged, [](int value) {
        detector->setEpsilon((double)value);
        updateImage();
    });

    auto saveButton = window->findChild<QPushButton*>("saveButton");
    QObject::connect(saveButton, &QPushButton::clicked, [&]() {
        profile.save("../out/orthoprof.bin");
//        new Profile("../out/orthoprof.bin");
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
            scene->createVisual(mesh);
            scene->update();

            body->print();
//            body->model()->print();

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
    body = std::make_shared<Sculpture>(mesh, 1.0, 2.0);
    scene->prepareBody(body);
    scene->createVisual(mesh);

    detector = new EdgeDetect(mesh);

    mesh->print();
    body->model()->mesh()->print();

    detector->setSize(400);
    detector->setEpsilon((double)epsilonField->value());

    detector->capture()->camera().setViewingAngle(-180 * body->rotation() / M_PI, 0);
    detector->capture()->focus();

    body->print();
//    body->model()->print();


    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(5, 0, 0));
    sceneWidget->setScene(scene);

    body->applyCompositeColors(showHullSetting->isChecked());

    scene->update();
    window->show();

    updateImage();


    return app.exec();
}
