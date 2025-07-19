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
    glm::vec3 offset = -mesh->boundedOffset();
    float scalar = body->scalar();

    std::cout << scalar << " " << offset.x << " " << offset.y << " " << offset.z << "!\n";

    detector->update();

    profile = detector->profile();
    profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);
//    profile.rotateAbout({ 0, 1, 0 }, -body->rotation());
//        profile.centerScale(scalar);
//    profile.scale(scalar);
    profile.translate(offset);

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

void refine()
{
    bool external = profile.isNextExternal();
    auto indices = profile.refine();
    auto border = profile.projected3D(indices);

    if (!border.empty()) {
        body->queueSection(border[0], border[1], border[2], profile.normal(), external);
        body->applySection();

//            auto extrude = MeshBuilder::extrude(border, profile.normal(), 1);
//            extrude->translate(0.5f * -profile.normal());
//            extrude->setFaceColor({ 0, 1, 1 });
//            scene->createBody(extrude);
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


#ifndef QT_NO_OPENGL

    mesh = MeshHandler::loadAsMeshBody(R"(..\res\meshes\devil.obj)");
    mesh->center();
//    mesh->normalize(5.0f);

//    hull = std::make_shared<Mesh>(ConvexHull(mesh->vertices()));
//    glm::vec3 centroid = -hull->centroid();
//    mesh->translate(centroid.x, centroid.y, centroid.z);
//    hull->zero();

    scene = std::make_shared<Scene>();
    body = std::make_shared<Sculpture>(mesh, 1.0f, 1.0f);
    scene->prepareBody(body);

//    body = scene->createBody(mesh);
//    body = scene->createBody(std::make_shared<Mesh>(ConvexHull(mesh)));
//    body->prepareColliderVisuals();
//    body->zero();

//    std::vector<glm::vec3> border = {
//            {0, 0, 0},
//            {2, 0, 0},
//            {2, 2, 0},
//            {0, 2, 0}
//    };
//    auto normal = FaceArray::calculateNormal(border);
//    std::cout << normal.x << " " << normal.y << " " << normal.z << " Normal\n";
//    auto mesh = MeshBuilder::extrude(border, normal, 5.0f);
//    mesh->setFaceColor({0, 0, 1});
//    scene->createBody(mesh);

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(5, 0, 0));
    sceneWidget->setScene(scene);

//    sceneWidget->show(0, Scene::Model::HULL);

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    showHullSetting = window->findChild<QCheckBox*>("showHullSetting");
    QObject::connect(showHullSetting, &QCheckBox::clicked, [&]() {
        body->applyCompositeColors(showHullSetting->checkState() == Qt::CheckState::Checked);
        sceneWidget->updateRenderGeometry(body->mesh());
        sceneWidget->update();
    });

    auto resetButton = window->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        body->restoreAsHull();
        updateImage();
    });

    stepButton = window->findChild<QPushButton*>("stepButton");
    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        detector->capture()->camera().rotate(5);
        detector->capture()->focus();
        updateImage();

        auto vec = mesh->boundedOffset();
        auto sc = profile.spanCenter();
        std::cout << vec.x << " " << vec.y << " " << vec.z << " ~~~ " << body->position().x << " " << body->position().y << " " << body->position().z << "\n";
        std::cout << profile.xSpan() << " " << profile.ySpan() << " " << sc.x << " " << sc.y << "\n";
    });

    auto refineButton = window->findChild<QPushButton*>("refineButton");
    QObject::connect(refineButton, &QPushButton::clicked, [&]() {
        refine();

        sceneWidget->update();
        polygonWidget->repaint();
    });

    auto completeButton = window->findChild<QPushButton*>("completeButton");
    QObject::connect(completeButton, &QPushButton::clicked, [&]() {
        while (!profile.complete()) {
            refine();
        }

        sceneWidget->update();
        polygonWidget->repaint();
    });

    auto formButton = window->findChild<QPushButton*>("formButton");
    QObject::connect(formButton, &QPushButton::clicked, [&]() {
        if (body->form()) {
            sceneWidget->updateRenderGeometry(body->mesh());
            sceneWidget->update();
        }
    });

    auto *epsilonField = window->findChild<QSpinBox*>("epsilonField");
    QObject::connect(epsilonField, &QSpinBox::valueChanged, [](int value) {
        detector->setEpsilon((float)value);
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
            body->applyCompositeColors(showHullSetting->checkState() == Qt::CheckState::Checked);
            scene->prepareBody(body);
//            body->restoreAsHull();

//            body->setMesh(mesh, false);
            sceneWidget->clear();

            detector->setMesh(mesh);
            detector->capture()->focus();

            updateImage();
        }

    });

    detector = new EdgeDetect(mesh);

    detector->setSize(400);
    detector->setEpsilon((float)epsilonField->value());

    detector->capture()->camera().setViewingAngle(0, 0);
    detector->capture()->focus();


    window->show();

    updateImage();


    return app.exec();
}
