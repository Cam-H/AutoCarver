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
#include "geometry/RigidBody.h"
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

QCheckBox *fixedCameraSetting = nullptr;
QPushButton *stepButton = nullptr;

std::shared_ptr<RigidBody> body = nullptr;
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
    detector->update();

    auto fwd = detector->capture()->camera().forward();
    glm::vec3 axis = { fwd.x(), fwd.y(), fwd.z() };

    profile = detector->profile();
    profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);

    std::cout << "Border: " << profile.vertexCount() << "\n";
    auto extrude = MeshBuilder::extrude(profile.projected3D(), -axis, 4);
    if (extrude != nullptr) {
        extrude->setBaseColor({1, 0, 1});
        extrude->translate(2.0f * axis);
        MeshHandler::exportMesh(extrude, "border.obj");

        if (silhouette == nullptr) silhouette = scene->createBody(extrude);
        else silhouette->setMesh(extrude, false);
    }

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
    mesh->normalize(5.0f);

//    hull = std::make_shared<Mesh>(ConvexHull(mesh->vertices()));
//    glm::vec3 centroid = -hull->centroid();
//    mesh->translate(centroid.x, centroid.y, centroid.z);
//    hull->zero();

    scene = std::make_shared<Scene>();
    body = scene->createBody(mesh);
//    body = scene->createBody(std::make_shared<Mesh>(ConvexHull(mesh)));
//    body->prepareColliderVisuals();
//    body->zero();

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(5, 0, 0));
    sceneWidget->setScene(scene);

//    sceneWidget->show(0, Scene::Model::HULL);

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    fixedCameraSetting = window->findChild<QCheckBox*>("fixedCameraSetting");
    QObject::connect(fixedCameraSetting, &QCheckBox::clicked, [&]() {
        stepButton->setEnabled(fixedCameraSetting->checkState() == Qt::CheckState::Checked);
//        if (detector->capture()->camera().getType() == Camera::Type::PERSPECTIVE)
//            detector->capture()->camera().setType(Camera::Type::ORTHOGRAPHIC);
//        else detector->capture()->camera().setType(Camera::Type::PERSPECTIVE);
//        updateImage();
    });

    stepButton = window->findChild<QPushButton*>("stepButton");
    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        detector->capture()->camera().rotate(5);
        detector->capture()->focus();
        updateImage();
    });

    auto refineButton = window->findChild<QPushButton*>("refineButton");
    QObject::connect(refineButton, &QPushButton::clicked, [&]() {
        profile.refine();
        polygonWidget->repaint();
    });

    auto *sizeField = window->findChild<QSpinBox*>("sizeField");
    QObject::connect(sizeField, &QSpinBox::valueChanged, [](int value) {
        detector->setSize(value);
        updateImage();
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
            mesh->normalize(5.0f);

            body->setMesh(mesh, false);
            sceneWidget->clear();

            detector->setMesh(mesh);
            detector->capture()->focus();

            updateImage();
        }

    });

    QObject::connect(sceneWidget, &SceneWidget::perspectiveChanged, [&]() {
        std::cout << "Perspective change trigger entry\n";
        if (fixedCameraSetting->checkState() == Qt::CheckState::Unchecked) {
            detector->capture()->camera().setPosition(sceneWidget->camera().getPosition());
            detector->capture()->focus();
            updateImage();
        }
    });

    detector = new EdgeDetect(mesh);

    detector->setSize(sizeField->value());
    detector->setEpsilon((float)epsilonField->value());

    detector->capture()->camera().setViewingAngle(0, 0);
    detector->capture()->focus();

    updateImage();

    window->show();


    return app.exec();
}
