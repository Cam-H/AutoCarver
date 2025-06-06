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
#include "geometry/MeshBuilder.h"
#include "robot/ArticulatedWrist.h"

#include "renderer/UiLoader.h"
#include "renderer/RenderCapture.h"
#include "renderer/EdgeDetect.h"

#endif

QWidget *window = nullptr;
QLabel *modelView = nullptr;
QLabel *processView = nullptr;

std::shared_ptr<Mesh> mesh = nullptr;
std::shared_ptr<Mesh> hull = nullptr;

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

    detector->sink().save(QString("image.png"));

    px = new QPixmap;
    px->convertFromImage(detector->source());
    modelView->setPixmap(*px);

    px = new QPixmap;
    px->convertFromImage(detector->sink());
    processView->setPixmap(*px);

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


#ifndef QT_NO_OPENGL

    mesh = MeshHandler::loadAsMeshBody(R"(..\res\meshes\devil.obj)");

    hull = std::make_shared<Mesh>(ConvexHull(mesh->vertices()));
    glm::vec3 centroid = -hull->centroid();
    mesh->translate(centroid.x, centroid.y, centroid.z);
    hull->zero();

    detector = new EdgeDetect(mesh);

    detector->capture()->camera().setViewingAngle(0, 0);
    detector->capture()->focus();

    updateImage();

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    // Handle adjacent prev/next buttons
    auto *stepButton = window->findChild<QPushButton*>("stepButton");
    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        detector->capture()->camera().rotate(5);
        detector->capture()->focus();
        updateImage();
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

    auto *projButton = window->findChild<QPushButton*>("projButton");
    QObject::connect(projButton, &QPushButton::clicked, [&]() {
        if (detector->capture()->camera().getType() == Camera::Type::PERSPECTIVE)
            detector->capture()->camera().setType(Camera::Type::ORTHOGRAPHIC);
        else detector->capture()->camera().setType(Camera::Type::PERSPECTIVE);
        updateImage();
    });

    window->show();

    return app.exec();
}
