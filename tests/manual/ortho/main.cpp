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

#ifndef QT_NO_OPENGL
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "core/SculptProcess.h"
#include "robot/ArticulatedWrist.h"
#include "renderer/LineChartWidget.h"

#include "renderer/UiLoader.h"
#include "robot/planning/Trajectory.h"
#include "renderer/RenderCapture.h"

#endif

QWidget *window = nullptr;
QLabel *view = nullptr;

std::shared_ptr<Mesh> mesh = nullptr;
std::shared_ptr<Mesh> hull = nullptr;

RenderCapture *rc = nullptr;

QPixmap* px;

bool edgeDetect = false;

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
    rc->capture();

    QImage image = rc->grabFramebuffer();

    if (edgeDetect) { // Do image processing

    }

//    image.save(QString("image.png"));

    px = new QPixmap;
    px->convertFromImage(image);
    view->setPixmap(*px);
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    view = window->findChild<QLabel*>("graphics");


#ifndef QT_NO_OPENGL

    mesh = MeshHandler::loadAsMeshBody(R"(..\res\meshes\devil.obj)");
    mesh->zero();

    hull = std::make_shared<Mesh>(ConvexHull(mesh->vertices()));

    rc = new RenderCapture(nullptr, QSize(500, 500));
    rc->addTarget(hull, QColor(0, 0, 255));
    rc->addTarget(mesh, QColor(255, 0, 0));

//    rc->addTarget(MeshBuilder::box(0.6f, 0.6f, 4.0f));
//    rc->addTarget(MeshBuilder::box(1.0f, 1.0f, 1.0f), QColor(0, 0, 255));
//    rc->addTarget(MeshBuilder::box(0.6f, 8.0f, 0.6f), QColor(255, 0, 0));

    rc->camera().setViewingAngle(0, 0);
    rc->focus();

    updateImage();

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    // Handle adjacent prev/next buttons
    auto *stepButton = window->findChild<QPushButton*>("stepButton");
    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        rc->camera().rotate(5);
        rc->focus();
        updateImage();
    });

    auto *projButton = window->findChild<QPushButton*>("projButton");
    QObject::connect(projButton, &QPushButton::clicked, [&]() {
        if (rc->camera().getType() == Camera::Type::PERSPECTIVE) rc->camera().setType(Camera::Type::ORTHOGRAPHIC);
        else rc->camera().setType(Camera::Type::PERSPECTIVE);
        updateImage();
    });

    auto *detectButton = window->findChild<QPushButton*>("detectButton");
    QObject::connect(detectButton, &QPushButton::clicked, [&]() {
        edgeDetect = !edgeDetect;
        updateImage();
    });

    window->show();

    return app.exec();
}
