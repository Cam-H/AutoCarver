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
#include <QRandomGenerator>


#ifndef QT_NO_OPENGL
#include "ControlWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "core/SculptProcess.h"
#include "robot/ArticulatedWrist.h"

#endif

#include "geometry/Mesh.h"
#include "geometry/MeshBuilder.h"
#include "geometry/ConvexHull.h"

std::shared_ptr<Mesh> base;
std::shared_ptr<Mesh> test;

ControlWidget *sceneWidget = nullptr;
std::shared_ptr<Scene> scene = nullptr;

std::vector<std::shared_ptr<Body>> bodies;

std::shared_ptr<Mesh> randomMesh()
{
    static QRandomGenerator rng;

    // Generate a cloud of count vertices within a cube around the origin
    uint32_t count = rng.global()->bounded(4, 12);
    auto *cloud = new float[3 * count], *ptr = cloud;
    for (uint32_t j = 0; j < 3 * count; j++) *ptr++ =  (float)rng.global()->bounded(2.0) - 1.0f;

    return std::make_shared<Mesh>(ConvexHull(cloud, count), false);
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    QMainWindow window;
    window.setWindowTitle(QStringLiteral("Auto Carver - Convex Hull Algorithm Testing"));

    window.resize(1200, 700);


    auto *content = new QWidget;
    window.setCentralWidget(content);

    auto *vLayout = new QVBoxLayout; // Highest-level layout for content
    vLayout->setAlignment(Qt::AlignTop);
    content->setLayout(vLayout);

    auto *render = new QWidget;
    render->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto *hRenderLayout = new QHBoxLayout; // Layout for render content
    render->setLayout(hRenderLayout);

    auto *control = new QWidget;
    auto *hControlLayout = new QHBoxLayout;
    control->setLayout(hControlLayout);

    vLayout->addWidget(render);
    vLayout->addWidget(control);

#ifndef QT_NO_OPENGL

    scene = std::make_shared<Scene>();

    base = MeshBuilder::box(2, 2, 1);

    test = MeshBuilder::cylinder(0.5, 2, 6);
    test->translate(0.5, 0.5, 0.5);

    bodies.push_back(scene->createBody(base));
    bodies.push_back(scene->createBody(test));

//    auto body = bodies[1];
//    body->translate({1, 2, 3});
//    body->serialize("../out/body.bin");
//
////    auto body = std::make_shared<Body>("../out/body.bin");
//    body->deserialize("../out/body.bin");
//    auto tt = body->getTransform();
//
//    std::cout << tt[0][0] << " " << tt[1][0] << " " << tt[2][0] << " " << tt[3][0] << "\n"
//              << tt[0][1] << " " << tt[1][1] << " " << tt[2][1] << " " << tt[3][1] << "\n"
//              << tt[0][2] << " " << tt[1][2] << " " << tt[2][2] << " " << tt[3][2] << "\n"
//              << tt[0][3] << " " << tt[1][3] << " " << tt[2][3] << " " << tt[3][3] << "\n";


    sceneWidget = new ControlWidget(scene);
    sceneWidget->setFocusPolicy(Qt::StrongFocus); // Enable keyboard focus

    hRenderLayout->addWidget(sceneWidget);

    sceneWidget->update();

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    auto randomButton = new QPushButton("Randomize Shapes", control);
    hControlLayout->addWidget(randomButton);

    QObject::connect(randomButton, &QPushButton::clicked, [&]() {
        for (auto &body : bodies) {
            body->setMesh(randomMesh(), true);
        }

        sceneWidget->update();
    });


    auto saveButton = new QPushButton("Save State", control);
    hControlLayout->addWidget(saveButton);

    QObject::connect(saveButton, &QPushButton::clicked, [&]() {
        if (scene->serialize("../out/scene.bin")) std::cout << "Serialization complete!\n";
        else std::cout << "Serialization failed!\n";

        sceneWidget->update();
    });


    auto restoreButton = new QPushButton("Restore State", control);
    hControlLayout->addWidget(restoreButton);

    QObject::connect(restoreButton, &QPushButton::clicked, [&]() {
        sceneWidget->clear();

        if (scene->deserialize("../out/scene.bin")) std::cout << "Deserialization complete!\n";
        else std::cout << "Deserialization failed!\n";

        sceneWidget->update();
    });

    window.show();

    return app.exec();
}
