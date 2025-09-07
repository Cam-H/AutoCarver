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

#include "geometry/collision/Collision.h"

#ifndef QT_NO_OPENGL
#include "ControlWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "process/SculptProcess.h"
#include "robot/ArticulatedWrist.h"

#endif

#include "geometry/Mesh.h"
#include "geometry/MeshBuilder.h"
#include "geometry/primitives/ConvexHull.h"
#include "geometry/primitives/ConvexHull.h"
#include "geometry/primitives/AABB.h"
#include "geometry/primitives/AABB.h"
#include "geometry/primitives/Ray.h"
#include "geometry/primitives/Ray.h"
#include "geometry/collision/EPA.h"

std::shared_ptr<Mesh> base;
std::shared_ptr<Mesh> test;

ControlWidget *sceneWidget = nullptr;
std::shared_ptr<Scene> scene = nullptr;

std::shared_ptr<Mesh> randomMesh()
{
    static QRandomGenerator rng;

    // Generate a cloud of count vertices within a cube around the origin
    uint32_t count = rng.global()->bounded(4, 12);
    std::vector<glm::dvec3> cloud(count);
    auto *ptr = (double*)cloud.data();

    for (uint32_t j = 0; j < 3 * count; j++) *ptr++ = (double)rng.global()->bounded(2.0) - 1.0f;

    return std::make_shared<Mesh>(ConvexHull(cloud), false);
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


//    auto cp = CollisionPair(Sphere(), Plane());
//    std::cout << "Int: " << cp.intersects() << "|\n";

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
    std::cout << "ax\n";

    scene = std::make_shared<Scene>();

    base = MeshBuilder::box(2, 2, 1);

//    test = MeshBuilder::cylinder(0.5, 2, 6);
//    test = MeshBuilder::icosphere(1.0f, 3);
    test = MeshBuilder::icosphere();

//    auto hull = ConvexHull(VertexArray("VertexSet.bin"));
//    VertexArray(hull.vertices()).print();
//    hull.faces().print();
//    hull.evaluate();
//    hull.print();
//    std::cout << "==============\n";
//    base = std::make_shared<Mesh>(hull);

//    auto box = AABB({-1, -1, -1}, {1, 1, 1});
//    auto axes = std::vector<glm::dvec3>{
//            {0, 1, 0},
//            glm::normalize(glm::dvec3{1, 1, 0}),
//            {1, 0, 0},
//            glm::normalize(glm::dvec3{1, -0.5f, 0}),
//            {0, -1, 0}
//    };
//
//    for (const auto& axis : axes) {
//        const auto& [col, t, c] = Collision::intersection(box, Ray({}, axis));
//        std::cout << col << " " << t << " " << c.x << " " << c.y << " " << c.z << "~\n";
//    }

    scene->createBody(base);
    scene->createBody(test);

    sceneWidget = new ControlWidget(scene);
    sceneWidget->setFocusPolicy(Qt::StrongFocus); // Enable keyboard focus

    hRenderLayout->addWidget(sceneWidget);



    std::cout << "bx\n";

    sceneWidget->update();
    std::cout << "cx\n";

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    auto randomButton = new QPushButton("Randomize Shapes", control);
    hControlLayout->addWidget(randomButton);

    QObject::connect(randomButton, &QPushButton::clicked, [&]() {
        scene->bodies()[0]->setMesh(randomMesh());
        scene->bodies()[1]->setMesh(randomMesh());

        sceneWidget->update();
    });


    auto saveButton = new QPushButton("Save State", control);
    hControlLayout->addWidget(saveButton);

    QObject::connect(saveButton, &QPushButton::clicked, [&]() {
        if (scene->save("../out/scene.bin")) std::cout << "Serialization complete!\n";
        else std::cout << "Serialization failed!\n";

        sceneWidget->update();
    });


    auto restoreButton = new QPushButton("Restore State", control);
    hControlLayout->addWidget(restoreButton);

    QObject::connect(restoreButton, &QPushButton::clicked, [&]() {
        sceneWidget->clear();

        if (scene->load("../out/scene.bin")) std::cout << "Deserialization complete!\n";
        else std::cout << "Deserialization failed!\n";

        sceneWidget->handleCollision();

        sceneWidget->update();
    });

    std::cout << "x\n";

    scene->update();
    window.show();

    return app.exec();
}
