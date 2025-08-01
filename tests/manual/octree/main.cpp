#include <QApplication>
#include <QSurfaceFormat>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QSpinBox>

#include <QFile>
#include <QDir>
#include <vector>

#ifndef QT_NO_OPENGL
#include "geometry/primitives/Sphere.h"
#include "geometry/primitives/AABB.h"
#include "geometry/primitives/ConvexHull.h"
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "geometry/Octree.h"
#include "core/Scene.h"
#include "robot/ArticulatedWrist.h"
#include "renderer/LineChartWidget.h"

#include "renderer/UiLoader.h"
#include "robot/trajectory/Trajectory.h"

#endif

std::shared_ptr<Scene> scene = nullptr;
SceneWidget *sceneWidget = nullptr;

QSpinBox *depthField = nullptr;
QCheckBox *updateMeshButton = nullptr;

std::shared_ptr<Octree> oct = nullptr;

std::shared_ptr<RigidBody> render = nullptr;
std::shared_ptr<RigidBody> collider = nullptr;

bool updateMesh = true;

std::array<double, 6> pos = { 0, 0, 0, 0, 0, 0 };
uint32_t shapeType = 0, operationType = 0;

double radius = 0.4;

#include <QRandomGenerator>

std::shared_ptr<Mesh> randomMesh()
{
    static QRandomGenerator rng;

    // Generate a cloud of count vertices within a cube around the origin
    uint32_t count = rng.global()->bounded(8, 12);
    std::vector<glm::dvec3> cloud(count);
    auto *ptr = (double*)cloud.data();

    for (uint32_t j = 0; j < 3 * count; j++) *ptr++ = (double)rng.global()->bounded(2 * radius) - radius;

    return std::make_shared<Mesh>(ConvexHull(cloud), false);
}

static QWidget *loadUiFile(QWidget *parent)
{
//    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/octree/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void sceneUpdate()
{
    sceneWidget->update();
}

template <class T>
void apply(const T& body)
{
    switch (operationType) {
        case 0:
        {
            static bool collision = false;
            bool test = oct->collides(body) && updateMesh;
            if (test != collision) {
                auto mesh = MeshBuilder::mesh(oct);
                if (test) mesh->setFaceColor({ 1, 0, 0});
                else mesh->setFaceColor({ 1, 1, 1});
                render->setMesh(mesh);
                collision = test;
            }

            auto val = oct->locateParent(body);
            std::cout << "Pos: " << val << " |} ";
            if (val == std::numeric_limits<uint32_t>::max()) std::cout << "\n";
            else std::cout << oct->isParent(0, val) << " " << oct->isParent(7, val) << "\n";
        }

            break;
        case 1:
//                    if (oct->unite(body) && updateMesh) render->setMesh(MeshBuilder::mesh(oct));
            break;
        case 2:
            if (oct->subtract(body) && updateMesh) render->setMesh(MeshBuilder::mesh(oct));
            break;
        case 3:
//                    if (oct->intersect(body) && updateMesh) render->setMesh(MeshBuilder::mesh(oct));
            break;
        default: std::cout << "Unknown type: " << operationType << "\n";
    }
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    auto *widget = loadUiFile(nullptr);
    if (widget == nullptr) return -1;

#ifndef QT_NO_OPENGL

    scene = std::make_shared<Scene>();

    auto colliderMesh = MeshBuilder::icosphere(radius, 3);
    collider = scene->createBody(colliderMesh);

    sceneWidget = widget->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->camera().setPosition(QVector3D(10, -1, 0));
    sceneWidget->setScene(scene);

    // Handle position field update
    auto *jiw = widget->findChild<QWidget*>("inputWidget");
    std::array<std::string, 6> names = { "xField", "yField", "zField", "rxField", "ryField", "rzField" };
    for (uint32_t i = 0; i < 6; i++) {
        auto *jointField = jiw->findChild<QDoubleSpinBox*>(names[i].c_str());
        QObject::connect(jointField, &QDoubleSpinBox::valueChanged, [i](double value) {
            pos[i] = value;

            const glm::dvec3 position = { pos[0], pos[1], pos[2] };
            collider->setPosition(position);
            collider->setRotation({ pos[3], pos[4], pos[5] });

            switch (shapeType) {
                case 0:
                    apply(Sphere(position, radius));
                    break;
                case 1:
                    apply(AABB(position - 0.5 * radius * glm::dvec3{ 1, 1, 1 }, radius));
                    break;
                case 2:
                {
                    auto vertices = VertexArray(collider->hull().vertices());
                    vertices.translate(position);
                    apply(ConvexHull(vertices));
                }
                    break;
            }

//            std::cout << oct->memoryFootprint() << "\n";
            sceneWidget->update();
        });

        pos[i] = (double)jointField->value();
    }

    collider->setPosition({ pos[0], pos[1], pos[2] });
    collider->setRotation({ pos[3], pos[4], pos[5] });

#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    // Handle Octree reset button
    auto *resetButton = widget->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        oct->reset();

        if (render != nullptr) render->setMesh(MeshBuilder::mesh(oct));
    });

    // Handle mesh sphere button
    auto *sphereButton = widget->findChild<QPushButton*>("sphereButton");
    QObject::connect(sphereButton, &QPushButton::clicked, [&]() {
        collider->setMesh(MeshBuilder::icosphere(radius, 3), true);
        shapeType = 0;
    });

    // Handle mesh cube button
    auto *cubeButton = widget->findChild<QPushButton*>("cubeButton");
    QObject::connect(cubeButton, &QPushButton::clicked, [&]() {
        collider->setMesh(MeshBuilder::box(radius), true);
        shapeType = 1;
    });

    // Handle mesh randomization button
    auto *randomButton = widget->findChild<QPushButton*>("randomButton");
    QObject::connect(randomButton, &QPushButton::clicked, [&]() {
        collider->setMesh(randomMesh(), true);
        shapeType = 2;
    });

    depthField = widget->findChild<QSpinBox*>("depthField");
    QObject::connect(depthField, &QSpinBox::valueChanged, [&](int value) {
        oct->setMaximumDepth(value);
        render->setMesh(MeshBuilder::mesh(oct));
        sceneWidget->update();
    });

    updateMeshButton = widget->findChild<QCheckBox*>("updateMeshButton");
    QObject::connect(updateMeshButton, &QCheckBox::clicked, [&]() {
        if (updateMeshButton->checkState() == Qt::CheckState::Checked) {
            render->setMesh(MeshBuilder::mesh(oct));
            sceneWidget->update();
        }

        updateMesh = updateMeshButton->checkState() == Qt::CheckState::Checked;
    });

    auto *noneRButton = widget->findChild<QRadioButton*>("noneRButton");
    QObject::connect(noneRButton, &QRadioButton::pressed, [&]() {
        operationType = 0;
    });

    auto *uniteRButton = widget->findChild<QRadioButton*>("uniteRButton");
    QObject::connect(uniteRButton, &QRadioButton::pressed, [&]() {
        operationType = 1;
    });

    auto *differenceRButton = widget->findChild<QRadioButton*>("differenceRButton");
    QObject::connect(differenceRButton, &QRadioButton::pressed, [&]() {
        operationType = 2;
    });

    auto *intersectRButton = widget->findChild<QRadioButton*>("intersectRButton");
    QObject::connect(intersectRButton, &QRadioButton::pressed, [&]() {
        operationType = 3;
    });

    oct = std::make_shared<Octree>(depthField->value(), 5.0);

    auto model = MeshBuilder::mesh(oct);
    model->setBaseColor({1, 1, 1 });
    render = scene->createBody(model);

    widget->show();

    scene->start();

    scene->connect(&sceneUpdate);
//    sceneWidget->start();

    return app.exec();
}
