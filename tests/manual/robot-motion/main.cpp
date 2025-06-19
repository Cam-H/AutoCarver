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

#ifndef QT_NO_OPENGL
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "core/SculptProcess.h"
#include "robot/ArticulatedWrist.h"

#endif

std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget *sceneWidget = nullptr;
std::shared_ptr<Robot> robot = nullptr;

std::vector<QSpinBox*> jointFields;
std::vector<QDoubleSpinBox*> posFields;

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

//    QSize screenSize = view->screen()->size();
//    container->setMinimumSize(QSize(500, 500));
//    container->setMaximumSize(screenSize);

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

    std::string source = R"(..\res\meshes\devil.obj)";
    auto model = MeshHandler::loadAsMeshBody(source);

    model->setBaseColor({1, 0, 1});


    scene = std::make_shared<SculptProcess>(model);
    robot = scene->createRobot(new ArticulatedWrist(0.8, 2, 2, 1));

    auto box = MeshBuilder::box(2, 2, 1);
    box->translate({ 3, 0, 0 });
    scene->createBody(box);

    sceneWidget = new SceneWidget(scene);
    hRenderLayout->addWidget(sceneWidget);

    auto *robotControl = new QWidget;
    auto *vRobotControlLayout = new QVBoxLayout;
    robotControl->setLayout(vRobotControlLayout);
    hRenderLayout->addWidget(robotControl);

    robot->setJointValueDg(1, 135);
    robot->setJointValueDg(2, -45);

    robot->update();
    sceneWidget->update();


    // Joint angle control
    for (uint32_t i = 0; i < 6; i++) {
        auto *field = new QSpinBox(robotControl);
        field->setSingleStep(5);
        field->setRange(std::numeric_limits<int>::lowest(), std::numeric_limits<int>::max());
        field->setValue(robot->getJointValueDg(i));
        QObject::connect(field, &QSpinBox::valueChanged, [field, i](int value) {
            robot->setJointValueDg(i, value);
            robot->update();
            sceneWidget->update();

            field->setValue(robot->getJointValueDg(i));

            // Update position fields to match
            glm::vec3 position = robot->getEOATPosition();
            for (uint32_t j = 0; j < 3; j++) {
                posFields[j]->blockSignals(true);
                posFields[j]->setValue(position[j]);
                posFields[j]->blockSignals(false);
            }

            // Update orientation fields to match
            glm::vec3 euler = robot->getEOATEuler();
            for (uint32_t j = 0; j < 3; j++) {
                posFields[3 + j]->blockSignals(true);
                posFields[3 + j]->setValue(euler[j]);
                posFields[3 + j]->blockSignals(false);
            }
        });

        vRobotControlLayout->addWidget(field);
        jointFields.push_back(field);
    }


    // IK Position control
    auto *positionControl = new QWidget;
    auto *vPositionControlLayout = new QHBoxLayout;
    positionControl->setLayout(vPositionControlLayout);
    vRobotControlLayout->addWidget(positionControl);

    glm::vec3 position = robot->getEOATPosition();

    // EOAT position
    for (uint32_t i = 0; i < 3; i++) {
        auto *field = new QDoubleSpinBox(positionControl);
        field->setSingleStep(0.1);
        field->setRange(-5.0, 5.0);
        field->setValue(position[i]);
        QObject::connect(field, &QDoubleSpinBox::valueChanged, [field, i](double value) {
            glm::vec3 position = robot->getEOATPosition();
            position[i] = value;

//            robot->moveTo(position);
            robot->moveTo(position, robot->getEOATEuler());
            sceneWidget->update();

            field->blockSignals(true);
            field->setValue(robot->getEOATPosition()[i]);
            field->blockSignals(false);

            // Update joint fields to match
            for (uint32_t j = 0; j < 6; j++) {
                jointFields[j]->blockSignals(true);
                jointFields[j]->setValue(robot->getJointValueDg(j));
                jointFields[j]->blockSignals(false);
            }
        });
        vPositionControlLayout->addWidget(field);
        posFields.push_back(field);
    }

    // EOAT Orientation
    for (uint32_t i = 0; i < 3; i++) {
        auto *field = new QDoubleSpinBox(positionControl);
        field->setSingleStep(0.1);
        field->setRange(-5.0, 5.0);
        field->setValue(position[i]);
        QObject::connect(field, &QDoubleSpinBox::valueChanged, [field, i](double value) {
            glm::vec3 euler = robot->getEOATEuler();
            euler[i] = value;

            robot->moveTo(robot->getEOATPosition(), euler);
            sceneWidget->update();

            field->blockSignals(true);
            field->setValue(robot->getEOATEuler()[i]);
            field->blockSignals(false);

            // Update joint fields to match
            for (uint32_t j = 0; j < 6; j++) {
                jointFields[j]->blockSignals(true);
                jointFields[j]->setValue(robot->getJointValueDg(j));
                jointFields[j]->blockSignals(false);
            }
        });
        vPositionControlLayout->addWidget(field);
        posFields.push_back(field);
    }


//    auto *sw2 = new SceneWidget(new SculptProcess(model));
//    hRenderLayout->addWidget(sw2);

//    auto *imageLabel = new QLabel();
//    imageLabel->setBackgroundRole(QPalette::Base);
//    imageLabel->setMinimumSize(QSize(100, 100));
//    hRenderLayout->addWidget(imageLabel);

//    scene->start();
#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    auto robotButton = new QCheckBox("Show mesh", control);
    robotButton->setChecked(true);
    hControlLayout->addWidget(robotButton);

    QObject::connect(robotButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::MESH);
        else sceneWidget->hideAll(Scene::Model::MESH);
    });

    auto hullButton = new QCheckBox("Show convex hull", control);
    hControlLayout->addWidget(hullButton);

    QObject::connect(hullButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::HULL);
        else sceneWidget->hideAll(Scene::Model::HULL);
    });

    auto bSphereButton = new QCheckBox("Show bounding sphere", control);
    hControlLayout->addWidget(bSphereButton);

    QObject::connect(bSphereButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::BOUNDING_SPHERE);
        else sceneWidget->hideAll(Scene::Model::BOUNDING_SPHERE);
    });

    auto stepButton = new QPushButton("Next step", control);
    hControlLayout->addWidget(stepButton);

    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        scene->next();
    });

    auto captureButton = new QPushButton("Capture", control);
    hControlLayout->addWidget(captureButton);

    QObject::connect(captureButton, &QPushButton::clicked, [&]() {
//        m_rc->capture();
        if (sceneWidget != nullptr) {
            auto image = sceneWidget->grabFramebuffer();
//            auto img = new uint8_t[image.width()][image.height()];

            image.save("..\\out\\capture.png");
        }
    });

//    auto thread = std::thread(update);

    window.show();

    return app.exec();
}
