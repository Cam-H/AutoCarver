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

#endif

std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget *sceneWidget = nullptr;
std::shared_ptr<Robot> robot = nullptr;

QListWidget* qlw;

std::vector<QSpinBox*> jointFields;

std::vector<Waypoint> m_waypoints = {
        { std::vector<float>{ 0, 0, 0, 0, 0, 0 }, false },
        { std::vector<float>{ 180, 50, -25, 0, -50, 0 }, false }
};
std::pair<int, int> m_selection = { 0, 0 };
Trajectory* m_trajectory = nullptr;

LineChartWidget *plotWidget = nullptr;


static std::string toString(const Waypoint& waypoint)
{
    std::string content = "{ ";
    for (uint32_t i = 0; i < waypoint.values.size() - 1; i++) content += std::to_string(waypoint.values[i]) + ", ";
    content += std::to_string(waypoint.values[waypoint.values.size() - 1]) + " }";

    return content;
}

static QWidget *loadUiFile(QWidget *parent)
{
    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/robot-pathing/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;
//    builder.addPluginPath("C:/Users/Cam/CLionProjects/AutoCarver/src/renderer/");
//
//    std::cout << builder.pluginPaths().count() << " plugin paths...\n";
//    for (auto path : builder.pluginPaths()) {
//        std::cout << "Path: " << path.toStdString() << "\n\n";
//    }

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    auto *widget = loadUiFile(nullptr);
    if (widget == nullptr) return -1;

#ifndef QT_NO_OPENGL

    std::string source = R"(..\res\meshes\devil.obj)";
    auto model = MeshHandler::loadAsMeshBody(source);

    model->setBaseColor({1, 0, 1});


    scene = std::make_shared<SculptProcess>(model);
    robot = scene->createRobot(new ArticulatedWrist(0.8, 2, 2, 1));
    robot->translate({1, 0, 0});
    robot->update();

    sceneWidget = widget->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->setScene(scene);

    plotWidget = widget->findChild<LineChartWidget*>("graphWidget");
    plotWidget->ylim(-10, 10);

    {
        std::vector<float> y(200);

        for (uint32_t i = 0; i < y.size(); i++) y[i] = i * 0.1f;
        plotWidget->plot(y);

        for (uint32_t i = 0; i < y.size(); i++) y[i] = 5.0f * cosf(i * M_PI * 0.04f);
        plotWidget->plot(y, "Sinusoid");

    }

    plotWidget->update();


    qlw = widget->findChild<QListWidget*>("waypointWidget");

    // Prepare UI elements to interact with waypoints
    for (const Waypoint& waypoint : m_waypoints) {
        qlw->addItem(toString(waypoint).c_str());
    }

    // Handle waypoint selection change
    QObject::connect(qlw, &QListWidget::itemSelectionChanged, [&]() {
        for (uint32_t i = 0; i < jointFields.size(); i++) {
            jointFields[i]->setValue((int)m_waypoints[qlw->currentRow()].values[i]);
        }
    });

    // Handle joint field update
    auto *jiw = widget->findChild<QWidget*>("jointInputWidget");
    for (uint32_t i = 0; i < 6; i++) {
        std::string name = "j" + std::to_string(i) + "Field";
        auto *jointField = jiw->findChild<QSpinBox*>(name.c_str());
        QObject::connect(jointField, &QSpinBox::valueChanged, [jointField, i](int value) {
            robot->setJointValueDg(i, (float)value);

            robot->update();
            sceneWidget->update();

            jointField->setValue((int)robot->getJointValueDg(i));
            m_waypoints[qlw->currentRow()].values[i] = robot->getJointValueDg(i);
            qlw->item(qlw->currentRow())->setText(toString(m_waypoints[qlw->currentRow()]).c_str());
        });

        jointFields.push_back(jointField);
    }


#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    // Handle adjacent prev/next buttons
    auto *prevButton = widget->findChild<QPushButton*>("prevButton");
    QObject::connect(prevButton, &QPushButton::clicked, [&]() {
        if (qlw->currentRow() > 0) qlw->setCurrentRow(qlw->currentRow() - 1);
    });
    auto *nextButton = widget->findChild<QPushButton*>("nextButton");
    QObject::connect(nextButton, &QPushButton::clicked, [&]() {
        if (qlw->currentRow() < m_waypoints.size() - 1) qlw->setCurrentRow(qlw->currentRow() + 1);
    });

    // Handle request for additional waypoints
    auto *waypointButton = widget->findChild<QPushButton*>("waypointButton");
    QObject::connect(waypointButton, &QPushButton::clicked, [&]() {
        m_waypoints.push_back(m_waypoints[qlw->currentRow()]);
        qlw->addItem(toString(m_waypoints[qlw->currentRow()]).c_str());
    });

    // Handle selection of waypoints for trajectory
    auto *startField = widget->findChild<QSpinBox*>("startIdxField");
    m_selection.first = startField->value();
    QObject::connect(startField, &QSpinBox::valueChanged, [&, startField](int value) {
        if (value >= m_waypoints.size()) {
            value = (int)m_waypoints.size() - 1;
            startField->setValue(value);
        }

        m_selection.first = value;
    });
    auto *endField = widget->findChild<QSpinBox*>("endIdxField");
    m_selection.second = endField->value();
    QObject::connect(endField, &QSpinBox::valueChanged, [&, endField](int value) {
        if (value >= m_waypoints.size()) {
            value = (int)m_waypoints.size() - 1;
            endField->setValue(value);
        }

        m_selection.second = value;
    });

    // Handle request to generate/follow a trajectory
    auto *trajectoryButton = widget->findChild<QPushButton*>("trajectoryButton");
    QObject::connect(trajectoryButton, &QPushButton::clicked, [&]() {
        if (m_selection.first != m_selection.second) {
            delete m_trajectory;

//            m_trajectory = new Trajectory(m_waypoints[m_selection.first], m_waypoints[m_selection.second], TrajectorySolverType::LINEAR);
//            robot->traverse(*m_trajectory);
//            JointTrajectory jt(0, 180);
            std::vector<float> y(100);
            for (uint32_t i = 0; i < y.size(); i++) {
//                y[i] = jt.interpolate(0, 0.01f * i);
                y[i] = cosf(i * M_PI * 0.01f);
            }
            // TODO plot
            plotWidget->clear();
            plotWidget->plot(y);
        }
    });

    auto *serializeButton = widget->findChild<QPushButton*>("serializeButton");
    QObject::connect(serializeButton, &QPushButton::clicked, [&]() {
        std::cout << "serialize unimplemented!\n";
    });

    auto *deserializeButton = widget->findChild<QPushButton*>("deserializeButton");
    QObject::connect(deserializeButton, &QPushButton::clicked, [&]() {
        std::cout << "deserialize unimplemented!\n";
    });

    widget->show();

    return app.exec();


//
//    auto robotButton = new QCheckBox("Show mesh", control);
//    robotButton->setChecked(true);
//    hControlLayout->addWidget(robotButton);
//
//    QObject::connect(robotButton, &QCheckBox::clicked, [&](bool checked) {
//        if (checked) sceneWidget->showAll(Scene::Model::MESH);
//        else sceneWidget->hideAll(Scene::Model::MESH);
//    });
//
//    auto hullButton = new QCheckBox("Show convex hull", control);
//    hControlLayout->addWidget(hullButton);
//
//    QObject::connect(hullButton, &QCheckBox::clicked, [&](bool checked) {
//        if (checked) sceneWidget->showAll(Scene::Model::HULL);
//        else sceneWidget->hideAll(Scene::Model::HULL);
//    });
//
//    auto bSphereButton = new QCheckBox("Show bounding sphere", control);
//    hControlLayout->addWidget(bSphereButton);
//
//    QObject::connect(bSphereButton, &QCheckBox::clicked, [&](bool checked) {
//        if (checked) sceneWidget->showAll(Scene::Model::BOUNDING_SPHERE);
//        else sceneWidget->hideAll(Scene::Model::BOUNDING_SPHERE);
//    });
//
//    auto stepButton = new QPushButton("Next step", control);
//    hControlLayout->addWidget(stepButton);
//
//    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
//        scene->next();
//    });
//
//    auto captureButton = new QPushButton("Capture", control);
//    hControlLayout->addWidget(captureButton);
//
//    QObject::connect(captureButton, &QPushButton::clicked, [&]() {
////        m_rc->capture();
//        if (sceneWidget != nullptr) {
//            auto image = sceneWidget->grabFramebuffer();
////            auto img = new uint8_t[image.width()][image.height()];
//
//            image.save("..\\out\\capture.png");
//        }
//    });
//
}
