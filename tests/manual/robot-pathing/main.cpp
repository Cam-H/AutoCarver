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
#include "robot/trajectory/SimpleTrajectory.h"
#include "robot/trajectory/TOPPTrajectory.h"
#include "geometry/curves/Interpolator.h"

#include "core/Timer.h"

#endif

std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget *sceneWidget = nullptr;
std::shared_ptr<Robot> robot = nullptr;

QListWidget* qlw;

std::vector<QSpinBox*> jointFields;

std::vector<Waypoint> m_waypoints = {
        Waypoint(std::vector<double>{ 0, 0, 0, 0, 0, 0 }, true),
        Waypoint(std::vector<double>{ 180, 55, -25, 0, -50, 0 }, true),
        Waypoint(std::vector<double>{ 270, 100, 60, 0, 30, 50 }, true)

};
std::pair<int, int> m_selection = { 0, 0 };
std::shared_ptr<Trajectory> m_trajectory = nullptr;
std::vector<double> m_x = {};
bool m_inputEnable = true;

LineChartWidget *plotWidget = nullptr;

std::unique_ptr<std::thread> updateThread;


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
    robot = scene->createRobot(std::make_shared<ArticulatedWrist>(0.8, 2, 2, 1));
    robot->translate({1, 0, 0});
    robot->update();

    auto body = scene->createBody(MeshBuilder::box(5.0f, 5.0f, 0.1f));
    body->mesh()->setBaseColor({0.6f, 0.6f, 0.6f});
    body->translate({0, -0.5f, 0});

    sceneWidget = widget->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->setScene(scene);

    plotWidget = widget->findChild<LineChartWidget*>("graphWidget");
    plotWidget->ylim(-180, 180);


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
            if (!m_inputEnable) return;

            robot->setJointValueDg(i, (double)value);

            scene->step(0);
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
        qlw->setCurrentRow(m_waypoints.size() - 1);
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
        if (m_inputEnable && m_waypoints.size() > 1) {

//            auto traj = std::make_shared<SimpleTrajectory>(m_waypoints[0], m_waypoints[1], Interpolator::SolverType::QUINTIC);
            auto traj = std::make_shared<TOPPTrajectory>(m_waypoints);
            traj->limitVelocity(90.0);
            traj->limitAcceleration(60.0);
            traj->smooth();

            m_trajectory = traj;
            robot->traverse(m_trajectory);
            robot->step();

            plotWidget->clear();
            plotWidget->zero();

            plotWidget->setX(500, 0, std::ceil(m_trajectory->duration()));
            plotWidget->ylim(-360, 360);

            for (uint32_t i = 0; i < 6; i++) {
                std::string name = "j" + std::to_string(i);
                plotWidget->create(500, (name + "Position").c_str());
            }

            for (uint32_t i = 0; i < 6; i++) {
                std::string name = "j" + std::to_string(i);
                plotWidget->create(500, (name + "Velocity").c_str());
//                plotWidget->create(500, (name + "Acceleration").c_str());
            }

            m_inputEnable = false;

            qlw->setCurrentRow(m_waypoints.size() - 1);


            plotWidget->update();
        }
    });

    // Handle updating robot
    updateThread = std::make_unique<std::thread>([](){
        Timer rateTimer;
        uint32_t xIdx = 0;
        double time = 0;

        while (true) {
            double delta = rateTimer.getElapsedSeconds();
            scene->step(delta);
            rateTimer.reset();

            if (robot->inTransit()) {

                plotWidget->setX(++xIdx, time += delta);
                Waypoint pos = robot->getWaypoint().toDg();
                for (uint32_t i = 0; i < 6; i++) plotWidget->stream(i, pos.values[i]);
                auto vel = robot->getJointVelocity(), acc = robot->getJointAcceleration();
                for (uint32_t i = 0; i < 6; i++) plotWidget->stream(6 + i, vel[i]);
//                for (uint32_t i = 0; i < 6; i++) plotWidget->stream(6 + i, acc[i]);

                plotWidget->update();

                sceneWidget->update();

                for (uint32_t i = 0; i < jointFields.size(); i++) {
                    jointFields[i]->setValue((int)robot->getJointValueDg(i));
                }

            } else if (!m_inputEnable) {
                for (uint32_t i = 0; i < jointFields.size(); i++) {
                    jointFields[i]->setValue((int)robot->getJointValueDg(i));
                }
                m_inputEnable = true;

                xIdx = 0;
                time = 0;
            }

            std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));
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
}
