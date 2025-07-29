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
#include <QFile>
#include <QDir>


#ifndef QT_NO_OPENGL
#include "renderer/UiLoader.h"
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "core/SculptProcess.h"
#include "robot/ArticulatedWrist.h"
#include "geometry/Axis3D.h"
#include "core/Timer.h"

#include "robot/Pose.h"
#include "robot/planning/Interpolator.h"
#include "robot/planning/SimpleTrajectory.h"
#include "robot/planning/CompoundTrajectory.h"
#include "robot/planning/TrajectoryBuilder.h"

#endif

QWidget *window = nullptr;

std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget *sceneWidget = nullptr;
std::shared_ptr<Robot> robot = nullptr;

std::vector<QSpinBox*> jointFields;
std::vector<QDoubleSpinBox*> posFields;

QTreeWidget* treeWidget = nullptr;
QTreeWidgetItem* waypointWidget = nullptr;
QTreeWidgetItem* trajectoryWidget = nullptr;

QTreeWidgetItem *selected = nullptr;

QPushButton *removeTrajectoryButton = nullptr, *addTrajectoryButton = nullptr;
QRadioButton *linearButton = nullptr, *cubicButton = nullptr, *quinticButton = nullptr;
QRadioButton *simpleButton = nullptr, *constrainedButton = nullptr, *compoundButton = nullptr;
QPushButton *followButton = nullptr;

QSpinBox *wpStartField = nullptr, *wpEndField = nullptr;

LineChartWidget *plotWidget = nullptr;

std::unique_ptr<std::thread> updateThread;

bool localTranslation = false;
Axis3D axes;
double theta = M_PI / 64;

std::vector<Waypoint> waypoints;
std::vector<std::shared_ptr<Trajectory>> trajectories;

Interpolator::SolverType solver = Interpolator::SolverType::QUINTIC;
uint8_t trajType = 0;

static QWidget *loadUiFile(QWidget *parent)
{
    QFile file("../tests/manual/robot-motion/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void updateJointFields()
{
    for (uint32_t j = 0; j < 6; j++) {
        jointFields[j]->blockSignals(true);
        jointFields[j]->setValue(robot->getJointValueDg(j));
        jointFields[j]->blockSignals(false);
    }
}

void updatePositionFields()
{
    axes = robot->getEOATAxes();

    auto position = robot->getEOATPosition();
    if (localTranslation) position = axes.localize(position);

    for (uint32_t j = 0; j < 3; j++) {
        posFields[j]->blockSignals(true);
        posFields[j]->setValue(position[j]);
        posFields[j]->blockSignals(false);
    }
}

void updateAxes()
{
    robot->moveTo(axes);
    sceneWidget->update();

    if (localTranslation) updatePositionFields();

    updateJointFields();
}

std::shared_ptr<Trajectory> currentTrajectory()
{
    if (selected != nullptr) {
        if (selected->parent() == trajectoryWidget) return trajectories[trajectoryWidget->indexOfChild(selected)];
        else if (selected->parent() != nullptr && selected->parent()->parent() == trajectoryWidget)
            return trajectories[trajectoryWidget->indexOfChild(selected->parent())];
    }

    return nullptr;
}

bool selectedTrajectoryWaypoint()
{
    return selected != nullptr && selected->parent() != nullptr && selected->parent()->parent() == trajectoryWidget;
}

bool selectedWaypoint()
{
    return selected != nullptr && (selected->parent() == waypointWidget || selectedTrajectoryWaypoint());
}

void saveWaypoint(const Waypoint& waypoint)
{
    std::string name = "WP" + std::to_string(waypoints.size());
    waypoints.push_back(waypoint);

    auto *item = new QTreeWidgetItem(waypointWidget);
    item->setText(0, name.c_str());
    item->setText(1, waypoints.back().toString().c_str());
    item->setData(1, Qt::UserRole, waypoints.size() - 1);
    waypointWidget->setExpanded(true);

    wpStartField->setMaximum(waypoints.size() - 1);
    wpEndField->setMaximum(waypoints.size() - 1);
}

void saveWaypoint()
{
    saveWaypoint(robot->getWaypoint().toDg());
}

void addWaypoint(uint32_t index)
{
    std::string name = "WP" + std::to_string(index);

    auto *trajItem = selected;
    if (selected == nullptr || selected->parent() != trajectoryWidget)
        trajItem = trajectoryWidget->child(trajectoryWidget->childCount() - 1);

    auto *item = new QTreeWidgetItem(trajItem);
    item->setText(0, name.c_str());
    item->setData(1, Qt::UserRole, index);
    trajItem->setExpanded(true);
}

void applyWaypoint()
{
    robot->moveTo(waypoints[selected->data(1, Qt::UserRole).toUInt()]);

    updateJointFields();
    updatePositionFields();

    scene->step(0.01);
    sceneWidget->update();
}

void removeTrajectory()
{
    if (selected != nullptr && selected->parent() == trajectoryWidget) {
        trajectories.erase(trajectories.begin() + trajectoryWidget->indexOfChild(selected));
        trajectoryWidget->removeChild(selected);
    }
}

void addTrajectory()
{
    if (wpStartField->value() == wpEndField->value()) return;

    std::string name = "TRAJ" + std::to_string(trajectories.size());

    const Waypoint& start = waypoints[wpStartField->value()], end = waypoints[wpEndField->value()];
    TrajectoryBuilder tb(robot);
    tb.setSolver(solver);

    std::shared_ptr<Trajectory> traj = nullptr;
    trajType = 1;
    switch (trajType) {
        case 0:
            tb.addWaypoint(start);
            tb.addWaypoint(end);
            break;
        case 1:
        {
            std::vector<Waypoint> waypoints = { robot->getWaypoint().toDg() };
            for (uint32_t i = 0; i < 10; i++) {
                Waypoint wp = waypoints.back();
                wp.values[0] += 10;
                waypoints.push_back(wp);
            }

            traj = std::make_shared<CompoundTrajectory>(waypoints, 60, 100);
//            auto startPose = robot->getPose(start);
//
//            tb.addWaypoint(start);
//            tb.moveConstrained(startPose.axes.localize({ 3, 0, 0 }));
        }

            break;
        case 2:
            break;
    }

//    tb.setVelocityLimit(90);
//    tb.setAccelerationLimit(200);
//    tb.generate();
//    tb.isValid()

    if (traj != nullptr) {

        trajectories.push_back(traj);

        auto *item = new QTreeWidgetItem(trajectoryWidget);
        item->setText(0, name.c_str());
        item->setData(1, Qt::UserRole, trajectories.size() - 1);
        trajectoryWidget->setExpanded(true);

        item->setSelected(true);
        selected = item;

        addWaypoint(wpStartField->value());
        addWaypoint(wpEndField->value());
    }
}

void preparePlot(const std::shared_ptr<Trajectory>& traj)
{
    double delta = traj->maximumDelta();
    delta = traj->maximumVelocity();

    std::cout << "Trajectory: " << traj->duration() << " " << traj->maximumVelocity() << " " << traj->maximumAcceleration() << "\n";

    plotWidget->clear();
    plotWidget->zero();

    plotWidget->setX(500, 0, std::ceil(traj->duration()));
    plotWidget->ylim(-delta, delta);

    for (uint32_t i = 0; i < 6; i++) {
        std::string name = "j" + std::to_string(i);
        plotWidget->create(500, (name + "Delta").c_str());
    }

//    for (uint32_t i = 0; i < 6; i++) {
//        std::string name = "j" + std::to_string(i);
//        plotWidget->create(500, (name + "Velocity").c_str());
//    }
//
//    for (uint32_t i = 0; i < 6; i++) {
//        std::string name = "j" + std::to_string(i);
//        plotWidget->create(500, (name + "Acceleration").c_str());
//    }

    plotWidget->update();
}

void stream(uint32_t idx, double time)
{
    plotWidget->setX(idx, time);

//    Waypoint pos = robot->getWaypoint().toDg();
//    for (uint32_t i = 0; i < 6; i++) plotWidget->stream(i, pos.values[i]);

    auto del = robot->getDistanceTravelled();
    for (uint32_t i = 0; i < 6; i++) plotWidget->stream(i, del[i]);

//    auto vel = robot->getJointVelocity();
//    for (uint32_t i = 0; i < 6; i++) plotWidget->stream(i, vel[i]);
//
//    auto acc = robot->getJointAcceleration();
//    for (uint32_t i = 0; i < 6; i++) plotWidget->stream(6 + i, acc[i]);

    plotWidget->update();
}

void enableInputs(bool enable = true)
{
    for (auto *field : jointFields) field->setEnabled(enable);
    for (auto *field : posFields) field->setEnabled(enable);
}

void doTrajectoryControlEnable()
{
    removeTrajectoryButton->setEnabled(selected != nullptr && selected->parent() == trajectoryWidget);
    addTrajectoryButton->setEnabled(wpStartField->value() != wpEndField->value());
    followButton->setEnabled(currentTrajectory() != nullptr);
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    std::string source = R"(..\res\meshes\devil.obj)";
    auto model = MeshHandler::loadAsMeshBody(source);

    model->setBaseColor({1, 0, 1});


    scene = std::make_shared<SculptProcess>(model);
    robot = scene->createRobot(std::make_shared<ArticulatedWrist>(0.8, 2, 2, 1));
    robot->translate({ -2, 0, 0 });
    robot->rotate({ 0, 1, 0 }, M_PI);

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->setScene(scene);

    robot->setJointValueDg(1, 135);
    robot->setJointValueDg(2, -45);
    axes = robot->getEOATAxes();

    robot->update();
    sceneWidget->update();

    jointFields = {
            window->findChild<QSpinBox*>("j0Field"),
            window->findChild<QSpinBox*>("j1Field"),
            window->findChild<QSpinBox*>("j2Field"),
            window->findChild<QSpinBox*>("j3Field"),
            window->findChild<QSpinBox*>("j4Field"),
            window->findChild<QSpinBox*>("j5Field")
    };

    posFields = {
            window->findChild<QDoubleSpinBox*>("xField"),
            window->findChild<QDoubleSpinBox*>("yField"),
            window->findChild<QDoubleSpinBox*>("zField")
    };

    // Joint angle control
    for (uint8_t i = 0; i < 6; i++) {
        auto *field = jointFields[i];

        field->setValue(std::round(robot->getJointValueDg(i)));
        QObject::connect(field, &QSpinBox::valueChanged, [field, i](int value) {
            robot->setJointValueDg(i, value);
            robot->update();
            sceneWidget->update();

            field->setValue(std::round(robot->getJointValueDg(i)));

            updatePositionFields();
        });
    }


    // EOAT position
    for (uint8_t i = 0; i < 3; i++) {
        auto *field = posFields[i];
        auto position = robot->getEOATPosition();
        field->setValue(position[i]);

        QObject::connect(field, &QDoubleSpinBox::valueChanged, [i](double value) {
            glm::dvec3 position = robot->getEOATPosition();
            if (localTranslation) position = axes.localize(position);
            position[i] = value;

            if (localTranslation) position = axes.delocalize(position);

            robot->moveTo(position);
            sceneWidget->update();

            updatePositionFields();
            updateJointFields();
        });
    }

    auto localSettingButton = window->findChild<QCheckBox*>("localSettingButton");
    QObject::connect(localSettingButton, &QCheckBox::clicked, [&](bool checked) {
        localTranslation = checked;
        updatePositionFields();
    });

    auto decXButton = window->findChild<QPushButton*>("decXButton");
    QObject::connect(decXButton, &QPushButton::clicked, [&]() {
        axes.rotateX(-theta);
        updateAxes();
    });

    auto incXButton = window->findChild<QPushButton*>("incXButton");
    QObject::connect(incXButton, &QPushButton::clicked, [&]() {
        axes.rotateX(theta);
        updateAxes();
    });

    auto decYButton = window->findChild<QPushButton*>("decYButton");
    QObject::connect(decYButton, &QPushButton::clicked, [&]() {
        axes.rotateY(-theta);
        updateAxes();
    });

    auto incYButton = window->findChild<QPushButton*>("incYButton");
    QObject::connect(incYButton, &QPushButton::clicked, [&]() {
        axes.rotateY(theta);
        updateAxes();
    });

    auto decZButton = window->findChild<QPushButton*>("decZButton");
    QObject::connect(decZButton, &QPushButton::clicked, [&]() {
        axes.rotateZ(-theta);
        updateAxes();
    });

    auto incZButton = window->findChild<QPushButton*>("incZButton");
    QObject::connect(incZButton, &QPushButton::clicked, [&]() {
        axes.rotateZ(theta);
        updateAxes();
    });


    auto showMeshButton = window->findChild<QCheckBox*>("showMeshButton");
    QObject::connect(showMeshButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::MESH);
        else sceneWidget->hideAll(Scene::Model::MESH);
    });

    auto showHullButton = window->findChild<QCheckBox*>("showHullButton");
    QObject::connect(showHullButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::HULL);
        else sceneWidget->hideAll(Scene::Model::HULL);
    });

    auto showSphereButton = window->findChild<QCheckBox*>("showSphereButton");
    QObject::connect(showSphereButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::BOUNDING_SPHERE);
        else sceneWidget->hideAll(Scene::Model::BOUNDING_SPHERE);
    });

    treeWidget = window->findChild<QTreeWidget*>("treeWidget");
    treeWidget->setHeaderLabels({ "Name", "Data" });

    waypointWidget = new QTreeWidgetItem(treeWidget);
    waypointWidget->setText(0, "Waypoints");

    trajectoryWidget = new QTreeWidgetItem(treeWidget);
    trajectoryWidget->setText(0, "Trajectories");

    QObject::connect(treeWidget, &QTreeWidget::currentItemChanged, [&](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
        selected = current;

        if (selectedWaypoint()) {
            applyWaypoint();
        }

        selected = current;
        doTrajectoryControlEnable();
    });

    auto saveWaypointButton = window->findChild<QPushButton*>("saveWaypointButton");
    QObject::connect(saveWaypointButton, &QPushButton::clicked, [&]() {
        saveWaypoint();
    });


    removeTrajectoryButton = window->findChild<QPushButton*>("removeTrajectoryButton");
    QObject::connect(removeTrajectoryButton, &QPushButton::clicked, [&]() {
        removeTrajectory();
        doTrajectoryControlEnable();
    });

    addTrajectoryButton = window->findChild<QPushButton*>("addTrajectoryButton");
    QObject::connect(addTrajectoryButton, &QPushButton::clicked, [&]() {
        addTrajectory();
        doTrajectoryControlEnable();
    });

    linearButton = window->findChild<QRadioButton*>("linearButton");
    QObject::connect(linearButton, &QRadioButton::clicked, [&]() {
        solver = Interpolator::SolverType::LINEAR;
    });

    cubicButton = window->findChild<QRadioButton*>("cubicButton");
    QObject::connect(cubicButton, &QRadioButton::clicked, [&]() {
        solver = Interpolator::SolverType::CUBIC;
    });

    quinticButton = window->findChild<QRadioButton*>("quinticButton");
    QObject::connect(quinticButton, &QRadioButton::clicked, [&]() {
        solver = Interpolator::SolverType::QUINTIC;
    });

    simpleButton = window->findChild<QRadioButton*>("simpleButton");
    QObject::connect(simpleButton, &QRadioButton::clicked, [&]() {
        trajType = 1;
    });

    constrainedButton = window->findChild<QRadioButton*>("constrainedButton");
    QObject::connect(constrainedButton, &QRadioButton::clicked, [&]() {
        trajType = 1;
    });

    compoundButton = window->findChild<QRadioButton*>("compoundButton");
    QObject::connect(compoundButton, &QRadioButton::clicked, [&]() {
        trajType = 2;
    });

    wpStartField = window->findChild<QSpinBox*>("wpStartField");
    QObject::connect(wpStartField, &QSpinBox::valueChanged, [&](int value) {
        doTrajectoryControlEnable();
//        if (selectedTrajectoryWaypoint()) {
//            std::string name = "WP" + std::to_string(value);
//            selected->setText(0, name.c_str());
//            selected->setData(1, Qt::UserRole, value);
//
//            auto traj = currentTrajectory();
//            if (traj != nullptr) traj->replaceWaypoint(selected->parent()->indexOfChild(selected), waypoints[value]);
//
//            applyWaypoint();
//        }
    });

    wpEndField = window->findChild<QSpinBox*>("wpEndField");
    QObject::connect(wpEndField, &QSpinBox::valueChanged, [&](int value) {
        doTrajectoryControlEnable();
    });

    followButton = window->findChild<QPushButton*>("followButton");
    QObject::connect(followButton, &QPushButton::clicked, [&]() {
        auto traj = currentTrajectory();
        if (traj != nullptr) {
            traj->restart();
            preparePlot(traj);
            robot->traverse(traj);
        }
    });

    saveWaypoint(Waypoint({   0, 135, -45,  0,   0, 0 }, true));
    saveWaypoint(Waypoint({ -10, 150, -35, 25, -15, 0 }, true));

    wpEndField->setValue(1);
    addTrajectory();

    doTrajectoryControlEnable();


    plotWidget = window->findChild<LineChartWidget*>("chartWidget");
    plotWidget->ylim(-180, 180);

    window->show();

    updateThread = std::make_unique<std::thread>([](){
        Timer rateTimer;
        uint32_t xIdx = 0;
        double time = 0;
        bool inProcess = false;

        while (true) {
            double delta = rateTimer.getElapsedSeconds();
            scene->step(delta);
            rateTimer.reset();

            if (robot->inTransit()) {
                sceneWidget->update();

                stream(++xIdx, time += delta);

                updateJointFields();
                updatePositionFields();

                if (!inProcess) enableInputs(false);
                inProcess = true;

            } else if (inProcess) {
                robot->step();
                enableInputs();
                inProcess = false;

                xIdx = 0;
                time = 0;
            }


            std::this_thread::sleep_for(std::chrono::nanoseconds(4000000));
        }
    });

    return app.exec();
}
