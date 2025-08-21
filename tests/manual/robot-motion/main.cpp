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

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/string_cast.hpp>

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
#include "geometry/curves/Interpolator.h"
#include "robot/trajectory/SimpleTrajectory.h"
#include "robot/trajectory/TOPPTrajectory.h"
#include "robot/trajectory/CartesianTrajectory.h"
#include "robot/trajectory/CompositeTrajectory.h"

#endif

QWidget *window = nullptr;

std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget *sceneWidget = nullptr;
std::shared_ptr<Robot> robot = nullptr;
std::shared_ptr<RigidBody> lastPick = nullptr;
std::shared_ptr<Sculpture> sculpture = nullptr;
std::shared_ptr<RigidBody> eoat = nullptr;

uint32_t planeIdx = std::numeric_limits<uint32_t>::max();

std::vector<QSpinBox*> jointFields;
std::vector<QDoubleSpinBox*> posFields;

QSpinBox *ttAngleField = nullptr;

QTreeWidget* treeWidget = nullptr;
QTreeWidgetItem* waypointWidget = nullptr;

QTreeWidgetItem *selected = nullptr;

QDoubleSpinBox *velField = nullptr, *accField = nullptr;

QRadioButton *linearButton = nullptr, *cubicButton = nullptr, *quinticButton = nullptr;
QPushButton *simpleTestButton = nullptr, *constrainedTestButton = nullptr, *compositeTestButton = nullptr;

QSpinBox *wpStartField = nullptr, *wpEndField = nullptr;
QDoubleSpinBox *dxField = nullptr, *dyField = nullptr, *dzField = nullptr;

std::vector<QCheckBox*> jPlotButtons;
QCheckBox* posPlotButton = nullptr, *delPlotButton = nullptr, *remPlotButton = nullptr, *velPlotButton = nullptr, *accPlotButton = nullptr, *devPlotButton = nullptr;

LineChartWidget *plotWidget = nullptr;
uint32_t xIdx = 0;

std::unique_ptr<std::thread> updateThread;

bool localTranslation = false;
Axis3D axes;
double theta = M_PI / 64;

std::vector<Waypoint> waypoints;

Interpolator::SolverType solver = Interpolator::SolverType::QUINTIC;

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
    axes = robot->getAxes();

    auto position = robot->getPosition();
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
    scene->update();
    sceneWidget->update();

    if (localTranslation) updatePositionFields();

    updateJointFields();
}

bool selectedWaypoint()
{
    return selected != nullptr && selected->parent() == waypointWidget;
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

    auto *item = new QTreeWidgetItem(waypointWidget);
    item->setText(0, name.c_str());
    item->setData(1, Qt::UserRole, index);
    waypointWidget->setExpanded(true);
}

void applyWaypoint()
{
    robot->moveTo(waypoints[selected->data(1, Qt::UserRole).toUInt()]);

    updateJointFields();
    updatePositionFields();

    scene->update();
    sceneWidget->update();
}

std::tuple<std::vector<double>, std::vector<double>> getLimits(double scalar)
{
    auto vLims = std::vector<double>(6, velField->value() * scalar);
    auto aLims = std::vector<double>(6, accField->value() * scalar);

    return { vLims, aLims };
}

void createValuePlots(const std::string& title)
{
    for (uint32_t i = 0; i < 6; i++) {
        if (jPlotButtons[i]->isChecked()) {
            std::string name = "j" + std::to_string(i) + title;
            plotWidget->create(2000, name.c_str());
        }
    }
}

void preparePlot(const std::shared_ptr<Trajectory>& traj)
{

    xIdx = 0;
    plotWidget->clear();
    plotWidget->zero();

    plotWidget->setX(2000, 0, std::ceil(traj->duration()));

    // Determine reasonable limits
    double lim = 0;
    if (posPlotButton->isChecked()) lim = 360;
    if (delPlotButton->isChecked() || remPlotButton->isChecked()) lim = std::max(lim, traj->maximumDelta());
    if (velPlotButton->isChecked()) lim = std::max(lim, traj->maximumVelocity());
    if (accPlotButton->isChecked()) lim = std::max(lim, traj->maximumAcceleration());

    plotWidget->ylim(-lim, lim);

    if (posPlotButton->isChecked()) createValuePlots(" Position");
    if (delPlotButton->isChecked()) createValuePlots(" Distance Travelled");
    if (remPlotButton->isChecked()) createValuePlots(" Distance Remaining");
    if (velPlotButton->isChecked()) createValuePlots(" Velocity");
    if (accPlotButton->isChecked()) createValuePlots(" Acceleration");

    plotWidget->update();
}

void plotValues(uint32_t& idx, const std::vector<double>& values)
{
    for (uint32_t i = 0; i < 6; i++) {
        if (jPlotButtons[i]->isChecked()) {
            plotWidget->stream(idx++, values[i]);
        }
    }
}

void stream(uint32_t idx, double time)
{
    plotWidget->setX(idx, time);

    uint32_t offset = 0;
    if (posPlotButton->isChecked()) plotValues(offset, robot->getWaypoint().toDg().values);
    if (delPlotButton->isChecked()) plotValues(offset, robot->getDistanceTravelled());
    if (remPlotButton->isChecked()) plotValues(offset, robot->getDistanceRemaining());
    if (velPlotButton->isChecked()) plotValues(offset, robot->getJointVelocity());
    if (accPlotButton->isChecked()) plotValues(offset, robot->getJointAcceleration());

    plotWidget->update();
}

void test(const std::shared_ptr<Trajectory>& traj)
{
    preparePlot(traj);
    robot->traverse(traj);
}

void enableInputs(bool enable = true)
{
    for (auto *field : jointFields) field->setEnabled(enable);
    for (auto *field : posFields) field->setEnabled(enable);
}

void doControlEnable()
{
    simpleTestButton->setEnabled(wpStartField->value() != wpEndField->value());
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
    sculpture = scene->getSculpture();
    sculpture->restoreAsHull();

//    for (const std::shared_ptr<RigidBody>& body : scene->bodies()) {
//        body->prepareColliderVisuals();
//    }

    robot = scene->createRobot(std::make_shared<ArticulatedWrist>(0.2, 1.2, 1.2, 0.35));
    robot->translate({ -2, 1, 0 });
    robot->rotate({ 0, 1, 0 }, M_PI);
    robot->setLinkMesh(0, MeshHandler::loadAsMeshBody(R"(..\res\meshes\RobotBase.obj)"));

    sceneWidget = window->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->setScene(scene);

    robot->setJointValueDg(1, 135);
    robot->setJointValueDg(2, -45);
    axes = robot->getAxes();

    robot->setLinkMesh(6, MeshHandler::loadAsMeshBody(R"(..\res\meshes\BladeAttachment.obj)"));

    auto eoatMesh = MeshHandler::loadAsMeshBody("../res/meshes/Blade.obj");
    eoat = scene->createBody(eoatMesh);
    eoat->prepareColliderVisuals();
    robot->setEOAT(eoat, false);
    eoat->setName("BLADE");

    scene->setSculptingRobot(robot);

    auto mesh = MeshHandler::loadAsMeshBody("../res/meshes/TurntableJ0.obj");
    mesh->rotate({0, 0, 1}, M_PI);
    mesh->scale(3);
    auto hullTest = scene->createBody(mesh);
    hullTest->prepareColliderVisuals();
    hullTest->translate({ -5, 1, 0 });

    auto collider = scene->createBody(MeshBuilder::box(0.4));
    collider->translate({ 0, 4, 0});
    collider->setType(RigidBody::Type::DYNAMIC);

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
            scene->update();
            sceneWidget->update();

            field->setValue(std::round(robot->getJointValueDg(i)));

            updatePositionFields();
        });
    }


    // EOAT position
    for (uint8_t i = 0; i < 3; i++) {
        auto *field = posFields[i];
        auto position = robot->getPosition();
        field->setValue(position[i]);

        QObject::connect(field, &QDoubleSpinBox::valueChanged, [i](double value) {
            glm::dvec3 position = robot->getPosition();
            if (localTranslation) position = axes.localize(position);
            position[i] = value;

            if (localTranslation) position = axes.delocalize(position);

            robot->moveTo(position);
            scene->update();
            sceneWidget->update();

            updatePositionFields();
            updateJointFields();
        });
    }

    ttAngleField = window->findChild<QSpinBox*>("ttAngleField");
    QObject::connect(ttAngleField, &QSpinBox::valueChanged, [&](int value) {
        scene->getTurntable()->setJointValueDg(0, value);
        scene->getTurntable()->update();

        if (planeIdx != std::numeric_limits<uint32_t>::max()) {
            scene->alignToFace(planeIdx);
            updatePositionFields();
            updateJointFields();

            scene->update();
        }

        sceneWidget->update();
    });

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
        for (const std::shared_ptr<RigidBody>& link : robot->links()) {
            sceneWidget->setVisibility(checked, link->getID(), Scene::Model::MESH);
        }
    });

    auto showHullButton = window->findChild<QCheckBox*>("showHullButton");
    sceneWidget->setVisibility(showHullButton->isChecked(), Scene::Model::HULL);
    QObject::connect(showHullButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->showAll(Scene::Model::HULL);
        else sceneWidget->hideAll(Scene::Model::HULL);
    });

    auto showSphereButton = window->findChild<QCheckBox*>("showSphereButton");
    sceneWidget->setVisibility(showSphereButton->isChecked(), Scene::Model::BOUNDING_SPHERE);
    QObject::connect(showSphereButton, &QCheckBox::clicked, [&](bool checked) {
        sceneWidget->setVisibility(checked, Scene::Model::BOUNDING_SPHERE);
    });

    auto showAxesButton = window->findChild<QCheckBox*>("showAxesButton");
    sceneWidget->setVisibility(showAxesButton->isChecked(), eoat->getID(), Scene::Model::AXES);
    QObject::connect(showAxesButton, &QCheckBox::clicked, [&](bool checked) {
        sceneWidget->setVisibility(checked, eoat->getID(), Scene::Model::AXES);
        sceneWidget->update();
    });

    QObject::connect(sceneWidget, &SceneWidget::mousepick, [&](Ray ray) {
        auto [body, t] = scene->raycast(ray);
        if (body == sculpture) { // Only select sculpture
            body->mesh()->setFaceColor(body->mesh()->baseColor());
            auto [hit, tDup, faceIdx] = body->pickFace(ray, std::numeric_limits<double>::max());
            if (hit) body->mesh()->setFaceColor(faceIdx, { 0, 1, 0 });
            else body->mesh()->setFaceColor({ 0, 0, 1 });

            planeIdx = faceIdx;
            scene->alignToFace(planeIdx);
            updatePositionFields();
            updateJointFields();

            scene->update();

            sceneWidget->updateRenderGeometry(body->mesh());
        } else planeIdx = std::numeric_limits<uint32_t>::max();

        if (body != lastPick) {
            if (lastPick == sculpture) {
                lastPick->mesh()->setFaceColor(lastPick->mesh()->baseColor());
                sceneWidget->updateRenderGeometry(lastPick->mesh());
            }

            lastPick = body;
        }

        sceneWidget->update();
    });

    treeWidget = window->findChild<QTreeWidget*>("treeWidget");
    treeWidget->setHeaderLabels({ "Name", "Data" });

    waypointWidget = new QTreeWidgetItem(treeWidget);
    waypointWidget->setText(0, "Waypoints");

    QObject::connect(treeWidget, &QTreeWidget::currentItemChanged, [&](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
        selected = current;

        if (selectedWaypoint()) {
            applyWaypoint();
        }

        selected = current;
        doControlEnable();
    });

    velField = window->findChild<QDoubleSpinBox*>("velField");
    accField = window->findChild<QDoubleSpinBox*>("accField");


    auto saveWaypointButton = window->findChild<QPushButton*>("saveWaypointButton");
    QObject::connect(saveWaypointButton, &QPushButton::clicked, [&]() {
        saveWaypoint();
    });

    auto spinTestButton = window->findChild<QPushButton*>("spinTestButton");
    QObject::connect(spinTestButton, &QPushButton::clicked, [&]() {
        auto traj = std::make_shared<SimpleTrajectory>(Waypoint({ 0 }, true), Waypoint({ 270 }, true), solver);
        traj->setLimits({ 90 }, { 300 });
        scene->getTurntable()->traverse(traj);
//
        bool result = traj->test(scene.get(), scene->getTurntable(), 0.05);
        std::cout << "Collision test: " << result << "\n";
    });

    wpStartField = window->findChild<QSpinBox*>("wpStartField");
    QObject::connect(wpStartField, &QSpinBox::valueChanged, [&](int value) {
        doControlEnable();
    });

    wpEndField = window->findChild<QSpinBox*>("wpEndField");
    QObject::connect(wpEndField, &QSpinBox::valueChanged, [&](int value) {
        doControlEnable();
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

    simpleTestButton = window->findChild<QPushButton*>("simpleTestButton");
    QObject::connect(simpleTestButton, &QRadioButton::clicked, [&]() {
        const Waypoint& start = waypoints[wpStartField->value()].toRad(), end = waypoints[wpEndField->value()].toRad();
        auto [vLims, aLims] = getLimits(M_PI / 180);

        auto traj = std::make_shared<SimpleTrajectory>(start, end, solver);
        traj->limit(vLims, aLims);

        std::cout << "Collision: " << traj->test(scene.get(), robot, 0.01) << "\n";

        test(traj);
    });


    dxField = window->findChild<QDoubleSpinBox*>("dxField");
    dyField = window->findChild<QDoubleSpinBox*>("dyField");
    dzField = window->findChild<QDoubleSpinBox*>("dzField");

    constrainedTestButton = window->findChild<QPushButton*>("constrainedTestButton");
    QObject::connect(constrainedTestButton, &QRadioButton::clicked, [&]() {
        auto startPose = robot->getPose();
        glm::dvec3 translation = { dxField->value(), dyField->value(), dzField->value() };

        auto [vLims, aLims] = getLimits(M_PI / 180);

        auto traj = std::make_shared<CartesianTrajectory>(robot, startPose, translation);
        traj->setLimits(vLims, aLims);

        if (traj->isValid()) test(traj);
        else std::cout << "Validation failed!\n";
    });


    compositeTestButton = window->findChild<QPushButton*>("compositeTestButton");
    QObject::connect(compositeTestButton, &QRadioButton::clicked, [&]() {
        const Waypoint& start = waypoints[wpStartField->value()].toRad(), mid = waypoints[wpEndField->value()].toRad();
        auto midPose = robot->getPose(mid);
        glm::dvec3 translation = { dxField->value(), dyField->value(), dzField->value() };

        auto [vLims, aLims] = getLimits(M_PI / 180);
        auto traj = std::make_shared<CompositeTrajectory>(6);
        traj->setLimits(vLims, aLims);
        traj->addTrajectory(std::make_shared<SimpleTrajectory>(start, mid, solver));
        traj->addTrajectory(std::make_shared<CartesianTrajectory>(robot, midPose, translation));
        traj->update();
        test(traj);
    });

    jPlotButtons = {
            window->findChild<QCheckBox*>("j0PlotButton"),
            window->findChild<QCheckBox*>("j1PlotButton"),
            window->findChild<QCheckBox*>("j2PlotButton"),
            window->findChild<QCheckBox*>("j3PlotButton"),
            window->findChild<QCheckBox*>("j4PlotButton"),
            window->findChild<QCheckBox*>("j5PlotButton")
    };

    posPlotButton = window->findChild<QCheckBox*>("posPlotButton");
    delPlotButton = window->findChild<QCheckBox*>("delPlotButton");
    remPlotButton = window->findChild<QCheckBox*>("remPlotButton");
    velPlotButton = window->findChild<QCheckBox*>("velPlotButton");
    accPlotButton = window->findChild<QCheckBox*>("accPlotButton");
    devPlotButton = window->findChild<QCheckBox*>("devPlotButton");

    saveWaypoint(Waypoint({   0, 135, -45,  0,   0, 0 }, true));
    saveWaypoint(Waypoint({ -10, 150, -35, 25, -15, 0 }, true));

    wpEndField->setValue(1);

    doControlEnable();


    plotWidget = window->findChild<LineChartWidget*>("chartWidget");
    plotWidget->ylim(-180, 180);

    scene->print();

    scene->update();
    window->show();

    updateThread = std::make_unique<std::thread>([](){
        Timer rateTimer;
        double time = 0;
        bool inProcess = false;

        while (true) {
            double delta = rateTimer.getElapsedSeconds();
            scene->step(delta);
            rateTimer.reset();

            sceneWidget->update();

            if (robot->inTransit() || scene->getTurntable()->inTransit()) scene->update();

            if (robot->inTransit()) {

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
