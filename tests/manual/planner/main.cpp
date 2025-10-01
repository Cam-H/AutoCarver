#include <QApplication>
#include <QSurfaceFormat>

#include <QPushButton>
#include <QCheckBox>

#include <QFile>

#include <QtWidgets>
#include <QUiLoader>

#include <iostream>
#include <thread>

#ifndef QT_NO_OPENGL

#include "robot/trajectory/Waypoint.h"
#include "robot/trajectory/TOPPTrajectory.h"

#include "geometry/poly/Polygon.h"
#include "PlannerWidget.h"

#include "Configuration.h"
#include "RRTPlanner.h"
#include "core/Timer.h"

#endif

class UiLoader : public QUiLoader {
public:

    UiLoader(QObject *parent = nullptr) : QUiLoader(parent) {}

    QWidget *createWidget(const QString& className, QWidget *parent = nullptr, const QString& name = QString())
    {
        if (className == "PlannerWidget") {
            auto *w = new PlannerWidget(parent);
            w->setObjectName(name);
            return w;
        }

        return QUiLoader::createWidget(className, parent, name);
    }
};

std::unique_ptr<std::thread> updateThread;

Configuration config;
RRTPlanner planner;

QWidget *window = nullptr;
PlannerWidget *displayWidget = nullptr;

QDoubleSpinBox *widthField, *heightField, *timeScalarField;
QDoubleSpinBox *startXField, *startYField, *startAngleField;
QDoubleSpinBox *goalXField, *goalYField, *goalAngleField;

QSpinBox *countField, *rateField;
QPushButton *randomButton, *iterateButton, *solveButton, *followButton;

QCheckBox *autoButton;

QLineEdit *outField;

TOPPTrajectory* traj = nullptr;
bool sim = false, delReq = false;

static QWidget *loadUiFile(QWidget *parent)
{
    QFile file("../tests/manual/planner/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void rmTraj()
{
    delete traj;
    traj = nullptr;
}

void updateDisplay()
{
    displayWidget->clearPolygons();
    for (Polygon& poly : config.polygons) displayWidget->addPolygon(&poly);

    displayWidget->setStart(config.start, config.width, config.height);
    displayWidget->setGoal(config.goal, config.width, config.height);

    planner.updateBounds(config.polygons);

    planner.setStart(config.start);
    planner.setGoal(config.goal);

    if (displayWidget->isCentered()) displayWidget->center(); // addPolygon does not automatically recalculate offset

    displayWidget->update();
}

void updateStartFields()
{
    startXField->setValue(config.start.x);
    startYField->setValue(config.start.y);
    startAngleField->setValue(config.start.z);
}

void updateGoalFields()
{
    goalXField->setValue(config.goal.x);
    goalYField->setValue(config.goal.y);
    goalAngleField->setValue(config.goal.z);
}

void updateFields()
{
    widthField->setValue(config.width);
    heightField->setValue(config.height);

    updateStartFields();
    updateGoalFields();
}

void updatePath()
{
    traj = new TOPPTrajectory(planner.waypoints(config.polygons), std::vector<double>{ 4.0, 4.0, 4.0 }, std::vector<double>{ 1.5, 1.5, 1.5 });
    displayWidget->setPath(PiecewisePolyPath(planner.waypoints(config.polygons)));
}

void updateStart()
{
    rmTraj();

    updateStartFields();

    planner.setStart(config.start);
    displayWidget->updateStart(config.start, config.width, config.height);
    displayWidget->update();
}

void updateGoal()
{
    rmTraj();

    updateGoalFields();
    displayWidget->updateGoal(config.goal, config.width, config.height);

    planner.setGoal(config.goal);
    if (planner.solutionImproved()) {
        updatePath();

        planner.clearUpdateFlag();
    }

    displayWidget->update();
}

void updateDims()
{
    widthField->setValue(config.width);
    heightField->setValue(config.height);

    planner.setTargetDimensions(config.width, config.height);

    displayWidget->updateStart(config.start, config.width, config.height);
    displayWidget->updateGoal(config.goal, config.width, config.height);
    displayWidget->update();
}

void travel(const glm::dvec3& position)
{
    glm::dvec3 offset = position - config.start;
    config.start = position;

//    updateStartFields();

    displayWidget->offsetStart(offset);
    displayWidget->update();
}

void load(const std::string& filename)
{
    config.load(filename);
    updateDisplay();
    updateFields();
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("planner");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    displayWidget = window->findChild<PlannerWidget*>("displayWidget");
    displayWidget->enableFill(true);
    displayWidget->enableVertices(false);
    displayWidget->setPlanner(&planner);

    QObject::connect(displayWidget, &PlannerWidget::startMoved, [](const glm::dvec2& delta) {
        config.start += glm::dvec3(delta.x, delta.y, 0.0);
        updateStart();
    });

    QObject::connect(displayWidget, &PlannerWidget::goalMoved, [](const glm::dvec2& delta) {
        config.goal += glm::dvec3(delta.x, delta.y, 0.0);
        updateGoal();
    });

    widthField = window->findChild<QDoubleSpinBox*>("widthField");
    QObject::connect(widthField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.width = value;
        updateDims();
    });

    timeScalarField = window->findChild<QDoubleSpinBox*>("timeScalarField");

    auto tField = window->findChild<QDoubleSpinBox*>("tField");
    QObject::connect(tField, &QDoubleSpinBox::valueChanged, [](double value) {
        if (planner.solved()) {
            if (sim || traj == nullptr) {
                updatePath();
            }

            auto wp = traj->evaluate(value);
            travel({ wp.values[0], wp.values[1], wp.values[2] });

        }
    });

    heightField = window->findChild<QDoubleSpinBox*>("heightField");
    QObject::connect(heightField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.height = value;
        updateDims();
    });

    startXField = window->findChild<QDoubleSpinBox*>("startXField");
    QObject::connect(startXField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.start.x = value;
        updateStart();
    });

    startYField = window->findChild<QDoubleSpinBox*>("startYField");
    QObject::connect(startYField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.start.y = value;
        updateStart();
    });

    startAngleField = window->findChild<QDoubleSpinBox*>("startAngleField");
    QObject::connect(startAngleField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.start.z = value;
        updateStart();
    });


    goalXField = window->findChild<QDoubleSpinBox*>("goalXField");
    QObject::connect(goalXField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.goal.x = value;
        updateGoal();
    });

    goalYField = window->findChild<QDoubleSpinBox*>("goalYField");
    QObject::connect(goalYField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.goal.y = value;
        updateGoal();
    });

    goalAngleField = window->findChild<QDoubleSpinBox*>("goalAngleField");
    QObject::connect(goalAngleField, &QDoubleSpinBox::valueChanged, [](double value) {
        config.goal.z = value;
        updateGoal();
    });

    countField = window->findChild<QSpinBox*>("countField");

    randomButton = window->findChild<QPushButton*>("randomButton");
    QObject::connect(randomButton, &QPushButton::clicked, [&]() {
        config.randomize(countField->value());
        updateDisplay();
    });

    iterateButton = window->findChild<QPushButton*>("iterateButton");
    QObject::connect(iterateButton, &QPushButton::clicked, [&]() {
        {
            ScopedTimer timer("[RRTPlanner] Iteration");
            planner.iterate(config.polygons);
        }
        displayWidget->update();
    });

    solveButton = window->findChild<QPushButton*>("solveButton");
    QObject::connect(solveButton, &QPushButton::clicked, [&]() {
        planner.solve(config.polygons, 5000);
        displayWidget->update();

        if (planner.solved()) {
            rmTraj();
            updatePath();
        }
    });

    auto updatePathButton = window->findChild<QPushButton*>("updatePathButton");
    QObject::connect(followButton, &QPushButton::clicked, [&]() {
        if (planner.solved()) {
            updatePath();
        }
    });

    followButton = window->findChild<QPushButton*>("followButton");
    QObject::connect(followButton, &QPushButton::clicked, [&]() {
        if (planner.solved()) {
            delete traj;

            traj = new TOPPTrajectory(planner.waypoints(), std::vector<double>{ 4.0, 4.0, 4.0 }, std::vector<double>{ 1.5, 1.5, 1.5 });
            sim = true;

            if (!traj->isValid()) {
                std::cout << "\033[93mFailed to prepare trajectory\033[0m\n";
                rmTraj();
            }
        }
    });

    auto resetButton = window->findChild<QPushButton*>("resetButton");
    QObject::connect(resetButton, &QPushButton::clicked, [&]() {
        if (!sim) rmTraj();
        else delReq = true;

        planner.reset();
        displayWidget->update();
    });

    auto centerButton = window->findChild<QCheckBox*>("centerButton");
    displayWidget->setCentered(centerButton->isChecked());
    QObject::connect(centerButton, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->setCentered(state == Qt::CheckState::Checked);
    });

    auto showBoundsButton = window->findChild<QCheckBox*>("showBoundsButton");
    displayWidget->enableBounds(showBoundsButton->isChecked());
    QObject::connect(showBoundsButton, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->enableBounds(state == Qt::CheckState::Checked);
        displayWidget->update();
    });

    auto showTreeButton = window->findChild<QCheckBox*>("showTreeButton");
    displayWidget->enableTree(showTreeButton->isChecked());
    QObject::connect(showTreeButton, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->enableTree(state == Qt::CheckState::Checked);
        displayWidget->update();
    });

    auto showPathButton = window->findChild<QCheckBox*>("showPathButton");
    displayWidget->enablePath(showPathButton->isChecked());
    QObject::connect(showPathButton, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->enablePath(state == Qt::CheckState::Checked);
        displayWidget->update();
    });

    rateField = window->findChild<QSpinBox*>("rateField");
    autoButton = window->findChild<QCheckBox*>("autoButton");

    outField = window->findChild<QLineEdit*>("outField");

    auto saveButton = window->findChild<QPushButton*>("saveButton");
    QObject::connect(saveButton, &QPushButton::clicked, [&]() {
        config.save("../out/" + outField->text().toStdString() + ".bin");
    });

    auto loadButton = window->findChild<QPushButton*>("loadButton");
    QObject::connect(loadButton, &QPushButton::clicked, [&]() {
        const QString selectedFile = QFileDialog::getOpenFileName(nullptr, "Select Configuration",
                                                                        "../out", "Binary Files (*.bin)");
        if (!selectedFile.isEmpty()) {
            load(selectedFile.toStdString());
        }
    });

    load("../out/PlanEnv0.bin");

    // Handle updating robot
    updateThread = std::make_unique<std::thread>([](){
        Timer rateTimer;

        while (true) {
            if (sim && traj != nullptr) {
                auto wp = traj->timestep(timeScalarField->value() * rateTimer.getElapsedSeconds());
                travel({ wp.values[0], wp.values[1], wp.values[2] });

                if (delReq || traj->complete()) {
                    delReq = false;
                    rmTraj();
                }
            } else if (autoButton->isChecked()) {
                for (uint32_t i = 0; i < rateField->value(); i++) planner.iterate(config.polygons);

                if (planner.solutionImproved()) {
                    std::cout << "Superior solution discovered!\n";
                    updatePath();

                    planner.clearUpdateFlag();
                }

                displayWidget->update();

//                planner.print();
            }

            rateTimer.reset();

            std::this_thread::sleep_for(std::chrono::nanoseconds(100000000));
        }
    });

    window->show();

    return app.exec();
}
