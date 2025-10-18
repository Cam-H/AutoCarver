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


#include "geometry/poly/Profile.h"
#include "ViewerWidget.h"

#include "core/Timer.h"

#endif

class UiLoader : public QUiLoader {
public:

    UiLoader(QObject *parent = nullptr) : QUiLoader(parent) {}

    QWidget *createWidget(const QString& className, QWidget *parent = nullptr, const QString& name = QString())
    {
        if (className == "ViewerWidget") {
            auto *w = new ViewerWidget(parent);
            w->setObjectName(name);
            return w;
        }

        return QUiLoader::createWidget(className, parent, name);
    }
};

std::unique_ptr<std::thread> updateThread;

Profile profile;

QWidget *window = nullptr;
ViewerWidget *displayWidget = nullptr;

QComboBox *setSelectBox;
QDoubleSpinBox *tField;

bool sim = false;
double total = 0, t = 0;

void updateProfile(const std::string& filename)
{
    profile = Profile(filename);
    profile.prepare();

    displayWidget->setProfile(&profile);
}

double length(const glm::dvec3& a, const glm::dvec3& b)
{
    auto delta = b - a;
    delta.z *= 0.1;

    return glm::length(delta);
}

void planNext()
{
    if (!profile.planNext() && profile.nextSequenceAccessible()) {
        std::cout << "Approach retry:\n";
//        if (profile.developApproach(20000)) std::cout << "Developed fix!\n";
    }

    displayWidget->updatePath();

    const auto& path = displayWidget->path();
    if (!path.empty()) {
        displayWidget->updateTool();

        total = 0;
        for (uint32_t i = 0; i < path.size() - 1; i++) {
            total += length(path[i + 1], path[i]);
        }
        total = 1 / total;

        std::cout << "PATH:\n";
        for (const glm::dvec3& v : path) {
            std::cout << v.x << " " << v.y << " " << v.z << "\n";
        }
    }

    setSelectBox->clear();
    for (uint32_t i = 0; i < path.size(); i++) {
        setSelectBox->addItem(("P" + std::to_string(i)).c_str());
    }

    displayWidget->repaint();
}

void start()
{
    if (!displayWidget->path().empty()) {
        sim = true;
        t = 0;
    }
}

void stop()
{
    sim = false;
}

glm::dvec3 location()
{
    const auto& path = displayWidget->path();
    if (path.empty()) return {};

    double sum = 0;
    uint32_t idx = 0;

    while (idx < path.size() - 1) {
        double len = length(path[idx + 1], path[idx]);
        double dt = len * total;

        if (sum + dt > t) {
            double s = (t - sum) / dt;

            glm::dvec3 delta = (path[idx + 1] - path[idx]) / len;
            return path[idx] + s * len * delta;
        }

        sum += dt;
        idx++;
    }

    return path.back();
}

static QWidget *loadUiFile(QWidget *parent)
{
    QFile file("../tests/manual/soperator/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("viewer");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    displayWidget = window->findChild<ViewerWidget*>("displayWidget");
    displayWidget->enableFill(true);
    displayWidget->enableVertices(false);
    displayWidget->setProfile(&profile);

    auto refineButton = window->findChild<QPushButton*>("refineButton");
    QObject::connect(refineButton, &QPushButton::clicked, [&]() {
        stop();

        profile.refine();

        displayWidget->updatePolygons();
    });

    auto skipButton = window->findChild<QPushButton*>("skipButton");
    QObject::connect(skipButton, &QPushButton::clicked, [&]() {
        stop();

        profile.skip();

        displayWidget->updatePolygons();
    });

    auto planButton = window->findChild<QPushButton*>("planButton");
    QObject::connect(planButton, &QPushButton::clicked, [&]() {
        stop();

        planNext();
    });

    setSelectBox = window->findChild<QComboBox*>("setSelectBox");
    QObject::connect(setSelectBox, &QComboBox::currentIndexChanged, [&](int state) {
        stop();

        if (state < displayWidget->path().size()) {
            displayWidget->updateTool(displayWidget->path()[state]);
            displayWidget->update();
        }
    });

    tField = window->findChild<QDoubleSpinBox*>("tField");
    QObject::connect(tField, &QDoubleSpinBox::valueChanged, [](double value) {

    });

    auto simButton = window->findChild<QPushButton*>("simButton");
    QObject::connect(simButton, &QPushButton::clicked, [&]() {
        start();
    });

    auto centerButton = window->findChild<QPushButton*>("centerButton");
    displayWidget->setCentered(centerButton->isChecked());
    QObject::connect(centerButton, &QPushButton::clicked, [&]() {
        displayWidget->center();
    });

    auto printButton = window->findChild<QPushButton*>("printButton");
    QObject::connect(printButton, &QPushButton::clicked, [&]() {
        profile.print();
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

    auto samplerBox = window->findChild<QComboBox*>("samplerBox");
    QObject::connect(samplerBox, &QComboBox::currentIndexChanged, [&](int state) {
//       switch (state) {
//           case 0: planner.setSampler(RRTPlanner::SampleSpace::BOX); break;
//           case 1: planner.setSampler(RRTPlanner::SampleSpace::TRIANGLE); break;
//           case 2: planner.setSampler(RRTPlanner::SampleSpace::CIRCLE); break;
//           default: std::cout << "Unrecognized sampler\n";
//       }
//
//        displayWidget->update();
    });

    auto setButton = window->findChild<QPushButton*>("setButton");
    QObject::connect(setButton, &QPushButton::clicked, [&]() {
//        int sel = displayWidget->selection();
//        if (sel != -1) {
//            switch (planner.getSampler()) {
//                case RRTPlanner::SampleSpace::BOX:
//                {
//                    auto [xMin, xMax] = config.polygons[sel].xExtents();
//                    auto [yMin, yMax] = config.polygons[sel].yExtents();
//                    planner.setBounds({ xMin, yMin }, { xMax, yMax });
//                }
//                    break;
//                case RRTPlanner::SampleSpace::TRIANGLE:
//                {
//                    if (config.polygons[sel].vertexCount() == 3) {
//                        Triangle2D tri(config.polygons[sel].border()[0], config.polygons[sel].border()[1], config.polygons[sel].border()[2]);
//                        planner.setBounds(tri);
//                    }
//                }
//                    break;
//                case RRTPlanner::SampleSpace::CIRCLE:
//                    planner.setBounds(config.polygons[sel].bounds());
//                    break;
//            }
//
//            displayWidget->update();
//        }
    });

    auto restoreButton = window->findChild<QPushButton*>("restoreButton");
    QObject::connect(restoreButton, &QPushButton::clicked, [&]() {
//        planner.setSampler(RRTPlanner::SampleSpace::BOX);
//        updateBounds();
//
//        displayWidget->update();
    });


    auto loadProfileButton = window->findChild<QPushButton*>("loadProfileButton");
    QObject::connect(loadProfileButton, &QPushButton::clicked, [&]() {
        const QString selectedFile = QFileDialog::getOpenFileName(nullptr, "Select Profile",
                                                                  "../out", "Binary Files (*.bin)");

        if (!selectedFile.isEmpty()) {
            updateProfile(selectedFile.toStdString());
        }
    });


    updateProfile("../out/APROF0.bin");

    // Handle updating robot
    updateThread = std::make_unique<std::thread>([&](){
        Timer rateTimer;

        while (true) {
            if (sim) {
                if (t < 1.0) {
                    displayWidget->updateTool(location());
                } else {
                    displayWidget->updateTool(displayWidget->path().back());
                    stop();
                }

                t += rateTimer.getElapsedSeconds() * total / 5.0;
                displayWidget->update();
            }

            rateTimer.reset();

            std::this_thread::sleep_for(std::chrono::nanoseconds(100000000));
        }
    });

    window->show();

    return app.exec();
}
