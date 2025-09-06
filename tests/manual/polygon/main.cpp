#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>
#include <QClipboard>

#include <QPushButton>
#include <QCheckBox>

#include <QFile>
#include <QDir>

#include <iostream>
#include <QVBoxLayout>

#ifndef QT_NO_OPENGL

#include "geometry/poly/Polygon.h"
#include "geometry/poly/Profile.h"
#include "renderer/PolygonWidget.h"
#include "renderer/UiLoader.h"
#include "fileIO/Serializable.h"

#endif


QWidget *window = nullptr;

PolygonWidget *displayWidget = nullptr;
QLabel *countLabel = nullptr;
QCheckBox *centerButton = nullptr;

std::vector<Polygon*> polygons;
uint32_t selection = 0;


static QWidget *loadUiFile(QWidget *parent)
{
    QFile file("../tests/manual/polygon/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

void updateLabel()
{
    countLabel->setText(("Polygon " + std::to_string(selection + 1) + "/" + std::to_string(polygons.size())).c_str());
}

template<class T>
void load(const QStringList& selectedFiles) {
    if (selectedFiles.empty()) return;

    uint32_t idx = polygons.size();

    for (const QString& filename : selectedFiles) {
        auto poly = new T(filename.toStdString());

        polygons.push_back(poly);
    }

    centerButton->setChecked(true);
    displayWidget->setCentered(true);

    selection = idx;
    displayWidget->setPolygon(polygons[selection]);
    updateLabel();

}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

//    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n"
//        << QDir::homePath().toStdString() << "\n"
//        << QDir::rootPath().toStdString() << "\n";
//    "../tests/manual/polygon/main.ui"
//    QDir::path
    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    window = loadUiFile(nullptr);
    if (window == nullptr) return -1;

    displayWidget = window->findChild<PolygonWidget*>("displayWidget");

    polygons.emplace_back(new Polygon(std::vector<glm::dvec2>{
            { 213.000000, 156.000000},{ 205.000000, 276.000000},{ 400.000000, 100.000000},{ 420.000000, 189.000000},{ 314.000000, 252.000000},{ 342.000000, 400.000000},{ 265.000000, 304.000000},{ 114.000000, 422.000000},{ 95.000000, 402.000000},{ 158.000000, 349.000000},{ 24.000000, 196.000000},{ 108.000000, 246.000000},{ 103.000000, 34.000000},{ 119.000000, 171.000000},{ 197.000000, 96.000000},{ 240.000000, 23.000000}
    }));

    polygons.emplace_back(new Polygon(std::vector<glm::dvec2>{
            { 100.000000, 100.000000},{ 445.000000, 70.000000},{ 216.000000, 230.000000},{ 238.000000, 150.000000},{ 184.000000, 143.000000},{ 144.000000, 252.000000},{ 203.000000, 341.000000},{ 400.000000, 400.000000},{ 95.000000, 402.000000}
    }));

    polygons.emplace_back(new Polygon(std::vector<glm::dvec2>{
            { 100.000000, 100.000000},{ 400.000000, 100.000000},{ 400.000000, 400.000000},{ 95.000000, 402.000000}
    }));


    polygons.emplace_back(new Polygon(std::vector<glm::dvec2>{
            glm::vec2(100.0f, 100.0f),
            glm::vec2(400.0f, 100.0f),
            glm::vec2(400.0f, 400.0f),
            glm::vec2(114.0f, 422.0f),
            glm::vec2(95.0f, 402.0f),
            glm::vec2(68.0f, 358.0f),
            glm::vec2(32.0f, 279.0f),
            glm::vec2(28.0f, 185.0f),
            glm::vec2(63.0f, 62.0f)
    }));

    auto test = std::vector<glm::dvec2>{
            {341.096,42.1527},
            {345.347,42.4847},
            {351.444,46.3256},
            {391.774,127.265},
            {411.225,175.035},
            {411.699,203.445},
            {349,457},
            {334.986,457.153},
            {281.097,457.153},
            {227.85,456.939},
            {184.927,456.656},
            {183.151,456.611},
            {81.9157,340.89},
            {154.341,142.561},
            {159.765,132.142},
            {162.207,128.436},
            {174.76,113.482},
            {230.888,47.0066},
            {240.784,42.2321},
            {286.848,42.1527}
    };

//    Polygon::cullCollinear(test);
    polygons.emplace_back(new Polygon(test));

    countLabel = window->findChild<QLabel*>("countLabel");
    updateLabel();

    auto polygonCheck = window->findChild<QCheckBox*>("polygonCheck");
    QObject::connect(polygonCheck, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->enablePolygon(state == Qt::CheckState::Checked);
    });

    auto triangulationCheck = window->findChild<QCheckBox*>("triangulationCheck");
    QObject::connect(triangulationCheck, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->enableTriangulation(state == Qt::CheckState::Checked);
    });

    auto hullCheck = window->findChild<QCheckBox*>("hullCheck");
    QObject::connect(hullCheck, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->enableHull(state == Qt::CheckState::Checked);
    });

    auto prevButton = window->findChild<QPushButton*>("prevButton");
    QObject::connect(prevButton, &QPushButton::clicked, [&]() {
        if (selection > 0) selection--;
        displayWidget->setPolygon(polygons[selection]);
        updateLabel();
    });

    auto nextButton = window->findChild<QPushButton*>("nextButton");
    QObject::connect(nextButton, &QPushButton::clicked, [&]() {
        if (selection < polygons.size() - 1) selection++;
        displayWidget->setPolygon(polygons[selection]);
        updateLabel();
    });

    auto downScaleButton = window->findChild<QPushButton*>("downScaleButton");
    QObject::connect(downScaleButton, &QPushButton::clicked, [&]() {
        polygons[selection]->scale({ 250, 250}, 0.95f);
        displayWidget->update();
    });

    centerButton = window->findChild<QCheckBox*>("centerButton");
    QObject::connect(centerButton, &QCheckBox::stateChanged, [&](int state) {
        displayWidget->setCentered(state == Qt::CheckState::Checked);
    });

    auto upScaleButton = window->findChild<QPushButton*>("upScaleButton");
    QObject::connect(upScaleButton, &QPushButton::clicked, [&]() {
        polygons[selection]->scale({ 250, 250}, 1.05f);
        displayWidget->update();
    });

    auto copyButton = window->findChild<QPushButton*>("copyButton");
    QObject::connect(copyButton, &QPushButton::clicked, [=]() {
        Polygon *poly = displayWidget->getPolygon();

        std::string text = "polygons.emplace_back(std::vector<glm::vec2>{\n";
        for (uint32_t i = 0; i < poly->vertexCount(); i++) {

            text = text + "{ " + std::to_string(poly->border()[i].x) + ", " + std::to_string(poly->border()[i].y);
            if (i < poly->vertexCount() - 1) text = text + "},";
        }

        text = text + "}\n});";
        QGuiApplication::clipboard()->setText(QString(text.c_str()));
    });

    auto saveButton = window->findChild<QPushButton*>("saveButton");
    QObject::connect(saveButton, &QPushButton::clicked, [=]() {
        std::string source = "../out/polygon.bin";
        polygons[selection]->save(source);
    });

    auto loadPolygonButton = window->findChild<QPushButton*>("loadPolygonButton");
    QObject::connect(loadPolygonButton, &QPushButton::clicked, [=]() {
        const QStringList selectedFiles = QFileDialog::getOpenFileNames(nullptr, "Select Polygon",
                                                              "../out", "Binary Files (*.bin)");
        load<Polygon>(selectedFiles);
    });

    auto loadProfileButton = window->findChild<QPushButton*>("loadProfileButton");
    QObject::connect(loadProfileButton, &QPushButton::clicked, [=]() {
        const QStringList selectedFiles = QFileDialog::getOpenFileNames(nullptr, "Select Profile",
                                                                        "../out", "Binary Files (*.bin)");
        load<Profile>(selectedFiles);
    });

    displayWidget->setPolygon(polygons[selection]);

    window->show();

    return app.exec();
}
