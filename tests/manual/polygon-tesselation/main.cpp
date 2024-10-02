//
// Created by cameronh on 02/06/24.
//

#include <QApplication>
#include <QClipboard>

#include <QVBoxLayout>
#include <QHBoxLayout>

#include <QPushButton>
#include <QCheckBox>

#include <iostream>
#include "window.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    // Test data set
    uint32_t idx = 0;
    std::vector<Polygon> polygons;

    // Square
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(100, 400), QVector2D(400, 400), QVector2D(400, 100)}
    });

    // Square w/ diamond
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(100, 400), QVector2D(400, 400), QVector2D(400, 100)},
            std::vector<QVector2D>{QVector2D(250, 150), QVector2D(350, 250), QVector2D(250, 350), QVector2D(150, 250)}
    });

    // Multi-hole square
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(100, 400), QVector2D(400, 400), QVector2D(400, 100)},
            std::vector<QVector2D>{QVector2D(250, 150), QVector2D(350, 230), QVector2D(250, 210), QVector2D(150, 230)},
            std::vector<QVector2D>{QVector2D(250, 350), QVector2D(150, 270), QVector2D(250, 290), QVector2D(350, 270)}
    });

    // Circle hammer
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(28.000000, 235.000000), QVector2D(100.000000, 400.000000), QVector2D(261.000000, 467.000000), QVector2D(401.000000, 403.000000), QVector2D(476.000000, 253.000000), QVector2D(481.000000, 53.000000), QVector2D(335.000000, 136.000000), QVector2D(335.000000, 105.000000), QVector2D(302.000000, 141.000000), QVector2D(351.000000, 237.000000), QVector2D(383.000000, 190.000000), QVector2D(352.000000, 154.000000), QVector2D(411.000000, 124.000000), QVector2D(407.000000, 292.000000), QVector2D(271.000000, 215.000000), QVector2D(301.000000, 85.000000), QVector2D(249.000000, 36.000000), QVector2D(110.000000, 88.000000)}
    });

    // Circle
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(28.000000, 235.000000), QVector2D(100.000000, 400.000000), QVector2D(258.000000, 470.000000), QVector2D(401.000000, 403.000000), QVector2D(476.000000, 253.000000), QVector2D(403.000000, 99.000000), QVector2D(249.000000, 36.000000), QVector2D(110.000000, 88.000000)}
    });

    // Zap
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100.000000, 400.000000), QVector2D(161.000000, 257.000000), QVector2D(221.000000, 452.000000), QVector2D(319.000000, 331.000000), QVector2D(476.000000, 423.000000), QVector2D(466.000000, 92.000000), QVector2D(274.000000, 337.000000), QVector2D(338.000000, 81.000000), QVector2D(254.000000, 266.000000), QVector2D(100.000000, 100.000000)}
    });

    // Reverse E
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(365.000000, 118.000000), QVector2D(346.000000, 223.000000), QVector2D(265.000000, 129.000000), QVector2D(253.000000, 178.000000), QVector2D(328.000000, 329.000000), QVector2D(240.000000, 303.000000), QVector2D(191.000000, 165.000000), QVector2D(171.000000, 319.000000), QVector2D(400.000000, 400.000000), QVector2D(482.000000, 227.000000), QVector2D(390.000000, 166.000000), QVector2D(446.000000, 96.000000), QVector2D(402.000000, 72.000000), QVector2D(100.000000, 100.000000)}
    });

    // Seahorse
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(230.000000, 382.000000), QVector2D(167.000000, 263.000000), QVector2D(100.000000, 400.000000), QVector2D(258.000000, 470.000000), QVector2D(401.000000, 403.000000), QVector2D(476.000000, 253.000000), QVector2D(403.000000, 99.000000), QVector2D(479.000000, 141.000000), QVector2D(466.000000, 61.000000), QVector2D(269.000000, 75.000000), QVector2D(273.000000, 188.000000), QVector2D(178.000000, 129.000000)}
    });

    // Sigil
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(309.000000, 402.000000), QVector2D(248.000000, 377.000000), QVector2D(145.000000, 94.000000), QVector2D(147.000000, 326.000000), QVector2D(63.000000, 88.000000), QVector2D(79.000000, 479.000000), QVector2D(104.000000, 394.000000), QVector2D(125.000000, 479.000000), QVector2D(147.000000, 387.000000), QVector2D(184.000000, 476.000000), QVector2D(258.000000, 470.000000), QVector2D(401.000000, 403.000000), QVector2D(462.000000, 261.000000), QVector2D(484.000000, 295.000000), QVector2D(493.000000, 170.000000), QVector2D(415.000000, 124.000000), QVector2D(469.000000, 187.000000), QVector2D(430.000000, 271.000000), QVector2D(430.000000, 186.000000), QVector2D(402.000000, 270.000000), QVector2D(412.000000, 147.000000), QVector2D(377.000000, 201.000000), QVector2D(364.000000, 259.000000), QVector2D(343.000000, 172.000000), QVector2D(398.000000, 107.000000), QVector2D(477.000000, 96.000000), QVector2D(466.000000, 61.000000), QVector2D(310.000000, 28.000000), QVector2D(326.000000, 303.000000), QVector2D(277.000000, 191.000000), QVector2D(178.000000, 129.000000)}
    });

    // Twist
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(158.000000, 272.000000), QVector2D(316.000000, 294.000000), QVector2D(324.000000, 196.000000), QVector2D(213.000000, 174.000000), QVector2D(206.000000, 252.000000), QVector2D(289.000000, 267.000000), QVector2D(287.000000, 219.000000), QVector2D(250.000000, 210.000000), QVector2D(243.000000, 230.000000), QVector2D(271.000000, 231.000000), QVector2D(272.000000, 250.000000), QVector2D(225.000000, 242.000000), QVector2D(230.000000, 193.000000), QVector2D(307.000000, 212.000000), QVector2D(296.000000, 279.000000), QVector2D(189.000000, 262.000000), QVector2D(198.000000, 155.000000), QVector2D(348.000000, 162.000000), QVector2D(330.000000, 306.000000), QVector2D(170.000000, 295.000000), QVector2D(100.000000, 400.000000), QVector2D(258.000000, 470.000000), QVector2D(401.000000, 403.000000), QVector2D(476.000000, 253.000000), QVector2D(403.000000, 99.000000), QVector2D(479.000000, 141.000000), QVector2D(466.000000, 61.000000), QVector2D(269.000000, 75.000000), QVector2D(316.000000, 117.000000), QVector2D(178.000000, 129.000000)}
    });

//    std::vector<QVector2D> temp = std::vector<QVector2D>;
//    polygons.emplace_back(std::vector<QVector2D>(temp.rbegin(), temp.rend()));

    QWidget *container = new QWidget;

    QVBoxLayout *vLayout = new QVBoxLayout(container);
//    vLayout->setAlignment(Qt::AlignTop);

    Window *window = new Window;
    window->setPolygon(&polygons[0]);
    window->setFixedSize(500, 500);
    vLayout->addWidget(window);

    QHBoxLayout *hLayout = new QHBoxLayout;
    vLayout->addLayout(hLayout);

    auto prevButton = new QPushButton("<", container);
    prevButton->setMaximumWidth(50);
    hLayout->addWidget(prevButton);

    auto copyButton = new QPushButton("copy test", container);
    hLayout->addWidget(copyButton);

    auto nextButton = new QPushButton(">", container);
    nextButton->setMaximumWidth(50);
    hLayout->addWidget(nextButton);

    QObject::connect(prevButton, &QPushButton::clicked, [&]() {
        if (idx > 0) idx--;
        window->setPolygon(&polygons[idx]);
    });

    // Scuffed
    QClipboard *clipboard = QGuiApplication::clipboard();
    QObject::connect(copyButton, &QPushButton::clicked, [=]() {
        Polygon *poly = window->getPolygon();

        std::string text = "polygons.emplace_back(std::vector<std::vector<QVector2D>>{\n";
        for (uint32_t i = 0; i < poly->loopCount(); i++) {
            std::vector<uint32_t> loop = poly->getLoop(i);

            text = text + "std::vector<QVector2D>{";
            for (uint32_t j = 0; j < loop.size(); j++) {
                text = text + "QVector2D(" + std::to_string(poly->getVertex(loop[j]).x()) + ", " + std::to_string(poly->getVertex(loop[j]).y()) + ")";
                if (j + 1 < loop.size()) text = text + ", ";
            }
            text = text + "}";
            if (i + 1 < poly->loopCount()) text = text + ", ";
            text = text + "\n";

        }

        text = text + "});";
        clipboard->setText(QString(text.c_str()));
    });

    QObject::connect(nextButton, &QPushButton::clicked, [&]() {
        if (idx < polygons.size() - 1) idx++;
        window->setPolygon(&polygons[idx]);
    });

    auto polygonButton = new QCheckBox("Polygon", container);
    polygonButton->setChecked(true);
    vLayout->addWidget(polygonButton);

    auto partitionButton = new QCheckBox("Partition", container);
    vLayout->addWidget(partitionButton);

    auto tesselateButton = new QCheckBox("Tesselate", container);
    vLayout->addWidget(tesselateButton);

    QObject::connect(polygonButton, &QCheckBox::clicked, [&](bool checked) {
        window->enablePolygon(checked);
    });

    QObject::connect(partitionButton, &QCheckBox::clicked, [&](bool checked) {
        window->enablePartition(checked);
//        tesselateButton->setChecked(tesselateButton->isChecked() && checked);
//        tesselateButton->setEnabled(checked);
    });

    QObject::connect(tesselateButton, &QCheckBox::clicked, [&](bool checked) {
        window->enableTesselation(checked);
    });

    container->show();

    return app.exec();
}
