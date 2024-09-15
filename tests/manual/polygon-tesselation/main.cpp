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
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100.000000, 100.000000), QVector2D(400.000000, 100.000000), QVector2D(400.000000, 400.000000), QVector2D(114.000000, 422.000000), QVector2D(95.000000, 402.000000), QVector2D(68.000000, 358.000000), QVector2D(32.000000, 279.000000), QVector2D(28.000000, 185.000000), QVector2D(63.000000, 62.000000)},
            std::vector<QVector2D>{QVector2D(250.000000, 150.000000), QVector2D(150.000000, 250.000000), QVector2D(250.000000, 350.000000), QVector2D(350.000000, 250.000000)}
    });

    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(400, 100), QVector2D(400, 400), QVector2D(100, 400)},
            std::vector<QVector2D>{QVector2D(250, 150), QVector2D(150, 250), QVector2D(250, 350), QVector2D(350, 250)}
    });

    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(400, 100), QVector2D(400, 400), QVector2D(100, 400)},
            std::vector<QVector2D>{QVector2D(250, 150), QVector2D(150, 230), QVector2D(250, 210), QVector2D(350, 230)},
            std::vector<QVector2D>{QVector2D(250, 350), QVector2D(350, 270), QVector2D(250, 290), QVector2D(150, 270)}
    });

    polygons.emplace_back(std::vector<QVector2D>{QVector2D(110.000000, 88.000000), QVector2D(249.000000, 36.000000), QVector2D(301.000000, 85.000000), QVector2D(271.000000, 215.000000), QVector2D(407.000000, 292.000000), QVector2D(411.000000, 124.000000), QVector2D(352.000000, 154.000000), QVector2D(383.000000, 190.000000), QVector2D(351.000000, 237.000000), QVector2D(302.000000, 141.000000), QVector2D(335.000000, 105.000000), QVector2D(335.000000, 136.000000), QVector2D(481.000000, 53.000000), QVector2D(476.000000, 253.000000), QVector2D(401.000000, 403.000000), QVector2D(258.000000, 470.000000), QVector2D(100.000000, 400.000000), QVector2D(28.000000, 235.000000)});    polygons.emplace_back(std::vector<QVector2D>{QVector2D(110, 88), QVector2D(249, 36), QVector2D(403, 99), QVector2D(476, 253), QVector2D(401, 403), QVector2D(258, 470), QVector2D(100, 400), QVector2D(28, 235)});
    polygons.emplace_back(std::vector<QVector2D>{QVector2D(100, 100), QVector2D(254, 266), QVector2D(338, 81), QVector2D(274, 337), QVector2D(466, 92), QVector2D(476, 423), QVector2D(319, 331), QVector2D(221, 452), QVector2D(161, 257), QVector2D(100, 400)});
    polygons.emplace_back(std::vector<QVector2D>{QVector2D(100, 100), QVector2D(402, 72), QVector2D(446, 96), QVector2D(390, 166), QVector2D(482, 227), QVector2D(400, 400), QVector2D(171, 319), QVector2D(191, 165), QVector2D(240, 303), QVector2D(328, 329), QVector2D(253, 178), QVector2D(265, 129), QVector2D(346, 223), QVector2D(365, 118)});
    polygons.emplace_back(std::vector<QVector2D>{QVector2D(178, 129), QVector2D(273, 188), QVector2D(269, 75), QVector2D(466, 61), QVector2D(479, 141), QVector2D(403, 99), QVector2D(476, 253), QVector2D(401, 403), QVector2D(258, 470), QVector2D(100, 400), QVector2D(167, 263), QVector2D(230, 382)});
    polygons.emplace_back(std::vector<QVector2D>{QVector2D(178, 129), QVector2D(277, 191), QVector2D(326, 303), QVector2D(310, 28), QVector2D(466, 61), QVector2D(477, 96), QVector2D(398, 107), QVector2D(343, 172), QVector2D(364, 259), QVector2D(377, 201), QVector2D(412, 147), QVector2D(402, 270), QVector2D(430, 186), QVector2D(430, 271), QVector2D(469, 187), QVector2D(415, 124), QVector2D(493, 170), QVector2D(484, 295), QVector2D(462, 261), QVector2D(401, 403), QVector2D(258, 470), QVector2D(184, 476), QVector2D(147, 387), QVector2D(125, 479), QVector2D(104, 394), QVector2D(79, 479), QVector2D(63, 88), QVector2D(147, 326), QVector2D(145, 94), QVector2D(248, 377), QVector2D(309, 402)});
    polygons.emplace_back(std::vector<QVector2D>{QVector2D(178, 129), QVector2D(316, 117), QVector2D(269, 75), QVector2D(466, 61), QVector2D(479, 141), QVector2D(403, 99), QVector2D(476, 253), QVector2D(401, 403), QVector2D(258, 470), QVector2D(100, 400), QVector2D(170, 295), QVector2D(330, 306), QVector2D(348, 162), QVector2D(198, 155), QVector2D(189, 262), QVector2D(296, 279), QVector2D(307, 212), QVector2D(230, 193), QVector2D(225, 242), QVector2D(272, 250), QVector2D(271, 231), QVector2D(243, 230), QVector2D(250, 210), QVector2D(287, 219), QVector2D(289, 267), QVector2D(206, 252), QVector2D(213, 174), QVector2D(324, 196), QVector2D(316, 294), QVector2D(158, 272)});

//    std::vector<std::vector<QPoint>> borders;
////    borders.emplace_back(std::vector{QPoint(269, 75), QPoint(466, 61), QPoint(479, 141), QPoint(403, 99)});
////    borders.emplace_back(std::vector{QPoint(178, 129), QPoint(273, 188), QPoint(476, 253), QPoint(401, 403), QPoint(258, 470), QPoint(100, 400), QPoint(230, 382)});
//    borders.emplace_back(std::vector{QPoint(100, 100), QPoint(400, 100), QPoint(400, 400), QPoint(100, 400)});
//    borders.emplace_back(std::vector{QPoint(100, 100), QPoint(254, 266), QPoint(338, 81), QPoint(274, 337), QPoint(466, 92), QPoint(476, 423), QPoint(319, 331), QPoint(221, 452), QPoint(161, 257), QPoint(100, 400)});    borders.emplace_back(std::vector{QPoint(110, 88), QPoint(249, 36), QPoint(403, 99), QPoint(476, 253), QPoint(401, 403), QPoint(258, 470), QPoint(100, 400), QPoint(28, 235)});
//    borders.emplace_back(std::vector{QPoint(100, 100), QPoint(402, 72), QPoint(446, 96), QPoint(390, 166), QPoint(482, 227), QPoint(400, 400), QPoint(171, 319), QPoint(191, 165), QPoint(240, 303), QPoint(328, 329), QPoint(253, 178), QPoint(265, 129), QPoint(346, 223), QPoint(365, 118)});    borders.emplace_back(std::vector{QPoint(178, 129), QPoint(273, 188), QPoint(269, 75), QPoint(466, 61), QPoint(479, 141), QPoint(403, 99), QPoint(476, 253), QPoint(401, 403), QPoint(258, 470), QPoint(100, 400), QPoint(167, 263), QPoint(230, 382)});
//    borders.emplace_back(std::vector{QPoint(178, 129), QPoint(277, 191), QPoint(326, 303), QPoint(310, 28), QPoint(466, 61), QPoint(477, 96), QPoint(398, 107), QPoint(343, 172), QPoint(364, 259), QPoint(377, 201), QPoint(412, 147), QPoint(402, 270), QPoint(430, 186), QPoint(430, 271), QPoint(469, 187), QPoint(415, 124), QPoint(493, 170), QPoint(484, 295), QPoint(462, 261), QPoint(401, 403), QPoint(258, 470), QPoint(184, 476), QPoint(147, 387), QPoint(125, 479), QPoint(104, 394), QPoint(79, 479), QPoint(63, 88), QPoint(147, 326), QPoint(145, 94), QPoint(248, 377), QPoint(309, 402)});
//    borders.emplace_back(std::vector{QPoint(178, 129), QPoint(316, 117), QPoint(269, 75), QPoint(466, 61), QPoint(479, 141), QPoint(403, 99), QPoint(476, 253), QPoint(401, 403), QPoint(258, 470), QPoint(100, 400), QPoint(170, 295), QPoint(330, 306), QPoint(348, 162), QPoint(198, 155), QPoint(189, 262), QPoint(296, 279), QPoint(307, 212), QPoint(230, 193), QPoint(225, 242), QPoint(272, 250), QPoint(271, 231), QPoint(243, 230), QPoint(250, 210), QPoint(287, 219), QPoint(289, 267), QPoint(206, 252), QPoint(213, 174), QPoint(324, 196), QPoint(316, 294), QPoint(158, 272)});


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
