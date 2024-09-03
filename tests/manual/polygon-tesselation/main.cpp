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
    std::vector<std::vector<QPoint>> borders;
    borders.emplace_back(std::vector{QPoint(100, 100), QPoint(400, 100), QPoint(400, 400), QPoint(100, 400)});
    borders.emplace_back(std::vector{QPoint(100, 100), QPoint(254, 266), QPoint(338, 81), QPoint(274, 337), QPoint(466, 92), QPoint(476, 423), QPoint(319, 331), QPoint(221, 452), QPoint(161, 257), QPoint(100, 400)});    borders.emplace_back(std::vector{QPoint(110, 88), QPoint(249, 36), QPoint(403, 99), QPoint(476, 253), QPoint(401, 403), QPoint(258, 470), QPoint(100, 400), QPoint(28, 235)});
    borders.emplace_back(std::vector{QPoint(100, 100), QPoint(402, 72), QPoint(446, 96), QPoint(390, 166), QPoint(482, 227), QPoint(400, 400), QPoint(171, 319), QPoint(191, 165), QPoint(240, 303), QPoint(328, 329), QPoint(253, 178), QPoint(265, 129), QPoint(346, 223), QPoint(365, 118)});    borders.emplace_back(std::vector{QPoint(178, 129), QPoint(273, 188), QPoint(269, 75), QPoint(466, 61), QPoint(479, 141), QPoint(403, 99), QPoint(476, 253), QPoint(401, 403), QPoint(258, 470), QPoint(100, 400), QPoint(167, 263), QPoint(230, 382)});
    borders.emplace_back(std::vector{QPoint(178, 129), QPoint(277, 191), QPoint(326, 303), QPoint(310, 28), QPoint(466, 61), QPoint(477, 96), QPoint(398, 107), QPoint(343, 172), QPoint(364, 259), QPoint(377, 201), QPoint(412, 147), QPoint(402, 270), QPoint(430, 186), QPoint(430, 271), QPoint(469, 187), QPoint(415, 124), QPoint(493, 170), QPoint(484, 295), QPoint(462, 261), QPoint(401, 403), QPoint(258, 470), QPoint(184, 476), QPoint(147, 387), QPoint(125, 479), QPoint(104, 394), QPoint(79, 479), QPoint(63, 88), QPoint(147, 326), QPoint(145, 94), QPoint(248, 377), QPoint(309, 402)});
    borders.emplace_back(std::vector{QPoint(178, 129), QPoint(316, 117), QPoint(269, 75), QPoint(466, 61), QPoint(479, 141), QPoint(403, 99), QPoint(476, 253), QPoint(401, 403), QPoint(258, 470), QPoint(100, 400), QPoint(170, 295), QPoint(330, 306), QPoint(348, 162), QPoint(198, 155), QPoint(189, 262), QPoint(296, 279), QPoint(307, 212), QPoint(230, 193), QPoint(225, 242), QPoint(272, 250), QPoint(271, 231), QPoint(243, 230), QPoint(250, 210), QPoint(287, 219), QPoint(289, 267), QPoint(206, 252), QPoint(213, 174), QPoint(324, 196), QPoint(316, 294), QPoint(158, 272)});

    QWidget *container = new QWidget;

    QVBoxLayout *vLayout = new QVBoxLayout(container);
//    vLayout->setAlignment(Qt::AlignTop);

    Window *window = new Window;
    window->setBorder(borders[0]);
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
        window->setBorder(borders[idx]);
    });

    QClipboard *clipboard = QGuiApplication::clipboard();
    QObject::connect(copyButton, &QPushButton::clicked, [=]() {
        const std::vector<QPoint> &border = window->getBorder();
        std::string text = "borders.emplace_back(std::vector{";
        for (uint32_t i = 0; i < border.size(); i++) {
            text = text + "QPoint(" + std::to_string(border[i].x()) + ", " + std::to_string(border[i].y()) + ")";
            if (i + 1 < border.size()) text = text + ", ";
        }
        text = text + "});";
        clipboard->setText(QString(text.c_str()));
    });

    QObject::connect(nextButton, &QPushButton::clicked, [&]() {
        if (idx < borders.size() - 1) idx++;
        window->setBorder(borders[idx]);
    });

    auto partitionButton = new QCheckBox("Partition", container);
    partitionButton->setChecked(true);
    vLayout->addWidget(partitionButton);

    auto tesselateButton = new QCheckBox("Tesselate", container);
    vLayout->addWidget(tesselateButton);

    QObject::connect(partitionButton, &QCheckBox::clicked, [&](bool checked) {
        window->enablePartition(checked);
        tesselateButton->setChecked(tesselateButton->isChecked() && checked);
        tesselateButton->setEnabled(checked);
    });

    QObject::connect(tesselateButton, &QCheckBox::clicked, [&](bool checked) {
        window->enableTesselation(true);
    });

    container->show();

    return app.exec();
}
