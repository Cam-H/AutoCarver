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
#include "geometry/poly/CompositePolygon.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    // Test data set
    uint32_t idx = 0;
    std::vector<CompositePolygon> polygons;


    // Flame I
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(305.000000, 340.000000), QVector2D(319.026001, 340.000000), QVector2D(342.794006, 329.596008), QVector2D(368.294006, 288.187988), QVector2D(391.759003, 291.712006), QVector2D(403.009003, 302.872986), QVector2D(414.186005, 302.029999), QVector2D(417.878998, 355.549988), QVector2D(417.181000, 362.664978), QVector2D(422.727997, 363.583008), QVector2D(424.726990, 369.088013), QVector2D(422.903992, 388.690979), QVector2D(433.859009, 372.350006), QVector2D(435.858002, 377.856018), QVector2D(441.404999, 378.773987), QVector2D(455.221008, 345.057983), QVector2D(413.304993, 253.553009), QVector2D(401.951996, 244.700989), QVector2D(390.526978, 223.844498), QVector2D(366.885986, 210.624207), QVector2D(350.785004, 242.688995), QVector2D(327.903992, 243.436996), QVector2D(314.765991, 233.780701), QVector2D(298.540100, 269.747009), QVector2D(163.664703, 221.801498), QVector2D(202.718811, 203.487900), QVector2D(204.829666, 183.734695), QVector2D(217.261108, 183.852905), QVector2D(239.434799, 173.567001), QVector2D(246.881500, 144.721924), QVector2D(268.774109, 148.539001), QVector2D(278.451996, 159.992096), QVector2D(288.056885, 159.441101), QVector2D(275.131897, 150.881104), QVector2D(262.134094, 130.317017), QVector2D(236.921402, 117.389000), QVector2D(230.229904, 135.886963), QVector2D(205.754913, 136.752792), QVector2D(117.341797, 109.880402), QVector2D(79.407997, 157.404297), QVector2D(62.464005, 173.148102), QVector2D(56.395996, 190.156998), QVector2D(56.572998, 205.537109), QVector2D(93.482002, 231.640808), QVector2D(103.292999, 249.643997), QVector2D(134.482300, 254.524994), QVector2D(92.636002, 298.554016), QVector2D(126.312500, 326.876007)}
    });

    // Flame II
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(250.000000, 200.000000), QVector2D(289.457001, 200.000000), QVector2D(286.057983, 184.052994), QVector2D(296.148987, 163.503998), QVector2D(331.998993, 162.014008), QVector2D(329.669983, 135.028000), QVector2D(315.356995, 116.036003), QVector2D(311.957001, 102.046997), QVector2D(245.783691, 26.210100), QVector2D(209.369003, 24.939400), QVector2D(207.925201, 30.372801), QVector2D(212.121307, 34.459301), QVector2D(192.721497, 37.724998), QVector2D(211.339600, 44.124802), QVector2D(215.535706, 48.211201), QVector2D(214.092010, 53.644600), QVector2D(220.863800, 55.934101), QVector2D(268.130005, 81.309700), QVector2D(262.765015, 91.151100), QVector2D(268.312988, 105.995003), QVector2D(261.876007, 128.834000), QVector2D(213.644806, 135.048996), QVector2D(194.386993, 152.436996), QVector2D(188.619095, 165.222000), QVector2D(127.100700, 333.497986), QVector2D(139.067795, 375.842010), QVector2D(196.410202, 355.803986), QVector2D(188.033600, 386.242004), QVector2D(200.409500, 402.587006), QVector2D(209.025497, 446.966003), QVector2D(222.972107, 453.451996), QVector2D(240.971802, 454.915009), QVector2D(262.291016, 445.945007), QVector2D(321.210022, 430.910004), QVector2D(333.072998, 339.268005), QVector2D(343.927002, 317.313995), QVector2D(363.539978, 318.821991), QVector2D(362.123993, 290.523010), QVector2D(348.723999, 270.218994), QVector2D(346.237000, 254.917007), QVector2D(341.785004, 263.446014), QVector2D(348.244995, 276.976990), QVector2D(342.721008, 298.502991), QVector2D(313.365997, 293.428986), QVector2D(294.871002, 309.411011), QVector2D(289.867004, 320.790985), QVector2D(270.992981, 314.592010), QVector2D(238.239502, 342.660004)}
    });

    // Square
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(100, 400), QVector2D(400, 400), QVector2D(400, 100)}
    });

    // Square w/ diamond
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(100, 400), QVector2D(400, 400), QVector2D(400, 100)},
            std::vector<QVector2D>{QVector2D(250, 150), QVector2D(350, 250), QVector2D(250, 350), QVector2D(150, 250)}
    });

    // Square w/ square
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(100, 400), QVector2D(400, 400), QVector2D(400, 100)},
            std::vector<QVector2D>{QVector2D(150, 150), QVector2D(350, 150), QVector2D(350, 350), QVector2D(150, 350)}
    });

    // Multi-hole square
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 100), QVector2D(100, 400), QVector2D(400, 400), QVector2D(400, 100)},
            std::vector<QVector2D>{QVector2D(250, 150), QVector2D(350, 230), QVector2D(250, 210), QVector2D(150, 230)},
            std::vector<QVector2D>{QVector2D(250, 350), QVector2D(150, 270), QVector2D(250, 290), QVector2D(350, 270)}
    });

    // Emblem
    polygons.emplace_back(std::vector<std::vector<QVector2D>>{
            std::vector<QVector2D>{QVector2D(100, 350), QVector2D(432.901, 350), QVector2D(432.901, 17.0992), QVector2D(100, 17.0992)},
            std::vector<QVector2D>{QVector2D(372.519, 273.261), QVector2D(372.519, 93.8377), QVector2D(397.287, 69.0689), QVector2D(397.287, 298.03)},
            std::vector<QVector2D>{QVector2D(207.219, 242.781), QVector2D(207.219, 124.318), QVector2D(325.682, 124.318), QVector2D(325.682, 242.781)},
            std::vector<QVector2D>{QVector2D(124.048, 41.147), QVector2D(408.853, 41.147), QVector2D(360.953, 89.0471), QVector2D(171.948, 89.0471)},
            std::vector<QVector2D>{QVector2D(408.853, 325.952), QVector2D(124.048, 325.952), QVector2D(171.948, 278.052), QVector2D(360.953, 278.052)}
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

//    Polygon poly = Polygon({
//        {{0, 0}, {0, 10}, {10, 10}, {10, 0}},
//        {{2, 2}, {8, 2}, {8, 8}, {2, 8}}
//    });
//
//    std::cout << "Area: " << poly.area() << "\n";

    container->show();

    return app.exec();
}
