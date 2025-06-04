#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>

#ifndef QT_NO_OPENGL
#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "core/SculptProcess.h"
#include "renderer/RenderCapture.h"
#endif

std::shared_ptr<SculptProcess> scene = nullptr;
SceneWidget* sceneWidget = nullptr;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);


    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    QMainWindow window;
    window.setWindowTitle(QStringLiteral("Auto Carver - Convex Hull Algorithm Testing"));

    window.resize(1200, 700);

//    QSize screenSize = view->screen()->size();
//    container->setMinimumSize(QSize(500, 500));
//    container->setMaximumSize(screenSize);

    auto *content = new QWidget;
    window.setCentralWidget(content);

    auto *vLayout = new QVBoxLayout; // Highest-level layout for content
    vLayout->setAlignment(Qt::AlignTop);
    content->setLayout(vLayout);

    auto *render = new QWidget;
    render->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto *hRenderLayout = new QHBoxLayout; // Layout for render content
    render->setLayout(hRenderLayout);

    auto *control = new QWidget;
    auto *hControlLayout = new QHBoxLayout;
    control->setLayout(hControlLayout);

    vLayout->addWidget(render);
    vLayout->addWidget(control);

#ifndef QT_NO_OPENGL

    std::string source = R"(..\res\meshes\devil.obj)";
    auto model = MeshHandler::loadAsMeshBody(source);
    model->setBaseColor({1, 0, 1});

    scene = std::make_shared<SculptProcess>(model);
    sceneWidget = new SceneWidget(scene);

    hRenderLayout->addWidget(sceneWidget);

//    auto *sw2 = new SceneWidget(new SculptProcess(model));
//    hRenderLayout->addWidget(sw2);

//    auto *imageLabel = new QLabel();
//    imageLabel->setBackgroundRole(QPalette::Base);
//    imageLabel->setMinimumSize(QSize(100, 100));
//    hRenderLayout->addWidget(imageLabel);
#else
    QLabel note("OpenGL Support required");
    note.show();
#endif

    auto sculptureButton = new QCheckBox("Show sculpture", control);
    sculptureButton->setChecked(true);
    hControlLayout->addWidget(sculptureButton);

    QObject::connect(sculptureButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->show(1, Scene::Model::ALL);
        else sceneWidget->hide(1, Scene::Model::ALL);
        sceneWidget->update();
    });

    auto hullButton = new QCheckBox("Show convex hull", control);
    hControlLayout->addWidget(hullButton);

    QObject::connect(hullButton, &QCheckBox::clicked, [&](bool checked) {
        if (checked) sceneWidget->show(0, Scene::Model::HULL);
        else sceneWidget->hide(0, Scene::Model::HULL);
        sceneWidget->update();
    });

    auto stepButton = new QPushButton("Next step", control);
    hControlLayout->addWidget(stepButton);

    QObject::connect(stepButton, &QPushButton::clicked, [&]() {
        scene->next();
        sceneWidget->update();
    });

    auto captureButton = new QPushButton("Capture", control);
    hControlLayout->addWidget(captureButton);

    QObject::connect(captureButton, &QPushButton::clicked, [&]() {
//        m_rc->capture();
        if (sceneWidget != nullptr) {
            auto image = sceneWidget->grabFramebuffer();
//            auto img = new uint8_t[image.width()][image.height()];

            image.save("..\\out\\capture.png");
        }
    });

//    auto thread = std::thread(update);

    window.show();

    return app.exec();
}
