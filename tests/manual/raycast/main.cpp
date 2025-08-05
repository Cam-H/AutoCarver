#include <QApplication>
#include <QSurfaceFormat>
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>

#include <QFile>
#include <QDir>

#include <memory>
#include "physics/RigidBody.h"

#include "renderer/SceneWidget.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"
#include "geometry/primitives/Plane.h"

#include "renderer/UiLoader.h"

#include "core/Timer.h"

std::shared_ptr<Scene> scene = nullptr;
SceneWidget *sceneWidget = nullptr;

std::shared_ptr<RigidBody> last = nullptr;

static QWidget *loadUiFile(QWidget *parent)
{
    std::cout << "Current directory: |" << QDir::currentPath().toStdString() << "|\n";
    QFile file("../tests/manual/raycast/main.ui");
    file.open(QFile::ReadOnly);

    UiLoader builder;

    auto *widget = builder.load(&file, parent);

    file.close();

    return widget;
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    app.setApplicationName("cube");
    app.setApplicationVersion("0.1");

    auto *widget = loadUiFile(nullptr);
    if (widget == nullptr) return -1;

    scene = std::make_shared<Scene>();

    auto base = MeshBuilder::box();
    scene->createBody(base);

    base = MeshBuilder::plane(Plane({ 0, -5, 0 }, { 0, 1, 0 }), 5);
    scene->createBody(base);

    base = MeshBuilder::icosphere(1.0, 3);
    auto nb = scene->createBody(base);
    nb->translate({ 5, 0, 0 });

    std::string source = R"(..\res\meshes\devil.obj)";
    base = MeshHandler::loadAsMeshBody(source);
    base->normalize();
    base->zero();
    nb = scene->createBody(base);
    nb->translate({ 0, 3, 0});
    nb->rotate({ 1, 0, 0 }, M_PI / 3);


    sceneWidget = widget->findChild<SceneWidget*>("sceneWidget");
    sceneWidget->setScene(scene);

    QObject::connect(sceneWidget, &SceneWidget::mousepick, [&](Ray ray) {
        auto [body, t] = scene->raycast(ray);
        if (body != nullptr) {
            body->mesh()->setFaceColor(body->mesh()->baseColor());
            auto [hit, tDup, faceIdx] = body->mesh()->pickFace(glm::inverse(body->getTransform()) * ray);
            if (hit) body->mesh()->setFaceColor(faceIdx, { 0, 1, 0 });
            else body->mesh()->setFaceColor({ 0, 0, 1 });

            sceneWidget->updateRenderGeometry(body->mesh());
        }

        if (body != last) {
            if (last != nullptr) {
                last->mesh()->setFaceColor(last->mesh()->baseColor());
                sceneWidget->updateRenderGeometry(last->mesh());
            }

            last = body;
        }

        sceneWidget->update();
    });

//    const QMetaObject *meta = sceneWidget->metaObject();
//    for (int i = 0; i < meta->methodCount(); ++i) {
//        qDebug() << meta->method(i).methodSignature();
//    }

    widget->show();

    return app.exec();
}
