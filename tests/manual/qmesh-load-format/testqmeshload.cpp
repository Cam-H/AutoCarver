//
// Created by cameronh on 25/04/24.
//

#include <QTest>
#include <QThread>

#include <QEntity>

#include <Qt3DRender/QMesh>
#include <Qt3DRender/QRenderSettings>

#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/qforwardrenderer.h>

#include <iostream>

class TestQMeshLoad: public QObject
{
    Q_OBJECT

private slots:
    void objFormat();

private:
    Qt3DRender::QMesh *mesh;
    bool hold;
};
//! [0]

//! [1]
void TestQMeshLoad::objFormat()
{
    Qt3DCore::QEntityPtr entity(new Qt3DCore::QEntity());
    Qt3DRender::QRenderSettings *settings = new Qt3DRender::QRenderSettings(entity.data());
    entity->addComponent(settings);

    mesh = new Qt3DRender::QMesh();
    mesh->setSource(QUrl::fromLocalFile("/home/cameronh/CLionProjects/AutoCarver/res/bunny.obj"));
    entity->addComponent(mesh);

    // TODO manual trigger render job - QMesh does not start loading without a window
    hold = true;
    QObject::connect(mesh, &Qt3DRender::QMesh::statusChanged, this, [this](Qt3DRender::QMesh::Status status) {
        std::cout << status << "\n";
        hold = status <  2;
    });

    QTest::qWaitFor([this]() { return !hold; }, 3000);

    for (auto *attrib : mesh->geometry()->attributes()){
        std::cout << "Attrib: " << attrib->name().toStdString() << " " << attrib->count() << " " << attrib->attributeType() << "\n";
    }

    QString str = "Hello";
    QCOMPARE(str.toUpper(), QString("HELLO"));
}
//! [1]

//! [2]
QTEST_MAIN(TestQMeshLoad)
#include "testqmeshload.moc"