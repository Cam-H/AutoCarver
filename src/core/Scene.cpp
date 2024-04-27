//
// Created by cameronh on 25/04/24.
//

#include "Scene.h"

#include <QRandomGenerator>

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QPerVertexColorMaterial>

#include <Qt3DRender/QMesh>
#include <QGeometry>
#include <QAttribute>
#include <QUrl>
#include <QObject>
#include <QVector3D>
#include <iostream>
#include <vector>

#include "../geometry/Tesselation.h"
#include "../geometry/GeometryBuilder.h"

Scene::Scene(Qt3DCore::QEntity *rootEntity)
    : m_rootEntity(rootEntity)
    , m_frameAction(new Qt3DLogic::QFrameAction())
{

    // Cuboid shape data
    Qt3DExtras::QCuboidMesh *cuboid = new Qt3DExtras::QCuboidMesh();

    // CuboidMesh Transform
    Qt3DCore::QTransform *cuboidTransform = new Qt3DCore::QTransform();
    cuboidTransform->setScale(4.0f);
    cuboidTransform->setTranslation(QVector3D(5.0f, -4.0f, 0.0f));

    Qt3DExtras::QPhongMaterial *cuboidMaterial = new Qt3DExtras::QPhongMaterial();
    cuboidMaterial->setDiffuse(QColor(QRgb(0x665423)));

    //Cuboid
    {
        Qt3DCore::QEntity *m_cuboidEntity = new Qt3DCore::QEntity(m_rootEntity);
        m_cuboidEntity->addComponent(cuboid);
        m_cuboidEntity->addComponent(cuboidMaterial);
        m_cuboidEntity->addComponent(cuboidTransform);
    }

    Qt3DCore::QTransform *bunnyTransform = new Qt3DCore::QTransform();
    bunnyTransform->setTranslation(QVector3D(-10.0f, -4.0f, 0.0f));
    auto bunnyMaterial = new Qt3DExtras::QPerVertexColorMaterial();
//    bunnyMaterial->setDiffuse(QColor(QRgb(0xa69929)));

    Qt3DCore::QEntity *m_BunnyEntity = new Qt3DCore::QEntity(rootEntity);
    m_BunnyMesh = new Qt3DRender::QMesh();
    m_BunnyMesh->setSource(QUrl::fromLocalFile("/home/cameronh/CLionProjects/AutoCarver/res/icosphere.obj"));

    good = false;
    QObject::connect(m_BunnyMesh, &Qt3DRender::QMesh::statusChanged, this, [this](Qt3DRender::QMesh::Status status) {
        std::cout << status << "\n";
        if (status == 2){
            good = true;

            m_tessel = Tesselation();
            GeometryBuilder::add(&m_tessel, m_BunnyMesh->geometry());

            auto colors = std::vector<QVector3D>(m_tessel.getTriangleCount());
            for (auto & color : colors) {
                color = {QRandomGenerator::global()->bounded(100) / 100.0f
                             , QRandomGenerator::global()->bounded(100) / 100.0f
                             , QRandomGenerator::global()->bounded(100) / 100.0f
                };
            }

            Qt3DCore::QGeometry *geo = GeometryBuilder::convert(m_tessel, colors);
            geo->setParent(m_BunnyMesh);
            m_BunnyMesh->setGeometry(geo);
        }
    });

    m_BunnyEntity->addComponent(m_BunnyMesh);
    m_BunnyEntity->addComponent(bunnyMaterial);
    m_BunnyEntity->addComponent(bunnyTransform);

//    QObject::connect(m_frameAction, &Qt3DLogic::QFrameAction::triggered, this, [this] (float dt) {
//        static float f = 0;
//        if (good) {
//            for (auto *attrib : m_BunnyMesh->geometry()->attributes()){
////                std::cout << "Write: " << attrib->name().toStdString() << " | " << attrib->buffer() << " " << f << "\n";
//                if (attrib->name() == "vertexPosition") {
//                    auto buffer = attrib->buffer();
//                    buffer->updateData(0, QByteArray(reinterpret_cast<const char*>(&f), sizeof(f)));
//                    f+=0.05f;
//                }
//            }
//        }
//    });

    m_BunnyEntity->addComponent(m_frameAction);

}

Scene::~Scene()
{
}