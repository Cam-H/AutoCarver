//
// Created by cameronh on 25/04/24.
//

#include "Scene.h"

#include <QRandomGenerator>

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QDiffuseSpecularMaterial>
#include <Qt3DExtras/QPerVertexColorMaterial>
#include <Qt3DRender/QRenderStateSet>
#include <Qt3DRender/QCullFace>

#include <QLine>

#include <Qt3DRender/QMesh>
#include <QGeometry>
#include <QAttribute>
#include <QUrl>
#include <QObject>
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
    cuboidTransform->setTranslation(QVector3D(5.0f, -4.0f, -10.0f));

    Qt3DExtras::QPhongMaterial *cuboidMaterial = new Qt3DExtras::QPhongMaterial();
    cuboidMaterial->setDiffuse(QColor(QRgb(0x665423)));

    //Cuboid
    {
        Qt3DCore::QEntity *m_cuboidEntity = new Qt3DCore::QEntity(m_rootEntity);
        m_cuboidEntity->addComponent(cuboid);
        m_cuboidEntity->addComponent(cuboidMaterial);
        m_cuboidEntity->addComponent(cuboidTransform);
    }

    auto *plane = new Qt3DExtras::QCuboidMesh();
    m_cutPlaneTransform = new Qt3DCore::QTransform();
    m_cutPlaneTransform->setScale3D(QVector3D{16.0f, 0.01f, 16.0f});
    m_plane = new Qt3DCore::QEntity(m_rootEntity);
    auto *planeMaterial = new Qt3DExtras::QDiffuseSpecularMaterial();
    planeMaterial->setAmbient(QColor::fromRgbF(0, 0, 1.0f, 0.5f));
    planeMaterial->setDiffuse(QColor(QRgb(0x000000)));
    planeMaterial->setSpecular(QColor(QRgb(0x000000)));
//    planeMaterial->setAlphaBlendingEnabled(true);
    m_plane->addComponent(plane);
    m_plane->addComponent(planeMaterial);
    m_plane->addComponent(m_cutPlaneTransform);




    auto *renderStateSet = new Qt3DRender::QRenderStateSet();

// Create a front face culling render state
    auto *cullFront = new Qt3DRender::QCullFace();
    cullFront->setMode(Qt3DRender::QCullFace::NoCulling);

// Add the render state to the render pass
    renderStateSet->addRenderState(cullFront);

    Qt3DCore::QTransform *bunnyTransform = new Qt3DCore::QTransform();
    bunnyTransform->setTranslation(QVector3D(0.0f, -2.0f, 0.0f));
    auto bunnyMaterial = new Qt3DExtras::QPerVertexColorMaterial();
//    bunnyMaterial->setDiffuse(QColor(QRgb(0xa69929)));

    Qt3DCore::QEntity *m_BunnyEntity = new Qt3DCore::QEntity(rootEntity);
    m_BunnyMesh = new Qt3DRender::QMesh();
    m_BunnyMesh->setSource(QUrl::fromLocalFile("/home/cameronh/CLionProjects/AutoCarver/res/icosphere.obj"));
//    bunnyTransform->setTranslation(QVector3D(-10.0f, -4.0f, -400.0f));

    good = false;
    QObject::connect(m_BunnyMesh, &Qt3DRender::QMesh::statusChanged, this, [this](Qt3DRender::QMesh::Status status) {
        std::cout << status << "\n";
        if (status == 2){
//            good = true;

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

    m_set = std::vector<QVector3D>(100);
    for (auto &color : m_set) {
        color = {QRandomGenerator::global()->bounded(100) / 100.0f
                , QRandomGenerator::global()->bounded(100) / 100.0f
                , QRandomGenerator::global()->bounded(100) / 100.0f
        };
    }

    QObject::connect(m_frameAction, &Qt3DLogic::QFrameAction::triggered, this, [this] (float dt) {
        static int c = 0;
        if (good) {
//            if (c++ < 100) {
//                return;
//            }
//
            auto colors = std::vector<QVector3D>(m_tessel.getTriangleCount());
            auto res = m_tessel.horizon(m_apex.normalized());
            for (uint32_t i = 0; i < colors.size(); i++) {
                colors[i] = m_set[res[i]];
            }

            Qt3DCore::QGeometry *geo = GeometryBuilder::convert(m_tessel, colors);
            geo->setParent(m_BunnyMesh);
            m_BunnyMesh->setGeometry(geo);

            good = false;
            c = 0;
        }
    });

    m_BunnyEntity->addComponent(m_frameAction);

}

Scene::~Scene()
{
}

void Scene::apexX(int value)
{
    m_apex.setX(value != 0 ? value : 1);
    good = true;
}

void Scene::apexY(int value)
{
    m_apex.setY(value);
    good = true;
}

void Scene::apexZ(int value)
{
    m_apex.setZ(value);
    good = true;
}

void Scene::cut(int value)
{
    QVector3D origin = {0, value / 10.0f, 0};
    m_cutPlaneTransform->setTranslation(origin);

    Tesselation tessel;
    m_tessel.slice(origin + QVector3D{0, 2, 0}, {0, 1, 0}, tessel);
    std::cout << "COMPARE: " << m_tessel.getTriangleCount() << " " << tessel.getTriangleCount() << " " << m_tessel.getVertexCount() << " " << tessel.getVertexCount() << "\n";

    m_BunnyMesh->setEnabled(tessel.getTriangleCount());

    if (tessel.getTriangleCount() == 0) {
        return;
    }

//    std::cout << tessel.getTriangleCount() << " " << tessel.getVertexCount() << "\n";
    std::vector<QVector3D> colors(tessel.getTriangleCount(), {1, 1, 1});
    for (auto & color : colors) {
        color = {QRandomGenerator::global()->bounded(100) / 100.0f
                , QRandomGenerator::global()->bounded(100) / 100.0f
                , QRandomGenerator::global()->bounded(100) / 100.0f
        };
    }
    Qt3DCore::QGeometry *geo = GeometryBuilder::convert(tessel, colors);
    geo->setParent(m_BunnyMesh);
    m_BunnyMesh->setGeometry(geo);
}

void Scene::show(bool enabled)
{
    m_plane->setEnabled(enabled);
}